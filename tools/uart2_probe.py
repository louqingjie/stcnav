#!/usr/bin/env python3
"""UART2（P1.0 = RxD2 / P1.1 = TxD2）收发探测。

配合 apps/uart2_echo 固件使用：

  接线（3.3 V TTL，必须共地）：
    USB-TTL TXD -> P1.0（MCU RX）
    USB-TTL RXD -> P1.1（MCU TX）
    USB-TTL GND <-> MCU GND

  用法：
    python tools/uart2_probe.py --port COM3                  # 默认 115200
    python tools/uart2_probe.py --port COM3 -b 230400        # uart2_echo_230400
    python tools/uart2_probe.py --port COM3 --autobaud       # 收不到横幅时扫描波特率
    python tools/uart2_probe.py --port COM3 --listen 5       # 只监听并打印

判定：
  TX(MCU->PC)：能收到 "U2 READY ..." 或 "U2 HB ..." 心跳；
  RX(PC->MCU)：固件回显的字节流与发送内容逐字节一致，且心跳里的 rx 计数增加。
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
import time

# 管道/重定向时 Windows 默认按本地代码页编码，统一成 UTF-8 便于日志收集。
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")

# 固件按 FOSC=35 MHz 计算 Timer2 分频；实际 IRC 不同则波特率按比例偏移。
FIRMWARE_FOSC_MHZ = 35.0
IRC_CANDIDATES_MHZ = (
    11.0592, 12, 20, 22.1184, 24, 24.576, 27, 30, 33.1776, 35, 36, 40, 45,
)

LINE_READY = re.compile(
    rb"U2 READY baud=(?P<baud>\d+) fosc=(?P<fosc>\d+)"
    rb"(?: hb=(?P<hb>\d+) rx=(?P<rx>\d+))?"
)
LINE_HB = re.compile(
    rb"U2 HB (?P<hb>\d+) rx=(?P<rx>\d+) uptime=(?P<uptime>\d+)"
)


def rx_from_match(match) -> int | None:
    if match is None:
        return None
    value = match.group("rx")
    return int(value) if value is not None else None


def decode_line(line: bytes) -> str:
    return line.decode("latin-1", errors="replace").strip()


def preview(data: bytes, limit: int = 48) -> str:
    chunk = data[:limit]
    text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
    return f"{text}  [{chunk.hex(' ')}]"


def pick_port(explicit: str | None) -> str:
    ports = list(list_ports.comports())
    if explicit:
        return explicit

    usb = [p for p in ports if p.vid is not None]
    ch340 = [
        p for p in usb
        if p.vid == 0x1A86 or "CH340" in (p.description or "").upper()
    ]
    if len(ch340) == 1:
        return ch340[0].device
    if len(usb) == 1:
        return usb[0].device

    names = "、".join(f"{p.device}({p.description})" for p in ports) or "无"
    raise SystemExit(
        f"无法自动确定串口（检测到：{names}）。请用 --port 指定，例如 --port COM3。"
    )


class Port:
    def __init__(self, device: str, baud: int) -> None:
        self.ser = serial.Serial(
            device,
            baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
        self.buffer = bytearray()
        self.ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def pump(self) -> bytes:
        chunk = self.ser.read(256)
        if chunk:
            self.buffer.extend(chunk)
        return chunk

    def read_exact(self, size: int, timeout: float) -> bytes:
        out = bytearray()
        deadline = time.time() + timeout
        while len(out) < size and time.time() < deadline:
            if self.buffer:
                take = min(size - len(out), len(self.buffer))
                out.extend(self.buffer[:take])
                del self.buffer[:take]
                continue
            self.pump()
        return bytes(out)

    def wait_line(self, pattern: re.Pattern, timeout: float):
        """读取行，返回匹配 pattern 的 match；不匹配的行直接丢弃。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            index = self.buffer.find(b"\n")
            if index >= 0:
                line = bytes(self.buffer[:index + 1])
                del self.buffer[:index + 1]
                match = pattern.search(line)
                if match:
                    return match, line
                continue
            self.pump()
        return None, b""

    def wait_status(self, timeout: float):
        """同时等待上电横幅与心跳，返回 (kind, match, line)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            index = self.buffer.find(b"\n")
            if index >= 0:
                line = bytes(self.buffer[:index + 1])
                del self.buffer[:index + 1]
                match = LINE_READY.search(line)
                if match:
                    return "ready", match, line
                match = LINE_HB.search(line)
                if match:
                    return "hb", match, line
                continue
            self.pump()
        return None, None, b""

    def drain(self) -> None:
        time.sleep(0.02)
        self.ser.reset_input_buffer()
        self.buffer.clear()


def build_payload(size: int) -> bytes:
    if size < 256:
        size = 256
    repeats = size // 256 + 1
    return (bytes(range(256)) * repeats)[:size]


def echo_test(port: Port, payload: bytes, chunk: int, chunk_timeout: float):
    matched = 0
    mismatch = 0
    lost = 0
    first_bad = None
    rates: list[float] = []

    for offset in range(0, len(payload), chunk):
        block = payload[offset:offset + chunk]
        try:
            port.ser.write(block)
            port.ser.flush()
        except serial.SerialException as error:
            raise SystemExit(f"串口写入失败：{error}") from error

        head = port.read_exact(1, chunk_timeout)
        if not head:
            lost += len(block)
            continue

        start = time.perf_counter()
        tail = port.read_exact(len(block) - 1, chunk_timeout)
        elapsed = time.perf_counter() - start
        if len(tail) == len(block) - 1 and len(block) > 2:
            rates.append((len(block) - 1) / max(elapsed, 1e-6))

        received = head + tail
        if received == block:
            matched += len(block)
            continue

        for index, expected in enumerate(block):
            if index >= len(received):
                lost += 1
            elif received[index] != expected:
                mismatch += 1
                if first_bad is None:
                    first_bad = (offset + index, expected, received[index])
        # 丢弃本块可能的余留字节，避免下一块比较错位。
        port.drain()

    return {
        "matched": matched,
        "mismatch": mismatch,
        "lost": lost,
        "first_bad": first_bad,
        "bytes_per_second": statistics.median(rates) if rates else 0.0,
    }


def probe_once(device: str, args) -> bool:
    print(f"[probe] 串口 {device} @ {args.baud} 8N1，等待固件横幅/心跳 ...")
    try:
        port = Port(device, args.baud)
    except serial.SerialException as error:
        print(f"[probe] 打开失败：{error}")
        return False

    try:
        kind, match, line = port.wait_status(args.wait)
        if kind == "ready":
            print(f"[probe] TX(MCU->PC): PASS  状态行: {decode_line(line)}")
        elif kind == "hb":
            print(f"[probe] TX(MCU->PC): PASS  心跳: {decode_line(line)}")
            print("[probe] 状态行为兼容旧固件格式，不影响结论。")
        else:
            captured = bytes(port.buffer)
            port.buffer.clear()
            if captured:
                print(f"[probe] TX(MCU->PC): FAIL  收到 {len(captured)} 字节但无法解析：")
                print(f"        {preview(captured)}")
            else:
                print("[probe] TX(MCU->PC): FAIL  串口没有任何数据")
            return False

        rx_baseline = rx_from_match(match) or 0

        # 回显测试前清空接收缓冲，避免横幅/心跳混进回显流。
        port.drain()

        payload = build_payload(args.bytes)
        chunk = max(8, min(args.chunk, 128))
        chunk_timeout = max(0.2, chunk * 10.0 / args.baud * 4.0)

        started = time.time()
        stats = echo_test(port, payload, chunk, chunk_timeout)
        elapsed = max(time.time() - started, 1e-6)

        total = len(payload)
        print(
            f"[probe] 回显: 发送 {total} B，匹配 {stats['matched']} B，"
            f"错误 {stats['mismatch']} B，丢失 {stats['lost']} B，"
            f"耗时 {elapsed:.2f} s"
        )
        if stats["first_bad"]:
            offset, expected, actual = stats["first_bad"]
            print(
                f"[probe] 首个不一致：偏移 {offset}，发送 0x{expected:02X}，"
                f"收到 0x{actual:02X}"
            )

        if stats["bytes_per_second"] > 0:
            measured = stats["bytes_per_second"] * 10.0
            ratio = measured / args.baud
            if 0.2 <= ratio <= 5.0:
                print(
                    f"[probe] 实测 MCU TX 速率 ≈ {measured:.0f} baud"
                    f"（配置 {args.baud}，比值 {ratio:.3f}）"
                )
                if abs(ratio - 1.0) > 0.02:
                    estimate = FIRMWARE_FOSC_MHZ * ratio
                    print(
                        f"[probe] 警告：实测波特率与配置偏差 "
                        f"{abs(ratio - 1.0) * 100:.1f}%，"
                        f"实际 IRC ≈ {estimate:.2f} MHz。"
                        "固件按 FOSC=35 MHz 计算分频，请确认 STC-ISP 下载时 IRC 选 "
                        "35 MHz；否则用 -DFOSC=<实际频率> 重新编译。"
                    )
            else:
                print(
                    f"[probe] 速率测量失真（≈{measured:.0f} baud），忽略；"
                    "USB-TTL 会把字节攒包再上报，单块计时不可靠。"
                )

        rx_after = None
        kind, match, line = port.wait_status(2.5)
        rx_after = rx_from_match(match)

        echo_ok = (
            stats["matched"] == total
            and stats["mismatch"] == 0
            and stats["lost"] == 0
        )
        print(f"[probe] 回显判定: {'PASS 逐字节一致' if echo_ok else 'FAIL'}")

        delta = None
        if rx_after is not None:
            delta = (rx_after - rx_baseline) & 0xFFFF
            print(f"[probe] MCU rx 计数 +{delta}（本次发送 {total} B）")

        print()
        if echo_ok:
            print("[probe] 结论：P1.0(RxD2)/P1.1(TxD2) 双向收发正常。")
            return True

        print("[probe] 结论：回显失败，RX/TX 至少有一路不通。")
        if delta == total:
            print("        MCU 收到了全部字节但没有回显：确认烧录的是 uart2_echo 固件；")
            print("        若固件正确，检查 MCU 是否停在中断/异常里（重新上电再试）。")
        elif delta is not None:
            print(
                f"        MCU 只收到 {delta}/{total} 字节："
                "检查 USB-TTL TXD -> P1.0 接线、共地，以及 P1.0 是否被外部拉死。"
            )
        else:
            print("        未取到心跳 rx 计数：检查 USB-TTL TXD -> P1.0 接线与共地。")
        return False
    finally:
        port.close()


def autobaud(device: str, args) -> bool:
    requested = args.baud
    candidates = {requested}
    for mhz in IRC_CANDIDATES_MHZ:
        candidates.add(int(round(requested * mhz / FIRMWARE_FOSC_MHZ)))
    candidates.update((9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600))
    candidates = sorted(baud for baud in candidates if 300 <= baud <= 2_000_000)

    print(
        f"[probe] {requested} 波特率下没有 U2 横幅/心跳，"
        f"开始扫描 {len(candidates)} 个候选波特率 ..."
    )
    hits = []
    for baud in candidates:
        try:
            with serial.Serial(device, baud, timeout=0.05) as ser:
                ser.reset_input_buffer()
                buf = bytearray()
                deadline = time.time() + 1.5
                while time.time() < deadline:
                    chunk = ser.read(256)
                    if chunk:
                        buf.extend(chunk)
        except serial.SerialException as error:
            print(f"[probe]   {baud}: 打不开（{error}）")
            continue

        if b"U2 READY" in buf or b"U2 HB" in buf:
            hits.append(baud)
            print(f"[probe]   命中 {baud} baud: {preview(bytes(buf))}")

    if not hits:
        print("[probe] 所有候选波特率都没有 U2 数据。")
        print("        1) MCU 是否已上电并运行 uart2_echo 固件？")
        print("        2) USB-TTL RXD 是否接到 P1.1（MCU TX）？GND 是否共地？")
        print("        3) 板子是否还在 ISP 下载模式（需要断电重启）？")
        return False

    for baud in hits:
        ratio = baud / requested
        print(
            f"[probe] 命中 {baud}：相对请求 {requested} 的比值 {ratio:.3f}"
            f"（对应 IRC ≈ {FIRMWARE_FOSC_MHZ * ratio:.2f} MHz）"
        )
    print(
        "[probe] 可能是固件按其它波特率编译（例如 uart2_echo_230400），"
        "也可能实际 IRC 与 FOSC=35 MHz 不一致。"
    )
    print("[probe] 用命中的波特率重跑可完成回显判定。")
    return False


def loopback(device: str, baud: int) -> int:
    """USB-TTL 自环测试：把适配器的 TXD 与 RXD 直接短接后运行。"""
    print(f"[probe] USB-TTL 自环测试 {device} @ {baud} 8N1")
    print("[probe] 请先拔掉与板子相连的 TXD/RXD，并把这两根线直接短接（GND 不用管）")
    payload = bytes(range(256)) * 2 + b"\r\nU2 LOOPBACK\r\n"
    try:
        with serial.Serial(device, baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            ser.write(payload)
            ser.flush()
            got = bytearray()
            deadline = time.time() + 1.0
            while time.time() < deadline and len(got) < len(payload):
                chunk = ser.read(512)
                if chunk:
                    got.extend(chunk)
    except serial.SerialException as error:
        print(f"[probe] 打开失败：{error}")
        return 2

    if bytes(got) == payload:
        print(f"[probe] 自环 PASS：{len(payload)} 字节原样返回，USB-TTL 与驱动正常。")
        print("[probe] 那么问题在板子一侧：接线（TXD->P1.0 / RXD->P1.1）或固件未运行。")
        return 0

    print(
        f"[probe] 自环 FAIL：发出 {len(payload)} B，收回 {len(got)} B"
    )
    if got:
        print(f"        {preview(bytes(got))}")
    else:
        print("        没有收回任何字节。")
    print("        1) 确认短接的是适配器自己的 TXD 与 RXD（不是接到板子的两根）；")
    print("        2) 确认 CH340 驱动正常、换一根 USB 数据线/换 USB 口；")
    print("        3) 有的模块是 3.3V/5V 跳线，确认供电跳线正确。")
    return 1


def listen(device: str, baud: int, seconds: float) -> int:
    print(f"[probe] 监听 {device} @ {baud} 8N1，{seconds:.1f} s ...")
    try:
        with serial.Serial(device, baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            deadline = time.time() + seconds
            total = 0
            while time.time() < deadline:
                chunk = ser.read(256)
                if not chunk:
                    continue
                total += len(chunk)
                print(f"{total - len(chunk):06d}  {preview(chunk, 64)}")
    except serial.SerialException as error:
        print(f"[probe] 打开失败：{error}")
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="UART2 P1.0/P1.1 收发探测")
    parser.add_argument("--port", help="串口，例如 COM3；缺省自动选择 USB 串口")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--bytes", type=int, default=1024, help="回显测试字节数")
    parser.add_argument("--chunk", type=int, default=64, help="每块字节数")
    parser.add_argument("--wait", type=float, default=2.5, help="等待上电横幅秒数")
    parser.add_argument("--autobaud", action="store_true", help="收不到数据时扫描波特率")
    parser.add_argument("--listen", type=float, metavar="SECONDS", help="只监听")
    parser.add_argument(
        "--loopback",
        action="store_true",
        help="USB-TTL 自环测试（先短接适配器 TXD/RXD）",
    )
    args = parser.parse_args()

    try:
        device = pick_port(args.port)
    except SystemExit as error:
        print(error)
        return 2

    if args.loopback:
        return loopback(device, args.baud)

    if args.listen is not None:
        return listen(device, args.baud, args.listen)

    if probe_once(device, args):
        return 0

    if args.autobaud:
        autobaud(device, args)
    else:
        print("[probe] 可加 --autobaud 扫描波特率，或用 --listen 5 查看原始数据。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
