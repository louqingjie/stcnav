#!/usr/bin/env python3
"""串口原始日志采集：把收到的字节带时间戳写入文件，供后续离线脚本分析。

用法（后台运行，不阻塞）：
    python tools/uart2_log.py --port COM23 --plan "115200:15,230400:600" \
        --out build/uart2_live.log

    # 复现/验证 UART2 静默：主机侧同时按 10~50 Hz 发心跳，制造 MCU 接收中断
    python tools/uart2_log.py --port COM23 --plan "230400:1800" \
        --host-hb-hz 50 --out build/uart2_after_fix.log

日志格式（每行一个接收块）：
    <epoch 秒> <hex 字节，空格分隔>

--plan 用逗号分隔 `波特率:秒数` 段，用来跨"启动期 115200 / 运行期 230400"
切换（同一时刻只能有一个进程占用串口，所以由本进程依次切换）。

--host-hb-hz > 0 时在 230400 段后台发 Host heartbeat（0x03）帧。原因：UART2
静默的根因是"接收中断与发送完成几个时钟内重合"的竞态，只收不发时几乎不触发；
实测主机持续发心跳时旧固件 10~120 s 内必静默，用来做修复前后的 A/B 对比。
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")

MAGIC = b"\x52\x4d"
VERSION = 0x02
TYPE_HOST_HEARTBEAT = 0x03
HOST_HEARTBEAT_PAYLOAD = 13


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_host_heartbeat(sequence: int, uptime_ms: int, boot_id: int = 0x5A5A) -> bytes:
    payload = struct.pack("<IIIB", uptime_ms & 0xFFFFFFFF, boot_id, 0, 1)
    header = bytes([VERSION, TYPE_HOST_HEARTBEAT]) + struct.pack(
        "<HH", HOST_HEARTBEAT_PAYLOAD, sequence & 0xFFFF)
    body = header + payload
    return MAGIC + body + struct.pack("<H", crc16_modbus(body))


class HeartbeatInjector(threading.Thread):
    """后台线程：按固定频率向 MCU 发合法 Host heartbeat，制造接收中断。"""

    def __init__(self, ser: "serial.Serial", hz: float):
        super().__init__(daemon=True)
        self._ser = ser
        self._period = 1.0 / hz
        self._stop = threading.Event()
        self.sent = 0

    def run(self) -> None:
        seq = 0
        t0 = time.time()
        next_at = t0
        while not self._stop.is_set():
            frame = build_host_heartbeat(seq, int((time.time() - t0) * 1000))
            try:
                self._ser.write(frame)
                self.sent += 1
                seq += 1
            except serial.SerialException:
                return
            next_at += self._period
            delay = next_at - time.time()
            if delay > 0:
                self._stop.wait(delay)

    def stop(self) -> None:
        self._stop.set()
        self.join(timeout=1.0)


def parse_plan(text: str) -> list[tuple[int, float]]:
    plan: list[tuple[int, float]] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        baud_text, seconds_text = part.split(":")
        plan.append((int(baud_text), float(seconds_text)))
    if not plan:
        raise argparse.ArgumentTypeError("plan 不能为空")
    return plan


def main() -> int:
    parser = argparse.ArgumentParser(description="串口原始日志采集")
    parser.add_argument("--port", required=True)
    parser.add_argument("--plan", type=parse_plan, default=[(230400, 300.0)],
                        help='如 "115200:15,230400:600"')
    parser.add_argument("--out", required=True)
    parser.add_argument("--chunk", type=int, default=512)
    parser.add_argument("--host-hb-hz", type=float, default=0.0,
                        help="主机侧 0x03 心跳发送频率（0=不发；建议 10~50）")
    args = parser.parse_args()

    total_bytes = 0
    start = time.time()
    with open(args.out, "a", encoding="utf-8", buffering=1) as log:
        log.write(f"# port={args.port} plan={args.plan} start={start:.3f} "
                  f"host_hb_hz={args.host_hb_hz}\n")
        for baud, seconds in args.plan:
            try:
                ser = serial.Serial(args.port, baud, timeout=0.05)
            except serial.SerialException as error:
                log.write(f"# open {baud} failed: {error}\n")
                print(f"[log] open {baud} failed: {error}")
                continue
            print(f"[log] {baud} baud for {seconds:.0f}s ...")
            log.write(f"# segment start baud={baud}\n")
            injector = None
            if args.host_hb_hz > 0 and baud >= 230400:
                injector = HeartbeatInjector(ser, args.host_hb_hz)
                injector.start()
            deadline = time.time() + seconds
            try:
                while time.time() < deadline:
                    chunk = ser.read(args.chunk)
                    if chunk:
                        now = time.time()
                        total_bytes += len(chunk)
                        log.write(f"{now:.3f} {chunk.hex(' ')}\n")
            finally:
                if injector is not None:
                    injector.stop()
                    print(f"[log] host heartbeat injected: {injector.sent} frames")
                    log.write(f"# host heartbeat injected={injector.sent}\n")
                ser.close()
                log.write(f"# segment end baud={baud} t={time.time() - start:.1f}s\n")
    elapsed = time.time() - start
    print(f"[log] done: {total_bytes} bytes in {elapsed:.1f}s -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
