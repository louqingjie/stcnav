#!/usr/bin/env python3
"""STC32G 串口 ISP 烧录（ROM 系统区，8 数据位 + 偶校验）。

协议参考《STC32G 系列技术手册》附录 C：
  握手：2400 波特率连发 0x7F，直到收到状态包（首字节 0x50）
  0x01  设置参数并切到最高波特率
  0x05  准备编程
  0x03  擦除 Flash
  0x32/0x12  写 0xFE0000 区（首块/后续，128 字节一块）
  0x22/0x02  写 0xFF0000 区（首块/后续）

逻辑地址映射（命令中的 2 字节地址为逻辑地址低 16 位）：
  0x00000-0x0FFFF -> 物理 0xFE0000-0xFEFFFF（用户程序/自检固件）
  0x10000-0x1FFFF -> 物理 0xFF0000-0xFFFFFF（中断向量区）

用法：
  python tools/uart_flash.py build/iap_selftest/iap_selftest.hex -p COM5
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")

from check_layout import APP_END, APP_START, LayoutError, check_image, parse_hex

PACKET_START = bytes([0x46, 0xB9])
PACKET_HOST = 0x6A
PACKET_MCU = 0x68
PACKET_END = 0x16

CMD_SET_BAUD = 0x01
CMD_PREPARE = 0x05
CMD_ERASE = 0x03
CMD_WRITE_FIRST = 0x32
CMD_WRITE_NEXT = 0x12
CMD_WRITE_HIGH_FIRST = 0x22
CMD_WRITE_HIGH_NEXT = 0x02
ACK_WRITE = 0x02
ACK_OK = 0x54

HANDSHAKE_BYTE = 0x7F
STATUS_MAGIC = 0x50

BLOCK_SIZE = 128
LOW_REGION = 0x00000
LOW_REGION_END = 0x0FFFF
HIGH_REGION = 0x10000
HIGH_REGION_END = 0x1FFFF

DEFAULT_HANDSHAKE_BAUD = 2400
DEFAULT_BAUD = 115200
ISP_CLOCK = 24000000
IAP_WAIT_35M = 0x97


class FlashError(Exception):
    pass


def build_frame(payload: bytes) -> bytes:
    """主机帧：46 B9 6A 00 | len | payload | sum(2) | 16。"""
    length = len(payload) + 6
    if length > 0xFF:
        raise FlashError(f"数据包过长：{len(payload)} 字节")
    checksum = (PACKET_HOST + length + sum(payload)) & 0xFFFF
    return (
        PACKET_START
        + bytes([PACKET_HOST, 0x00, length])
        + payload
        + bytes([(checksum >> 8) & 0xFF, checksum & 0xFF, PACKET_END])
    )


def read_frame(ser: "serial.Serial", timeout: float) -> bytes:
    """读取 MCU 帧并校验，返回 payload。"""
    deadline = time.time() + timeout
    state = 0
    length = 0
    checksum = 0
    payload = bytearray()
    seen = bytearray()

    while time.time() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        value = byte[0]
        if len(seen) < 24:
            seen.append(value)

        if state == 0:
            state = 1 if value == 0x46 else 0
        elif state == 1:
            state = 2 if value == 0xB9 else (1 if value == 0x46 else 0)
        elif state == 2:
            state = 3 if value == PACKET_MCU else (1 if value == 0x46 else 0)
        elif state == 3:
            state = 4 if value == 0x00 else (1 if value == 0x46 else 0)
        elif state == 4:
            if value < 6:
                state = 0
                continue
            length = value
            checksum = (PACKET_MCU + length) & 0xFFFF
            payload = bytearray()
            state = 5
        elif state == 5:
            payload.append(value)
            checksum = (checksum + value) & 0xFFFF
            if len(payload) >= length - 6:
                state = 6
        elif state == 6:
            if value != (checksum >> 8) & 0xFF:
                state = 0
                continue
            state = 7
        elif state == 7:
            if value != (checksum & 0xFF):
                state = 0
                continue
            state = 8
        elif state == 8:
            if value == PACKET_END:
                return bytes(payload)
            state = 0

    raise TimeoutError(
        "等待 MCU 数据包超时"
        f"（收到 {seen.hex(' ') if seen else '无数据'}）"
    )


def send_payload(ser: "serial.Serial", payload: list[int] | bytes) -> None:
    ser.write(build_frame(bytes(payload)))
    ser.flush()


def expect_reply(
    ser: "serial.Serial", payload: list[int], command: int, timeout: float
) -> bytes:
    send_payload(ser, payload)
    reply = read_frame(ser, timeout)
    if not reply or reply[0] != command:
        got = reply[0] if reply else None
        raise FlashError(f"命令 {command:#04x} 响应异常：{got!r}")
    return reply


def sync(
    ser: "serial.Serial", timeout: float, on_wait=None
) -> bytes:
    """握手：持续发送 0x7F，直到收到状态包。"""
    deadline = time.time() + timeout
    last_notice = time.time()
    last_detail = ""
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                reply = read_frame(ser, 1.0)
                if reply and reply[0] == STATUS_MAGIC:
                    return reply
            except TimeoutError as error:
                last_detail = str(error)
            continue
        ser.write(bytes([HANDSHAKE_BYTE]))
        ser.flush()
        if on_wait and time.time() - last_notice > 2.0:
            on_wait()
            last_notice = time.time()
        time.sleep(0.03)
    detail = f"；{last_detail}" if last_detail else "；串口没有收到任何数据"
    raise FlashError(
        "握手超时：请给目标板断电再上电，并确认 P3.0/P3.1 接线" + detail
    )


def baud_reload(baud: int) -> int:
    """手册公式：65536 - FUSER / 4 / baud，STC32G 的 FUSER 为 24 MHz。"""
    reload = 65536 - ISP_CLOCK // 4 // baud
    if not 0 <= reload <= 0xFFFF:
        raise FlashError(f"无法计算 {baud} 波特率重装值")
    return reload


def set_baud(ser: "serial.Serial", baud: int) -> None:
    reload = baud_reload(baud)
    payload = [
        CMD_SET_BAUD,
        0x00,
        0x00,
        (reload >> 8) & 0xFF,
        reload & 0xFF,
        0x00,
        0x00,
        IAP_WAIT_35M,
    ]
    expect_reply(ser, payload, CMD_SET_BAUD, 2.0)
    time.sleep(0.01)
    ser.baudrate = baud


def parse_status_clock(status: bytes) -> int:
    if len(status) < 5:
        return 0
    return int.from_bytes(status[1:5], "big")


def logical_address(physical: int) -> int:
    if APP_START <= physical < 0xFF0000:
        return physical - APP_START
    if 0xFF0000 <= physical <= 0xFFFFFF:
        return HIGH_REGION + (physical - 0xFF0000)
    raise FlashError(f"地址 0x{physical:06X} 不在可烧录范围")


def build_blocks(image: dict[int, int]) -> list[tuple[int, bytes]]:
    """按 128 字节分块，返回 (逻辑地址, 数据)，先 0xFE0000 区后 0xFF0000 区。"""
    cells: dict[int, int] = {}
    for physical, value in image.items():
        cells[logical_address(physical)] = value

    blocks = []
    for base in range(0, HIGH_REGION_END + 1, BLOCK_SIZE):
        chunk = bytearray()
        used = False
        for offset in range(BLOCK_SIZE):
            value = cells.get(base + offset)
            if value is None:
                chunk.append(0xFF)
            else:
                chunk.append(value)
                used = True
        if used:
            blocks.append((base, bytes(chunk)))
    return blocks


def erase(ser: "serial.Serial") -> None:
    expect_reply(
        ser, [CMD_ERASE, 0x00, 0x00, 0x5A, 0xA5], CMD_ERASE, 20.0
    )


def program(ser: "serial.Serial", blocks: list[tuple[int, bytes]]) -> None:
    first = True
    total = len(blocks)
    for index, (address, data) in enumerate(blocks):
        if address <= LOW_REGION_END:
            command = CMD_WRITE_FIRST if first else CMD_WRITE_NEXT
        else:
            command = CMD_WRITE_HIGH_FIRST if first else CMD_WRITE_HIGH_NEXT

        payload = [
            command,
            (address >> 8) & 0xFF,
            address & 0xFF,
            0x5A,
            0xA5,
            *data,
        ]
        reply = expect_reply(ser, payload, ACK_WRITE, 5.0)
        if len(reply) < 2 or reply[1] != ACK_OK:
            raise FlashError(f"写入 0x{address:05X} 未确认成功")
        first = False
        print(f"[uart] {index + 1}/{total} 写入 0x{address:05X}")


def pick_port(port: str | None) -> str:
    if port:
        return port
    ports = list(list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise FlashError("没有检测到串口，请用 -p 指定（如 COM5）")
    names = "、".join(f"{item.device}({item.description})" for item in ports)
    raise FlashError(f"检测到多个串口：{names}；请用 -p 指定")


def main() -> int:
    parser = argparse.ArgumentParser(description="STC32G 串口 ISP 烧录")
    parser.add_argument("hex", type=Path, help="HEX 文件路径")
    parser.add_argument("-p", "--port", help="串口设备，如 COM5 或 /dev/ttyUSB0")
    parser.add_argument(
        "-b", "--baud", type=int, default=DEFAULT_BAUD, help="下载波特率"
    )
    parser.add_argument(
        "--handshake-baud", type=int, default=DEFAULT_HANDSHAKE_BAUD
    )
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument(
        "--no-baud-switch",
        action="store_true",
        help="不发送 0x01，保持在握手波特率下载",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印分块计划")
    args = parser.parse_args()

    try:
        image = parse_hex(args.hex)
        check_image(image, APP_START, APP_END)
        blocks = build_blocks(image)
    except (LayoutError, OSError, FlashError) as error:
        print(f"FAIL: {error}")
        return 1

    print(
        f"[uart] {args.hex}: {len(image)} 字节，{len(blocks)} 块，"
        f"物理 0x{min(image):06X}-0x{max(image):06X}"
    )
    if args.dry_run:
        for address, _ in blocks:
            print(f"[uart]   逻辑 0x{address:05X}")
        return 0

    try:
        port = pick_port(args.port)
    except FlashError as error:
        print(f"FAIL: {error}")
        return 1

    print(f"[uart] 串口 {port}，{args.handshake_baud} 8E1 握手")
    print("[uart] 请给目标板断电，脚本开始后重新上电")

    try:
        with serial.Serial(
            port=port,
            baudrate=args.handshake_baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
        ) as ser:
            ser.reset_input_buffer()

            def waiting() -> None:
                print("[uart] 等待握手（请确认已重新上电）...")

            status = sync(ser, args.connect_timeout, waiting)
            fosc = parse_status_clock(status)
            if fosc:
                print(f"[uart] 握手成功，目标频率约 {fosc / 1e6:.3f} MHz")
            else:
                print("[uart] 握手成功")

            if args.no_baud_switch:
                print(f"[uart] 保持 {args.handshake_baud} 波特率下载")
            else:
                print(f"[uart] 切换到 {args.baud} 波特率")
                set_baud(ser, args.baud)

            expect_reply(
                ser, [CMD_PREPARE, 0x00, 0x00, 0x5A, 0xA5], CMD_PREPARE, 2.0
            )
            print("[uart] 擦除 Flash ...")
            erase(ser)
            print("[uart] 擦除完成，开始写入")
            program(ser, blocks)
    except (FlashError, TimeoutError, serial.SerialException, OSError) as error:
        print(f"FAIL: {error}")
        return 1

    print("[uart] 写入完成；若程序未自动运行，请断电重启")
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
