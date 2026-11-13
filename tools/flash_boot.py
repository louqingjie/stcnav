#!/usr/bin/env python3
"""stcnav HID Bootloader 上位机。

用法：
  python3 tools/flash_boot.py build/usb_hid_isp/usb_hid_isp.hex
  python3 tools/flash_boot.py --enter          # 让正在运行的 App 进入 Bootloader
  python3 tools/flash_boot.py --wait           # 等待 Bootloader 的 1 秒启动窗口
  python3 tools/flash_boot.py --probe          # 探测 IAP 读映射
  python3 tools/flash_boot.py --isp            # 从 Bootloader 复位到 ROM ISP

协议：64 字节 HID 报告，主机侧报告 ID 占 1 字节，故实际数据 63 字节。
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

from check_layout import LayoutError, parse_hex
from flash import (
    HidTransport,
    NoResponse,
    ProtocolError,
    device_present,
    find_and_open,
)

BOOT_VID = 0x34BF
BOOT_PID = 0xFF02
APP_VID = 0x34BF
APP_PID = 0xFF01
ISP_PID = 0x1001

REPORT_SIZE = 64
HOST_REPORT_SIZE = 63

APP_START = 0xFE4000
APP_END = 0xFEFFFF
VEC_START = 0xFF0000
VEC_END = 0xFF0FFF

BOOTLOADER_ENTRY = 0xFE0000
TRAMPOLINE = 0xFF0003
TRAMPOLINE_BYTES = bytes([0x8A, 0xFE, 0x00, 0x00])

CMD_INFO = 0x01
CMD_ERASE = 0x02
CMD_BEGIN = 0x03
CMD_WRITE = 0x04
CMD_VERIFY = 0x05
CMD_FINALIZE = 0x06
CMD_RUN = 0x07
CMD_ISP = 0x08
CMD_READ = 0x09
CMD_PING = 0x0A
CMD_PROBE = 0x0B

STATUS_NAMES = {
    0: "OK",
    1: "FAIL",
    2: "BAD_COMMAND",
    3: "BAD_ADDRESS",
    4: "BAD_LENGTH",
    5: "IAP_ERROR",
    6: "CRC_ERROR",
    7: "NO_APP",
    8: "NOT_VERIFIED",
}


class BootError(Exception):
    pass


def build_report(command: int, payload: bytes = b"") -> bytes:
    data = b"STCNAV" + bytes([0x02, command]) + payload
    if len(data) > HOST_REPORT_SIZE:
        raise BootError(f"命令 {command:#04x} 载荷过长：{len(data)}")
    return data + bytes(HOST_REPORT_SIZE - len(data))


def send_command(
    transport: HidTransport,
    command: int,
    payload: bytes = b"",
    timeout_ms: int = 3000,
) -> bytes:
    report = build_report(command, payload)
    transport.write(bytes([0]) + report)
    response = transport.read(timeout_ms)
    if not response:
        raise NoResponse(f"命令 {command:#04x} 无响应")
    if len(response) < 9 or response[0:6] != b"STCNAV" or response[6] != 0x02:
        raise ProtocolError(
            f"命令 {command:#04x} 响应头异常：{response[:9].hex(' ')}"
        )
    if response[7] != command:
        raise ProtocolError(
            f"命令 {command:#04x} 响应命令不匹配：{response[7]:#04x}"
        )
    status = response[8]
    if status != 0:
        raise ProtocolError(
            f"命令 {command:#04x} 失败：{STATUS_NAMES.get(status, status)}"
        )
    return response[9:]


def prepare_image(hex_path: Path):
    image = parse_hex(hex_path)
    app_addresses = [address for address in image if APP_START <= address <= APP_END]
    vector_addresses = [address for address in image if VEC_START <= address <= VEC_END]
    if not app_addresses:
        raise LayoutError(f"HEX 中没有 0x{APP_START:06X} 以上的 App 数据")
    if not vector_addresses:
        raise LayoutError("HEX 中没有中断向量数据")

    # App 的复位跳转必须回到 Bootloader，而不是直接进 App。
    for offset, value in enumerate(TRAMPOLINE_BYTES):
        image[TRAMPOLINE + offset] = value
    if TRAMPOLINE not in vector_addresses:
        vector_addresses.append(TRAMPOLINE)

    app_length = max(app_addresses) - APP_START + 1
    vector_length = max(4, max(vector_addresses) - VEC_START + 1)

    app_data = bytearray([0xFF] * app_length)
    for address in app_addresses:
        app_data[address - APP_START] = image[address]

    vector_data = bytearray([0xFF] * vector_length)
    for address in vector_addresses:
        vector_data[address - VEC_START] = image[address]

    app_crc = zlib.crc32(app_data) & 0xFFFFFFFF
    vector_crc = zlib.crc32(vector_data) & 0xFFFFFFFF

    return app_length, vector_length, app_crc, vector_crc, app_data, vector_data


def print_info(payload: bytes) -> None:
    if len(payload) < 18:
        print(f"[boot] INFO 数据过短：{payload.hex(' ')}")
        return
    app_start, app_size, app_length, app_crc = struct.unpack("<IIII", payload[:16])
    print(
        f"[boot] app 0x{app_start:06X} 容量 {app_size} 字节，"
        f"已写入 {app_length} 字节，CRC32 0x{app_crc:08X}，"
        f"有效={payload[16]}"
    )


def wait_for_device(vid: int, pid: int, timeout_s: float, label: str) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if device_present(vid, pid):
            return
        time.sleep(0.05)
    raise BootError(f"等待 {label}（{vid:04X}:{pid:04X}）超时")


def enter_bootloader(timeout_s: float) -> None:
    device = find_and_open(APP_VID, APP_PID)
    if device is None:
        raise BootError(
            f"未找到 App HID（{APP_VID:04X}:{APP_PID:04X}）；"
            "请给板子上电，或先烧录 usb_hid_isp 应用"
        )

    payload = b"STCNAV" + bytes([0x02]) + bytes(HOST_REPORT_SIZE - 7)
    try:
        written = False
        for report in (bytes([0]) + payload, payload):
            try:
                if device.write(report) >= 0:
                    written = True
                    break
            except Exception:  # noqa: BLE001
                continue
        if not written:
            raise BootError("发送进入 Bootloader 命令失败")
    finally:
        try:
            device.close()
        except Exception:  # noqa: BLE001
            pass

    print("[boot] 已通知 App 进入 Bootloader，等待 FF02 枚举...")
    wait_for_device(BOOT_VID, BOOT_PID, timeout_s, "Bootloader")


def wait_bootloader(timeout_s: float) -> None:
    """给板子断电重上电后，抓住 Bootloader 的 1 秒启动窗口。"""
    print("[boot] 等待 Bootloader 枚举（请给板子上电）...")
    wait_for_device(BOOT_VID, BOOT_PID, timeout_s, "Bootloader")


def write_region(
    transport: HidTransport,
    base: int,
    data: bytes,
    label: str,
) -> None:
    offset = 0
    total = 0
    while offset < len(data):
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        start = offset
        while (
            offset < len(data)
            and data[offset] != 0xFF
            and offset - start < 51
        ):
            offset += 1
        chunk = data[start:offset]
        address = base + start
        payload = bytes(
            [
                address & 0xFF,
                (address >> 8) & 0xFF,
                (address >> 16) & 0xFF,
                len(chunk),
            ]
        ) + chunk
        send_command(transport, CMD_WRITE, payload)
        total += len(chunk)
        if (total // 1024) != ((total - len(chunk)) // 1024):
            print(f"[boot] {label} 已写入 {total} 字节")

    print(f"[boot] {label} 完成，共 {total} 字节")


def update(transport: HidTransport, hex_path: Path) -> None:
    app_length, vector_length, app_crc, vector_crc, app_data, vector_data = (
        prepare_image(hex_path)
    )
    print(
        f"[boot] 镜像 {hex_path.name}: App {app_length} 字节 "
        f"CRC32 0x{app_crc:08X}，向量 {vector_length} 字节 "
        f"CRC32 0x{vector_crc:08X}"
    )

    info = send_command(transport, CMD_INFO)
    print_info(info)

    print("[boot] 擦除 App + 向量区（约数秒）...")
    send_command(transport, CMD_ERASE, timeout_ms=60000)

    begin = struct.pack("<IHII", app_length, vector_length, app_crc, vector_crc)
    send_command(transport, CMD_BEGIN, begin)
    write_region(transport, APP_START, app_data, "App")
    write_region(transport, VEC_START, vector_data, "向量")

    print("[boot] 校验 CRC32...")
    send_command(transport, CMD_VERIFY, timeout_ms=60000)
    send_command(transport, CMD_FINALIZE, timeout_ms=10000)

    print("[boot] 启动 App...")
    send_command(transport, CMD_RUN)
    time.sleep(0.2)
    wait_for_device(APP_VID, APP_PID, 10, "App")
    print("[boot] App 已枚举为 34BF:FF01")


def main() -> int:
    parser = argparse.ArgumentParser(description="stcnav HID Bootloader 上位机")
    parser.add_argument("hex", nargs="?", type=Path, help="待更新的 App HEX")
    parser.add_argument("--enter", action="store_true", help="让运行中的 App 进入 Bootloader")
    parser.add_argument("--wait", action="store_true", help="等待 Bootloader 启动窗口")
    parser.add_argument("--probe", action="store_true", help="探测 IAP 读映射")
    parser.add_argument("--isp", action="store_true", help="复位到 ROM USB ISP")
    parser.add_argument("--ping", action="store_true", help="只发送 PING")
    parser.add_argument("--timeout", type=float, default=15, help="等待设备秒数")
    args = parser.parse_args()

    try:
        if args.wait:
            wait_bootloader(args.timeout)
        if args.enter:
            enter_bootloader(args.timeout)

        transport = None
        if args.hex or args.probe or args.isp or args.ping:
            transport = HidTransport(BOOT_VID, BOOT_PID)

        if transport is not None:
            try:
                if args.ping:
                    send_command(transport, CMD_PING)
                    print("[boot] PING OK")

                if args.probe:
                    data = send_command(transport, CMD_PROBE)
                    print(
                        f"[boot] IAP 读 0xFE0000={data[0]:02X} "
                        f"0xFF0000={data[1]:02X} 失败={data[2]}"
                    )

                if args.isp:
                    send_command(transport, CMD_ISP)
                    print("[boot] 已复位到 ROM ISP，等待 34BF:1001...")
                    wait_for_device(BOOT_VID, ISP_PID, args.timeout, "ROM ISP")

                if args.hex:
                    update(transport, args.hex)
            finally:
                transport.close()

        if not any([args.hex, args.enter, args.wait, args.probe, args.isp, args.ping]):
            parser.error("没有指定操作")
    except (BootError, LayoutError, ProtocolError, NoResponse, OSError) as error:
        print(f"FAIL: {error}")
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
