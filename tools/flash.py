#!/usr/bin/env python3
"""STC32G USB-ISP 烧录脚本（Linux hidraw / hidapi）。

协议流程参考 PIE-Block 的公开实现行为整理：
  - HID 报告 64 字节，报告 ID 0
  - 帧：46 B9 6A | len | payload | checksum | 16
  - 0xFE0000 区首块命令 0x32，后续 0x12
  - 0xFF0000 区命令 0x02
  - 每 128 字节一块，块间按地址写入
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import sys
import time
from pathlib import Path

from check_layout import (
    APP_END,
    APP_START,
    HIGH_GUARD,
    VECTOR_START,
    LayoutError,
    check_image,
    parse_hex,
)

VID_DEFAULT = 0x34BF
PID_DEFAULT = 0x1001
APP_VID_DEFAULT = 0x34BF
APP_PID_DEFAULT = 0xFF01
REPORT_SIZE = 64
HOT_COMMAND = b"STCNAV" + bytes([0x01])
BLOCK_SIZE = 128
REPORT_GAP = 0.001

CMD_INFO = 0x01
CMD_UNLOCK = 0x05
CMD_ERASE = 0x03
CMD_WRITE_FIRST = 0x32
CMD_WRITE_NEXT = 0x12
CMD_WRITE_HIGH = 0x02
CMD_EXIT = 0xFF
ACK_WRITE = 0x02
ACK_OK = 0x54

TIMEOUT_MS = 3000
ERASE_TIMEOUT_MS = 10000
WRITE_ATTEMPTS = 3
RETRY_BACKOFF = 0.05


class NoResponse(Exception):
    pass


class ProtocolError(Exception):
    pass


def build_packet(payload: list[int]) -> bytes:
    length = len(payload) + 6
    body = [0x6A, (length >> 8) & 0xFF, length & 0xFF] + payload
    checksum = sum(body) & 0xFFFF
    return bytes([0x46, 0xB9] + body + [(checksum >> 8) & 0xFF, checksum & 0xFF, 0x16])


def split_reports(packet: bytes) -> list[bytes]:
    # hidapi 的第一个字节是 report ID；无编号报告用 0，故每帧最多 63 字节数据。
    payload_size = REPORT_SIZE - 1
    reports = []
    for offset in range(0, len(packet), payload_size):
        chunk = packet[offset:offset + payload_size]
        report = bytes([0x00]) + chunk
        report += bytes(REPORT_SIZE - len(report))
        reports.append(report)
    return reports


def build_blocks(image: dict[int, int]) -> list[tuple[str, int, int, bytes]]:
    """返回 (区域, 相对地址, 命令, 128 字节数据)。"""
    chunks: dict[tuple[str, int], bytearray] = {}
    for address, value in image.items():
        if address < APP_START or address >= HIGH_GUARD:
            raise LayoutError(f"地址 0x{address:06X} 不在可烧录范围")
        if address < VECTOR_START:
            region = "app"
            relative = address - APP_START
        else:
            region = "vector"
            relative = address - VECTOR_START
        if relative >= 0x10000:
            raise LayoutError(f"地址 0x{address:06X} 超出 64 KiB 块")
        key = (region, relative // BLOCK_SIZE)
        chunks.setdefault(key, bytearray([0xFF] * BLOCK_SIZE))[relative % BLOCK_SIZE] = value

    blocks = []
    for key in sorted(chunks, key=lambda item: (item[0] != "app", item[1])):
        region, index = key
        if region == "app":
            command = CMD_WRITE_FIRST if not blocks else CMD_WRITE_NEXT
        else:
            command = CMD_WRITE_HIGH
        blocks.append((region, index * BLOCK_SIZE, command, bytes(chunks[key])))
    return blocks


def verify_blocks(image: dict[int, int], blocks: list[tuple[str, int, int, bytes]]) -> None:
    seen = {}
    for region, address, _, data in blocks:
        base = APP_START if region == "app" else VECTOR_START
        for offset, value in enumerate(data):
            target = base + address + offset
            if target in image:
                seen[target] = value
    if seen != image:
        missing = set(image) - set(seen)
        changed = [key for key in image if key in seen and image[key] != seen[key]]
        raise LayoutError(
            f"分块校验失败：缺 {len(missing)} 字节，数值不符 {len(changed)} 字节"
        )


class _HidDeviceInfo(ctypes.Structure):
    _fields_ = [
        ("path", ctypes.c_char_p),
        ("vendor_id", ctypes.c_ushort),
        ("product_id", ctypes.c_ushort),
        ("serial_number", ctypes.c_wchar_p),
    ]


class HidrawDevice:
    def __init__(self, library, handle) -> None:
        self._library = library
        self._handle = handle

    def write(self, data: bytes) -> int:
        buffer = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        return int(self._library.hid_write(self._handle, buffer, len(data)))

    def read(self, size: int, timeout_ms: int) -> bytes:
        buffer = (ctypes.c_ubyte * size)()
        count = int(
            self._library.hid_read_timeout(self._handle, buffer, size, timeout_ms)
        )
        if count <= 0:
            return b""
        return bytes(buffer[:count])

    def close(self) -> None:
        if self._handle:
            self._library.hid_close(self._handle)
            self._handle = None


class HidrawApi:
    """直接调用 libhidapi-hidraw，绕过 libusb 后端对 /dev/bus/usb 的权限要求。"""

    def __init__(self) -> None:
        name = ctypes.util.find_library("hidapi-hidraw") or "libhidapi-hidraw.so.0"
        self._library = ctypes.CDLL(name)
        self._library.hid_init.restype = ctypes.c_int
        self._library.hid_enumerate.argtypes = [ctypes.c_ushort, ctypes.c_ushort]
        self._library.hid_enumerate.restype = ctypes.POINTER(_HidDeviceInfo)
        self._library.hid_free_enumeration.argtypes = [
            ctypes.POINTER(_HidDeviceInfo)
        ]
        self._library.hid_open_path.argtypes = [ctypes.c_char_p]
        self._library.hid_open_path.restype = ctypes.c_void_p
        self._library.hid_write.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_size_t,
        ]
        self._library.hid_write.restype = ctypes.c_int
        self._library.hid_read_timeout.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_size_t,
            ctypes.c_int,
        ]
        self._library.hid_read_timeout.restype = ctypes.c_int
        self._library.hid_close.argtypes = [ctypes.c_void_p]
        self._library.hid_init()

    def find(self, vid: int, pid: int) -> bytes | None:
        info = self._library.hid_enumerate(vid, pid)
        if not info:
            return None
        path = info.contents.path
        self._library.hid_free_enumeration(info)
        return path

    def open(self, path: bytes) -> HidrawDevice:
        handle = self._library.hid_open_path(path)
        if not handle:
            raise OSError(f"无法打开 {path!r}")
        return HidrawDevice(self._library, handle)


_HIDRAW_API: HidrawApi | None = None


def hidraw_api() -> HidrawApi:
    global _HIDRAW_API
    if _HIDRAW_API is None:
        _HIDRAW_API = HidrawApi()
    return _HIDRAW_API


def find_and_open(vid: int, pid: int):
    """优先 Python hid 模块；权限不足时回退到 hidraw。"""
    hid = import_hid()
    devices = hid.enumerate(vid, pid)
    if devices:
        try:
            return open_hid(devices[0])
        except Exception:  # noqa: BLE001
            pass

    try:
        api = hidraw_api()
        path = api.find(vid, pid)
        if path:
            return api.open(path)
    except Exception:  # noqa: BLE001
        # Windows 上没有 libhidapi-hidraw；找不到设备时保持返回 None。
        pass
    return None


def import_hid():
    try:
        import hid  # type: ignore
    except ImportError as error:
        raise SystemExit(
            "缺少 hidapi：pip install --user hidapi（或安装 libhidapi-dev）"
        ) from error
    return hid


def open_hid(info) -> object:
    hid = import_hid()
    path = info["path"]
    if hasattr(hid, "Device"):
        return hid.Device(path=path)
    device = hid.device()
    device.open_path(path)
    return device


def device_present(vid: int, pid: int) -> bool:
    try:
        hid = import_hid()
        if hid.enumerate(vid, pid):
            return True
    except Exception:  # noqa: BLE001
        pass
    try:
        return hidraw_api().find(vid, pid) is not None
    except Exception:  # noqa: BLE001
        return False


def send_hot_reset(
    app_vid: int,
    app_pid: int,
    isp_vid: int,
    isp_pid: int,
    timeout_s: int,
) -> None:
    """向应用 HID 发送复位命令，并等待 ROM USB-ISP 重新枚举。"""
    device = find_and_open(app_vid, app_pid)
    if device is None:
        raise ProtocolError(
            f"未找到应用 HID（VID:PID {app_vid:04X}:{app_pid:04X}）；"
            "请确认 usb_hid_isp 正在运行且原生 USB 已连接"
        )

    try:
        payload = HOT_COMMAND + bytes(REPORT_SIZE - len(HOT_COMMAND))
        written = False
        for report in (bytes([0]) + payload, payload):
            try:
                if device.write(report) >= 0:
                    written = True
                    break
            except Exception:  # noqa: BLE001
                continue
        if not written:
            raise ProtocolError("发送复位命令失败")
    finally:
        try:
            device.close()
        except Exception:  # noqa: BLE001
            pass

    print("[flash] 已发送复位命令，等待 ROM USB-ISP 枚举...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if device_present(isp_vid, isp_pid):
            return
        time.sleep(0.1)
    raise ProtocolError(
        f"等待 ROM USB-ISP（{isp_vid:04X}:{isp_pid:04X}）超时；"
        "检查 P3.0/P3.1 是否接到电脑 USB"
    )


class HidTransport:
    def __init__(self, vid: int, pid: int) -> None:
        device = find_and_open(vid, pid)
        if device is None:
            raise SystemExit(
                f"未找到 USB-ISP 设备（VID:PID {vid:04X}:{pid:04X}）；"
                "请断电重新上电进入 ISP 模式"
            )
        self.device = device

    def write(self, report: bytes) -> None:
        if self.device.write(report) < 0:
            raise ProtocolError("HID 写入失败")

    def read(self, timeout_ms: int) -> bytes:
        return bytes(self.device.read(REPORT_SIZE, timeout_ms))

    def close(self) -> None:
        try:
            self.device.close()
        except Exception:  # noqa: BLE001
            pass


def expect_ack(
    transport: HidTransport,
    payload: list[int],
    label: str,
    command: int,
    timeout_ms: int = TIMEOUT_MS,
) -> bytes:
    packet = build_packet(payload)
    reports = split_reports(packet)
    for index, report in enumerate(reports):
        if index:
            time.sleep(REPORT_GAP)
        transport.write(report)

    response = transport.read(timeout_ms)
    if not response:
        raise NoResponse(f"{label}：设备无响应")
    if len(response) < 7 or response[0] != 0x46 or response[1] != 0xB9 or response[2] != 0x68:
        raise ProtocolError(f"{label}：响应帧头异常")
    size = (response[3] << 8) | response[4]
    if size < 7 or size + 2 > len(response):
        raise ProtocolError(f"{label}：响应长度异常")
    data = response[5:size - 1]
    checksum = (response[size - 1] << 8) | response[size]
    parity = 0x68 + (size >> 8) + (size & 0xFF) + sum(data)
    if (parity & 0xFFFF) != checksum or response[size + 1] != 0x16:
        raise ProtocolError(f"{label}：响应校验失败")
    if response[5] != command:
        raise ProtocolError(f"{label}：响应命令异常（{response[5]:#04x}）")
    return response


def flash(image: dict[int, int], transport: HidTransport) -> None:
    blocks = build_blocks(image)
    print(f"[flash] 共 {len(blocks)} 块（128 字节/块）")

    expect_ack(
        transport,
        [CMD_INFO, 0, 0, 0, 0, 0, 0, 0x80, 0],
        "info",
        CMD_INFO,
    )
    expect_ack(transport, [CMD_UNLOCK, 0, 0, 0x5A, 0xA5], "unlock", CMD_UNLOCK)
    expect_ack(
        transport,
        [CMD_ERASE, 0, 0, 0x5A, 0xA5],
        "erase",
        CMD_ERASE,
        timeout_ms=ERASE_TIMEOUT_MS,
    )

    for index, (region, address, command, data) in enumerate(blocks):
        label = f"写入 {region} 0x{address:04X}"
        payload = [
            command,
            (address >> 8) & 0xFF,
            address & 0xFF,
            0x5A,
            0xA5,
            *data,
        ]
        response = None
        for attempt in range(1, WRITE_ATTEMPTS + 1):
            try:
                response = expect_ack(transport, payload, label, ACK_WRITE)
                break
            except NoResponse:
                if attempt >= WRITE_ATTEMPTS:
                    raise
                print(f"[flash] {label} 无响应，重试 {attempt + 1}/{WRITE_ATTEMPTS}")
                time.sleep(RETRY_BACKOFF)
        if response is None or len(response) < 7 or response[6] != ACK_OK:
            raise ProtocolError(f"{label} 未确认成功")
        print(f"[flash] {index + 1}/{len(blocks)} {label}")

    for report in split_reports(build_packet([CMD_EXIT])):
        transport.write(report)
    time.sleep(0.2)
    print("[flash] 已复位主控板")


def main() -> int:
    parser = argparse.ArgumentParser(description="烧录 stcnav HEX")
    parser.add_argument("hex", type=Path)
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=VID_DEFAULT)
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=PID_DEFAULT)
    parser.add_argument("--dry-run", action="store_true", help="只校验并打印分块计划")
    parser.add_argument("--verify", action="store_true", help="校验分块打包未丢数据")
    parser.add_argument("--hot", action="store_true", help="先通过应用 HID 软复位到 ISP")
    parser.add_argument(
        "--app-vid", type=lambda value: int(value, 0), default=APP_VID_DEFAULT
    )
    parser.add_argument(
        "--app-pid", type=lambda value: int(value, 0), default=APP_PID_DEFAULT
    )
    parser.add_argument(
        "--reset-timeout", type=int, default=10, help="等待 ROM ISP 枚举的秒数"
    )
    args = parser.parse_args()

    try:
        image = parse_hex(args.hex)
        check_image(image, APP_START, APP_END)
        blocks = build_blocks(image)
        if args.verify:
            verify_blocks(image, blocks)
    except (LayoutError, OSError) as error:
        print(f"FAIL: {error}")
        return 1

    print(
        f"[flash] {args.hex}: {len(image)} 字节，"
        f"{len(blocks)} 块，0x{min(image):06X}-0x{max(image):06X}"
    )
    if args.dry_run:
        for region, address, command, _ in blocks:
            print(f"[flash]   {region} 0x{address:04X} cmd={command:#04x}")
        return 0

    if args.hot:
        try:
            send_hot_reset(
                args.app_vid,
                args.app_pid,
                args.vid,
                args.pid,
                args.reset_timeout,
            )
        except ProtocolError as error:
            print(f"FAIL: {error}")
            return 1

    transport = HidTransport(args.vid, args.pid)
    try:
        flash(image, transport)
    except (NoResponse, ProtocolError) as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        transport.close()

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
