#!/usr/bin/env python3
"""IAP + USB HID 自检上位机（34BF:FF03）。

固件：apps/iap_selftest
命令：
  --ping        确认 USB EP1 OUT/IN
  --read-code   比较 __code 与 IAP 对 0xFE4000/0xFF0000 的读取
  --test-sector 擦除/写入/回读 0xFEF000 的 512 字节
  --all         依次执行以上三项
  --reset-isp   让自检固件复位到 ROM ISP
"""

from __future__ import annotations

import argparse
import sys
import time

from flash import (
    HidTransport,
    NoResponse,
    ProtocolError,
    device_present,
)

VID = 0x34BF
PID = 0xFF03
ISP_PID = 0x1001

REPORT_SIZE = 64
HOST_REPORT_SIZE = 63
PROTO_VERSION = 0x03

CMD_PING = 0x01
CMD_READ_CODE = 0x02
CMD_TEST_SECTOR = 0x03
CMD_RESET_ISP = 0x04


def send_command(transport: HidTransport, command: int, timeout_ms: int = 3000) -> bytes:
    report = (
        b"STCNAV"
        + bytes([PROTO_VERSION, command])
        + bytes(HOST_REPORT_SIZE - 8)
    )
    transport.write(bytes([0]) + report)
    response = transport.read(timeout_ms)
    if not response:
        raise NoResponse(f"命令 {command:#04x} 无响应")
    if len(response) < 9 or response[0:6] != b"STCNAV" or response[6] != PROTO_VERSION:
        raise ProtocolError(
            f"命令 {command:#04x} 响应头异常：{response[:9].hex(' ')}"
        )
    if response[7] != command:
        raise ProtocolError(
            f"命令 {command:#04x} 响应命令不匹配：{response[7]:#04x}"
        )
    if response[8] != 0:
        raise ProtocolError(f"命令 {command:#04x} 失败：status={response[8]}")
    return response[9:]


def print_ping(data: bytes) -> None:
    print("[selftest] PING OK（USB EP1 OUT/IN 正常）")


def print_read_code(data: bytes) -> None:
    if len(data) < 20:
        raise ProtocolError(f"READ_CODE 数据过短：{len(data)}")
    result = data[0]
    bad_app = data[1]
    bad_vec = data[2]
    iap_error = data[3]
    app_bytes = data[4:12]
    vec_bytes = data[12:20]

    print(
        f"[selftest] READ_CODE app={'OK' if result & 1 else 'FAIL'} "
        f"vec={'OK' if result & 2 else 'FAIL'} "
        f"bad_app={bad_app:#04x} bad_vec={bad_vec:#04x} iap_err={iap_error}"
    )
    print(f"[selftest]   __code 0xFE4000: {app_bytes.hex(' ')}")
    print(f"[selftest]   __code 0xFF0000: {vec_bytes.hex(' ')}")

    if iap_error:
        print("[selftest]   IAP CMD_FAIL=1：地址映射或 IAP 使能有问题")


def print_sector(data: bytes) -> None:
    if len(data) < 6:
        raise ProtocolError(f"TEST_SECTOR 数据过短：{len(data)}")
    ok, iap_error, bad_l, bad_h, expected, actual = data[:6]
    bad_index = bad_l | (bad_h << 8)

    if ok:
        print("[selftest] TEST_SECTOR OK（0xFEF000 512 字节擦/写/回读通过）")
        return

    print(
        f"[selftest] TEST_SECTOR FAIL iap_err={iap_error} "
        f"bad_index=0x{bad_index:04X} expected={expected:#04x} actual={actual:#04x}"
    )


def wait_isp(timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if device_present(VID, ISP_PID):
            print("[selftest] 已回到 ROM ISP（34BF:1001）")
            return
        time.sleep(0.05)
    raise ProtocolError("等待 ROM ISP 34BF:1001 超时")


def main() -> int:
    parser = argparse.ArgumentParser(description="stcnav IAP 自检上位机")
    parser.add_argument("--ping", action="store_true", help="只发送 PING")
    parser.add_argument("--read-code", action="store_true", help="比较 __code 与 IAP 读取")
    parser.add_argument("--test-sector", action="store_true", help="测试 0xFEF000 扇区擦写")
    parser.add_argument("--all", action="store_true", help="执行 ping/read-code/test-sector")
    parser.add_argument("--reset-isp", action="store_true", help="复位到 ROM ISP")
    parser.add_argument("--timeout", type=float, default=10, help="测试命令超时秒数")
    args = parser.parse_args()

    if not any([args.ping, args.read_code, args.test_sector, args.all, args.reset_isp]):
        parser.error("没有指定操作")

    try:
        transport = HidTransport(VID, PID)
    except SystemExit as error:
        print(f"FAIL: {error}")
        print("[selftest] 请先通过 ROM ISP 烧录 build/iap_selftest/iap_selftest.hex")
        return 1

    try:
        if args.all or args.ping:
            print_ping(send_command(transport, CMD_PING))

        if args.all or args.read_code:
            print_read_code(
                send_command(transport, CMD_READ_CODE, timeout_ms=int(args.timeout * 1000))
            )

        if args.all or args.test_sector:
            print_sector(
                send_command(transport, CMD_TEST_SECTOR, timeout_ms=int(args.timeout * 1000))
            )

        if args.reset_isp:
            send_command(transport, CMD_RESET_ISP)
            wait_isp(args.timeout)
    except (NoResponse, ProtocolError, OSError) as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        try:
            transport.close()
        except Exception:  # noqa: BLE001
            pass

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
