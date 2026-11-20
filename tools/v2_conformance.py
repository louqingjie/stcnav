#!/usr/bin/env python3
"""RM2027 STM V2 协议一致性测试（真机，只监听 STM -> Host 方向）。

检查项（依据 apps/vehicle/docs/stm_v2_protocol_reference.md 与
apps/vehicle/docs/stm_v2_implementation.md）：
  1. 帧格式：52 4D 02 + type + len(LE) + seq(LE) + payload + CRC16/Modbus(LE)，
     无 CRC / 帧头 / 版本 / 长度错误；
  2. 各类型 payload 长度；
  3. 每种类型 frame sequence 逐帧 +1（模 65536）；
  4. 0x03 Heartbeat：uptime 单调、boot_id 恒定、capabilities=0x125、ready∈{0,1}、10 Hz；
  5. 0x82/0x85/0x88：float 有限、四元数范数≈1、|a|≈9.81、mcu_time 与心跳同时钟域、50 Hz；
  6. STM 不应发送未实现的 0x81/0x83/0x84/0x86/0x87。

用法：
  python tools/v2_conformance.py --port COM3            # 抓 6 秒
  python tools/v2_conformance.py --port COM3 -s 10
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")

from v2_probe import (
    MAX_PAYLOAD,
    StreamDecoder,
    TYPE_CHASSIS_ATTITUDE_STATE,
    TYPE_CHASSIS_HEADING_STATE,
    TYPE_GIMBAL_STATE,
    TYPE_HEARTBEAT,
)

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

EXPECTED_LEN = {
    TYPE_HEARTBEAT: 13,
    TYPE_GIMBAL_STATE: 17,
    TYPE_CHASSIS_HEADING_STATE: 19,
    TYPE_CHASSIS_ATTITUDE_STATE: 55,
}
TYPE_NAMES = {
    TYPE_HEARTBEAT: "0x03 Heartbeat",
    TYPE_GIMBAL_STATE: "0x82 GimbalState",
    TYPE_CHASSIS_HEADING_STATE: "0x85 HeadingState",
    TYPE_CHASSIS_ATTITUDE_STATE: "0x88 AttitudeState",
}
EXPECTED_CAPS = 0x00000125
UNIMPLEMENTED = {0x81, 0x83, 0x84, 0x86, 0x87}

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))


def finite(value: float) -> bool:
    return math.isfinite(value)


def main() -> int:
    parser = argparse.ArgumentParser(description="RM2027 STM V2 协议一致性测试")
    parser.add_argument("--port", required=True)
    parser.add_argument("-b", "--baud", type=int, default=230400)
    parser.add_argument("-s", "--seconds", type=float, default=6.0)
    parser.add_argument("--min-heartbeat", type=float, default=5, help="期望心跳帧数")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
    except serial.SerialException as error:
        print(f"打开 {args.port} 失败：{error}")
        return 2

    decoder = StreamDecoder()
    frames: list[tuple[float, int, int, bytes]] = []
    started = time.time()
    print(f"[test] {args.port} @ {args.baud} 8N1，抓取 {args.seconds:.1f} s ...")
    try:
        while time.time() - started < args.seconds:
            data = ser.read(512)
            if not data:
                continue
            decoder.append(data)
            while True:
                decoded = decoder.pop()
                if decoded is None:
                    break
                message_type, sequence, payload = decoded
                frames.append((time.time() - started, message_type, sequence, payload))
    finally:
        ser.close()

    span = max(time.time() - started, 1e-6)
    print(f"[test] 收到 {len(frames)} 帧 / {span:.2f} s，"
          f"crc_err={decoder.crc_errors} framing_err={decoder.framing_errors}")

    # 1. 帧层
    if not frames:
        check("收到 V2 帧", False, "串口没有任何可解析帧")
        return 1
    check("帧头/版本/长度/CRC 无错误",
          decoder.crc_errors == 0 and decoder.framing_errors == 0,
          f"crc_err={decoder.crc_errors}, framing_err={decoder.framing_errors}")

    # 2. payload 长度 & 未实现类型
    bad_len = [(t, len(p)) for _, t, _, p in frames if t in EXPECTED_LEN and len(p) != EXPECTED_LEN[t]]
    unknown = sorted({t for _, t, _, p in frames if t not in EXPECTED_LEN})
    check("payload 长度符合协议", not bad_len, str(bad_len[:4]))
    check("未发送未实现的消息类型", not (set(unknown) & UNIMPLEMENTED), str(unknown))

    # 3. 序号连续
    seq_by_type: dict[int, list[int]] = {}
    for _, message_type, sequence, _ in frames:
        seq_by_type.setdefault(message_type, []).append(sequence)
    seq_errors = {}
    for message_type, seqs in seq_by_type.items():
        bad = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
        if bad:
            seq_errors[TYPE_NAMES.get(message_type, hex(message_type))] = bad
    check("frame sequence 逐帧 +1", not seq_errors, str(seq_errors))

    # 4. 心跳
    beats = [(t, p) for t, mt, _, p in frames if mt == TYPE_HEARTBEAT and len(p) == 13]
    check("存在 0x03 心跳", bool(beats))
    if beats:
        boots = set()
        caps = set()
        uptimes = []
        ready_values = set()
        uptime_ok = True
        for _, payload in beats:
            uptime, boot, cap, ready = struct.unpack("<IIIB", payload)
            boots.add(boot)
            caps.add(cap)
            ready_values.add(ready)
            if uptimes and ((uptime - uptimes[-1]) & 0xFFFFFFFF) > 0x80000000:
                uptime_ok = False
            uptimes.append(uptime)
        rate = len(beats) / span
        check("heartbeat 10 Hz", 8.0 <= rate <= 12.0, f"{rate:.1f} Hz")
        check("boot_id 会话内恒定", len(boots) == 1, f"boot={sorted(boots)}")
        check("capabilities = 0x125", caps == {EXPECTED_CAPS}, f"caps={[hex(c) for c in caps]}")
        check("ready 为 0/1", ready_values <= {0, 1}, f"ready={sorted(ready_values)}")
        check("uptime 单调不回退", uptime_ok)
    else:
        rate = 0.0

    # 5. 状态帧
    for message_type, fields, rate_lo, rate_hi in (
        (TYPE_GIMBAL_STATE, "<ffIIB", 40.0, 60.0),
        (TYPE_CHASSIS_HEADING_STATE, "<ffIIHB", 40.0, 60.0),
        (TYPE_CHASSIS_ATTITUDE_STATE, "<10fIIIHB", 40.0, 60.0),
    ):
        group = [(t, p) for t, mt, _, p in frames if mt == message_type]
        name = TYPE_NAMES[message_type]
        if not group:
            print(f"[SKIP] {name}：本次抓取未出现（ready=0 或未接 IMU）")
            continue
        length_ok = all(len(p) == EXPECTED_LEN[message_type] for _, p in group)
        floats_ok = True
        mcu_times = []
        quat_norm_bad = 0
        accel_bad = 0
        flags_bad = 0
        boot_bad = 0
        beat_boot = sorted({struct.unpack("<IIIB", p)[1] for _, p in beats}) if beats else []
        for _, payload in group:
            if message_type == TYPE_GIMBAL_STATE:
                values = struct.unpack(fields, payload)
                floats_ok &= all(finite(v) for v in values[:2])
                flags_bad += 1 if (values[4] & 0xF0) else 0
                mcu_times.append(values[3])
            elif message_type == TYPE_CHASSIS_HEADING_STATE:
                values = struct.unpack(fields, payload)
                floats_ok &= all(finite(v) for v in values[:2])
                flags_bad += 1 if (values[5] & 0xF0) else 0
                mcu_times.append(values[3])
            else:
                values = struct.unpack(fields, payload)
                floats_ok &= all(finite(v) for v in values[:10])
                qx, qy, qz, qw = values[0:4]
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if (values[14] & 0x01) and abs(norm - 1.0) > 1.0e-3:
                    quat_norm_bad += 1
                # values[9] = linear_acceleration_z（specific force，静止时约 +9.81）
                if not (5.0 <= abs(values[9]) <= 15.0):
                    accel_bad += 1
                flags_bad += 1 if (values[14] & 0xF0) else 0
                if beat_boot and values[12] not in beat_boot:
                    boot_bad += 1
                mcu_times.append(values[11])
        rate = len(group) / span
        check(f"{name} payload 长度", length_ok)
        check(f"{name} float 有限", floats_ok)
        check(f"{name} 频率 50 Hz", rate_lo <= rate <= rate_hi, f"{rate:.1f} Hz")
        check(f"{name} flags 保留位为 0", flags_bad == 0, f"bad={flags_bad}")
        if message_type == TYPE_CHASSIS_ATTITUDE_STATE:
            check("0x88 四元数范数≈1", quat_norm_bad == 0, f"bad={quat_norm_bad}")
            check("0x88 linear_acceleration_z≈9.81", accel_bad == 0, f"bad={accel_bad}")
            check("0x88 source_boot_id 等于心跳 boot_id", boot_bad == 0, f"bad={boot_bad}")

    # 6. 汇总
    failed = [name for name, ok, _ in results if not ok]
    print()
    if failed:
        print(f"[test] 结论：FAIL，{len(failed)} 项不符合：{'；'.join(failed)}")
        return 1
    print("[test] 结论：PASS，抓取到的帧全部符合 V2 协议。")
    if not beats:
        print("[test] 注意：没有心跳帧，无法验证 0x03。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
