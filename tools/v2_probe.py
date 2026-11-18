#!/usr/bin/env python3
"""RM2027 V2 上位机链路探测脚本（配合 vehicle 固件）。

接线：
  USB-TTL RX -> STC P1.1 (UART2 TX)
  USB-TTL TX -> STC P1.0 (UART2 RX)
  GND 共地
  波特率 230400（UART2 与 UART1 共用 Timer2）

默认只监听：解码 STM 发出的 heartbeat / gimbal / heading / attitude，
校验 CRC、序号、四元数范数与心跳 uptime，并打印统计。

显式 --chassis VX VY WZ 才会向 STM 发送 0x01 ChassisCommand（100 Hz）
和 0x03 心跳（10 Hz）；退出时自动发送 3 帧零速禁用命令。
注意：当前固件 V2 运动授权默认开启（VEHICLE_V2_ENABLE_MOTION=1），
台架测试必须先把车轮架空或电机物理禁能。
"""

import argparse
import math
import random
import struct
import sys
import time


MAGIC = b"\x52\x4d"
VERSION = 2
HEADER_SIZE = 8
CRC_SIZE = 2
MAX_PAYLOAD = 128

TYPE_CHASSIS_COMMAND = 0x01
TYPE_POSTURE_REQUEST = 0x02
TYPE_HEARTBEAT = 0x03
TYPE_POSTURE_STATE = 0x81
TYPE_GIMBAL_STATE = 0x82
TYPE_REFEREE_STATE = 0x83
TYPE_OPERATOR_TARGET = 0x84
TYPE_CHASSIS_HEADING_STATE = 0x85
TYPE_CHASSIS_ATTITUDE_STATE = 0x88

PAYLOAD_SIZES = {
    TYPE_CHASSIS_COMMAND: 13,
    TYPE_POSTURE_REQUEST: 11,
    TYPE_HEARTBEAT: 13,
    TYPE_POSTURE_STATE: 13,
    TYPE_GIMBAL_STATE: 17,
    TYPE_REFEREE_STATE: 17,
    TYPE_OPERATOR_TARGET: 25,
    TYPE_CHASSIS_HEADING_STATE: 19,
    TYPE_CHASSIS_ATTITUDE_STATE: 55,
}

TYPE_NAMES = {
    TYPE_POSTURE_STATE: "PostureState",
    TYPE_GIMBAL_STATE: "GimbalState",
    TYPE_REFEREE_STATE: "RefereeState",
    TYPE_OPERATOR_TARGET: "OperatorTarget",
    TYPE_CHASSIS_HEADING_STATE: "HeadingState",
    TYPE_CHASSIS_ATTITUDE_STATE: "AttitudeState",
}

CAP_CHASSIS_COMMAND = 1 << 0
CAP_POSTURE = 1 << 1
CAP_GIMBAL_STATE = 1 << 2
CAP_REFEREE_STATE = 1 << 3
CAP_OPERATOR_TARGET = 1 << 4
CAP_CHASSIS_HEADING_STATE = 1 << 5
CAP_CHASSIS_ATTITUDE_STATE = 1 << 8

# docs/stm_v2_protocol_reference.md 第 10 节 golden vector。
GOLDEN_ATTITUDE = bytes.fromhex(
    "52 4d 02 88 37 00 44 33"
    " 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f"
    " 00 00 00 3e 00 00 80 be 00 00 00 3f"
    " 00 00 00 00 00 00 00 00 0a e8 1c 41"
    " 04 03 02 01 e8 03 00 00 01 00 27 20 02 00 0f"
    " 90 be"
)


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def encode_frame(message_type, sequence, payload):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    header = struct.pack("<BBBBHH", MAGIC[0], MAGIC[1], VERSION, message_type,
                         len(payload), sequence)
    body = header + payload
    crc = crc16_modbus(body[2:])
    return body + struct.pack("<H", crc)


def encode_heartbeat(uptime_ms, boot_id, capabilities, ready, sequence):
    payload = struct.pack("<III B", uptime_ms & 0xFFFFFFFF, boot_id & 0xFFFFFFFF,
                          capabilities & 0xFFFFFFFF, 1 if ready else 0)
    return encode_frame(TYPE_HEARTBEAT, sequence, payload)


def encode_chassis_command(vx, vy, wz, enabled, sequence):
    payload = struct.pack("<fff B", vx, vy, wz, 1 if enabled else 0)
    return encode_frame(TYPE_CHASSIS_COMMAND, sequence, payload)


class StreamDecoder:
    """与固件相同语义：版本/长度/CRC 错误丢弃 1 字节重扫。"""

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.framing_errors = 0

    def append(self, data):
        self.buffer.extend(data)

    def pop(self):
        while True:
            index = self.buffer.find(MAGIC)
            if index < 0:
                keep = 1 if self.buffer[-1:] == MAGIC[:1] else 0
                self.framing_errors += len(self.buffer) - keep
                self.buffer = self.buffer[-keep:] if keep else bytearray()
                return None
            if index > 0:
                self.framing_errors += index
                del self.buffer[:index]
            if len(self.buffer) < HEADER_SIZE:
                return None
            if self.buffer[2] != VERSION:
                self.framing_errors += 1
                del self.buffer[:1]
                continue
            payload_length = struct.unpack_from("<H", self.buffer, 4)[0]
            if payload_length > MAX_PAYLOAD:
                self.framing_errors += 1
                del self.buffer[:1]
                continue
            total = HEADER_SIZE + payload_length + CRC_SIZE
            if len(self.buffer) < total:
                return None
            expected = crc16_modbus(self.buffer[2:8 + payload_length])
            received = struct.unpack_from("<H", self.buffer, 8 + payload_length)[0]
            if expected != received:
                self.crc_errors += 1
                del self.buffer[:1]
                continue
            message_type = self.buffer[3]
            sequence = struct.unpack_from("<H", self.buffer, 6)[0]
            payload = bytes(self.buffer[8:8 + payload_length])
            del self.buffer[:total]
            return message_type, sequence, payload


class Monitor:
    def __init__(self):
        self.heartbeat = None
        self.last_heartbeat_ms = 0
        self.boot_id = 0
        self.ready = False
        self.capabilities = 0
        self.sequence = {}
        self.sequence_errors = 0
        self.attitude_norm_bad = 0
        self.attitude_count = 0
        self.heading_yaw = None
        self.counts = {}
        self.last_print = 0.0
        self.start = time.time()
        self.decoder_crc = 0
        self.framing_errors = 0
        self.last_attitude = None

    def feed(self, message_type, sequence, payload):
        self.counts[message_type] = self.counts.get(message_type, 0) + 1
        last = self.sequence.get(message_type)
        if last is not None and ((sequence - last) & 0xFFFF) not in range(1, 0x8000):
            self.sequence_errors += 1
        self.sequence[message_type] = sequence

        if message_type == TYPE_HEARTBEAT and len(payload) == 13:
            uptime, boot, caps, ready = struct.unpack("<IIIB", payload)
            if boot != self.boot_id:
                self.sequence.clear()
                self.boot_id = boot
            self.heartbeat = (uptime, boot, caps, ready)
            self.last_heartbeat_ms = uptime
            self.ready = bool(ready)
            self.capabilities = caps
        elif message_type == TYPE_GIMBAL_STATE and len(payload) == 17:
            yaw, rate, seq, mcu, flags = struct.unpack("<ffIIB", payload)
            self._yaw = yaw
        elif message_type == TYPE_CHASSIS_HEADING_STATE and len(payload) == 19:
            yaw, rate, seq, mcu, reset, flags = struct.unpack("<ffIIHB", payload)
            self.heading_yaw = yaw
        elif message_type == TYPE_CHASSIS_ATTITUDE_STATE and len(payload) == 55:
            values = struct.unpack("<10f III HB", payload)
            qx, qy, qz, qw = values[0:4]
            seq = values[10]
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            self.attitude_count += 1
            if abs(norm - 1.0) > 1.0e-3:
                self.attitude_norm_bad += 1
            self.last_attitude = values

    def print_status(self, force=False):
        now = time.time()
        if not force and now - self.last_print < 1.0:
            return
        self.last_print = now
        elapsed = max(now - self.start, 1e-6)
        if self.heartbeat is None:
            print("[v2] 尚无 heartbeat……检查接线/波特率/固件是否运行")
            return
        uptime, boot, caps, ready = self.heartbeat
        rates = " ".join(
            f"{TYPE_NAMES.get(t, hex(t))}={(c / elapsed):.1f}Hz"
            for t, c in sorted(self.counts.items())
        )
        print(
            f"[v2] boot=0x{boot:08X} uptime={uptime}ms ready={int(ready)} "
            f"caps=0x{caps:08X} seq_err={self.sequence_errors} "
            f"crc_err={self.decoder_crc} firmware_frame_err={self.framing_errors} "
            f"att_norm_bad={self.attitude_norm_bad}/{self.attitude_count}"
        )
        print(f"     {rates}")


def run_golden_check():
    frame = encode_frame(TYPE_CHASSIS_COMMAND, 0x1234, struct.pack("<fffB", 1.0, -2.0, 0.5, 1))
    expected = bytes.fromhex(
        "52 4d 02 01 0d 00 34 12 00 00 80 3f 00 00 00 c0 00 00 00 3f 01 e6 cb"
    )
    ok = frame == expected
    print(f"golden 0x01: {'PASS' if ok else 'FAIL'} {frame.hex(' ')}")
    if not ok:
        print(f"  expected: {expected.hex(' ')}")

    decoder = StreamDecoder()
    decoder.append(GOLDEN_ATTITUDE)
    decoded = decoder.pop()
    if decoded is None:
        print("golden 0x88: FAIL (decode)")
        return 1
    message_type, sequence, payload = decoded
    values = struct.unpack("<10f III HB", payload)
    norm = math.sqrt(sum(v * v for v in values[0:4]))
    ok = (
        message_type == TYPE_CHASSIS_ATTITUDE_STATE
        and sequence == 0x3344
        and abs(norm - 1.0) <= 1.0e-3
        and abs(values[9] - 9.80665) < 1e-3
    )
    print(f"golden 0x88: {'PASS' if ok else 'FAIL'} seq=0x{sequence:04X} |q|={norm:.6f}")
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description="RM2027 V2 上位机链路探测")
    parser.add_argument("--port", help="串口，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--duration", type=float, default=0.0, help="监听时长（秒），0=一直运行")
    parser.add_argument("--chassis", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
                        help="发送底盘速度命令（m/s, m/s, rad/s）；默认不发送")
    parser.add_argument("--golden", action="store_true", help="本地校验 golden vector 后退出")
    parser.add_argument("--boot-id", type=lambda x: int(x, 0),
                        help="模拟上位机 boot_id（默认随机非零）")
    args = parser.parse_args()

    if args.golden:
        return run_golden_check()
    if not args.port:
        parser.error("需要 --port（或使用 --golden）")

    try:
        import serial
    except ImportError:  # pragma: no cover
        print("需要 pyserial：pip install pyserial", file=sys.stderr)
        return 2

    serial_port = serial.Serial(args.port, args.baud, timeout=0.02)
    decoder = StreamDecoder()
    monitor = Monitor()
    rng = random.Random()
    host_boot_id = args.boot_id or (rng.getrandbits(31) | 1)
    heartbeat_seq = 0
    command_seq = 0
    next_command = 0.0
    next_heartbeat = 0.0
    end = time.time() + args.duration if args.duration > 0 else None

    motion = args.chassis is not None
    if motion:
        vx, vy, wz = args.chassis
        print(f"!! 将发送 ChassisCommand vx={vx} vy={vy} wz={wz}；"
              f"当前固件默认允许 V2 驱动底盘，台架务必架空/电机禁能")
    else:
        print("监听模式（不发送运动命令）。用 --chassis VX VY WZ 才发送底盘命令。")

    try:
        while end is None or time.time() < end:
            now = time.time()

            if motion and now >= next_heartbeat:
                next_heartbeat = now + 0.1
                frame = encode_heartbeat(
                    int((now - monitor.start) * 1000), host_boot_id,
                    CAP_CHASSIS_COMMAND, True, heartbeat_seq)
                heartbeat_seq = (heartbeat_seq + 1) & 0xFFFF
                serial_port.write(frame)

            if motion and now >= next_command:
                next_command = now + 0.01
                frame = encode_chassis_command(vx, vy, wz, True, command_seq)
                command_seq = (command_seq + 1) & 0xFFFF
                serial_port.write(frame)

            data = serial_port.read(256)
            if data:
                decoder.append(data)
                while True:
                    decoded = decoder.pop()
                    if decoded is None:
                        break
                    monitor.feed(*decoded)
                monitor.decoder_crc = decoder.crc_errors
                monitor.framing_errors = decoder.framing_errors
            monitor.print_status()
    except KeyboardInterrupt:
        pass
    finally:
        if motion:
            for _ in range(3):
                serial_port.write(encode_chassis_command(0.0, 0.0, 0.0, False, command_seq))
                command_seq = (command_seq + 1) & 0xFFFF
                time.sleep(0.01)
        monitor.print_status(force=True)
        print(f"[v2] 总计: crc_err={decoder.crc_errors} framing_err={decoder.framing_errors} "
              f"seq_err={monitor.sequence_errors}")
        serial_port.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
