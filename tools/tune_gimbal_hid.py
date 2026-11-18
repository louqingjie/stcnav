#!/usr/bin/env python3
"""云台 USB HID 调参脚本（配合 vehicle_tune_hid 固件）。

固件枚举为 VID 34BF / PID FF01 的 64 字节通用 HID。
用法：
  python tools/tune_gimbal_hid.py --auto
  python tools/tune_gimbal_hid.py --kp 30 --kd 1
"""

import argparse
import statistics
import sys
import time

import hid

VID = 0x34BF
PID = 0xFF01
REPORT = 64


def open_device():
    devices = hid.enumerate(VID, PID)
    if not devices:
        print("没有找到 VID=34BF PID=FF01 的 HID 设备。")
        print("请确认已烧录 vehicle_tune_hid.hex，并用 USB 线连接板子。")
        return None
    for item in devices:
        print(
            f"HID: {item.get('product_string')} "
            f"path={item.get('path')}"
        )
    device = hid.device()
    device.open_path(devices[0]["path"])
    device.set_nonblocking(0)
    return device


class Tuner:
    def __init__(self, device):
        self.device = device
        self.last_keepalive = 0.0

    def close(self):
        try:
            self.send("M0")
            self.send("E0")
        except Exception:
            pass
        self.device.close()

    def send(self, command):
        payload = (command + "\n").encode("ascii")
        if len(payload) >= REPORT:
            payload = payload[:REPORT - 1]
        data = [0] + list(payload) + [0] * (REPORT - len(payload))
        self.device.write(data)

    def keepalive(self):
        now = time.time()
        if now - self.last_keepalive > 0.4:
            self.send("K")
            self.last_keepalive = now

    def read_for(self, seconds):
        samples = []
        end = time.time() + seconds
        while time.time() < end:
            self.keepalive()
            raw = self.device.read(REPORT, timeout_ms=50)
            if not raw:
                continue
            data = bytes(raw)
            if 0 in data:
                data = data.split(b"\x00", 1)[0]
            text = data.decode("ascii", errors="ignore")
            for line in text.replace("\r", "\n").split("\n"):
                if not line.startswith("T,"):
                    continue
                parts = line.split(",")
                if len(parts) != 8:
                    continue
                try:
                    samples.append({
                        "ms": int(parts[1]),
                        "yaw": float(parts[2]) / 10.0,
                        "rate": float(parts[3]) / 10.0,
                        "target": float(parts[4]) / 10.0,
                        "duty": int(parts[5]),
                        "dir": int(parts[6]),
                        "mode": int(parts[7]),
                    })
                except ValueError:
                    continue
        return samples

    def wait_ready(self, timeout=25.0):
        end = time.time() + timeout
        while time.time() < end:
            self.keepalive()
            raw = self.device.read(REPORT, timeout_ms=100)
            if not raw:
                continue
            data = bytes(raw)
            if 0 in data:
                data = data.split(b"\x00", 1)[0]
            text = data.decode("ascii", errors="ignore")
            for line in text.replace("\r", "\n").split("\n"):
                if line:
                    print(f"[hid] {line}")
                if line.startswith("T,"):
                    return True
        return False

    def set_gains(self, kp, ki, kd, deadband):
        self.send(f"P{kp:.3f}")
        self.send(f"I{ki:.3f}")
        self.send(f"D{kd:.3f}")
        self.send(f"B{deadband:.3f}")
        self.read_for(0.2)

    def step_test(self, step, settle=0.8, record=2.0):
        self.send("E0")
        self.send("S0")
        self.read_for(settle)
        self.send("E1")
        self.send("S0")
        self.read_for(settle)
        self.send(f"S{step:.2f}")
        samples = self.read_for(record)
        self.send("E0")
        return samples


def wrap180(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def step_metrics(samples, step):
    if len(samples) < 10:
        return None
    yaw = [s["yaw"] for s in samples]
    target = float(step)
    initial = statistics.mean(yaw[:5])
    final = statistics.mean(yaw[-10:])
    delta = target - initial
    if abs(delta) < 1e-6:
        return None

    peak = max(yaw) if delta > 0 else min(yaw)
    overshoot = max(0.0, (peak - target) / delta * 100.0)

    rise_time = -1.0
    for i, value in enumerate(yaw):
        if (delta > 0 and value >= initial + 0.9 * delta) or \
           (delta < 0 and value <= initial + 0.9 * delta):
            rise_time = i * 0.02
            break

    tolerance = max(0.5, abs(delta) * 0.02)
    settle_index = 0
    for i, value in enumerate(yaw):
        if abs(value - target) > tolerance:
            settle_index = i

    crossings = 0
    previous = yaw[0] - target
    for value in yaw[1:]:
        current = value - target
        if previous == 0:
            previous = current
            continue
        if (current > 0) != (previous > 0):
            crossings += 1
        previous = current

    dir_changes = sum(
        1 for a, b in zip(samples, samples[1:]) if a["dir"] != b["dir"]
    )

    return {
        "overshoot": overshoot,
        "rise": rise_time,
        "settle": settle_index * 0.02,
        "oscillations": crossings // 2,
        "max_rate": max(abs(s["rate"]) for s in samples),
        "max_duty": max(abs(s["duty"]) for s in samples),
        "dir_changes": dir_changes,
        "steady_error": abs(final - target),
        "samples": len(samples),
    }


def print_metrics(name, m):
    if m is None:
        print(f"{name}: 没有足够数据")
        return
    print(
        f"{name}: overshoot={m['overshoot']:.1f}% "
        f"rise={m['rise']:.2f}s settle={m['settle']:.2f}s "
        f"osc={m['oscillations']} steady_err={m['steady_error']:.2f}deg "
        f"max_rate={m['max_rate']:.1f}dps max_duty={m['max_duty']} "
        f"dir_changes={m['dir_changes']}"
    )


def score(m):
    if m is None:
        return 1e9
    return (
        m["overshoot"] * 1.5
        + m["settle"] * 20.0
        + m["oscillations"] * 8.0
        + m["steady_error"] * 10.0
        + m["dir_changes"] * 1.0
        + (20.0 if m["settle"] >= 1.5 else 0.0)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kp", type=float, default=30.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--deadband", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=10.0)
    parser.add_argument("--record", type=float, default=2.0)
    parser.add_argument("--auto", action="store_true")
    args = parser.parse_args()

    device = open_device()
    if device is None:
        return 1

    tuner = Tuner(device)
    try:
        if not tuner.wait_ready():
            print("没有收到遥测。检查云台 IMU、拓展板和固件是否运行。")
            return 1

        tuner.set_gains(args.kp, args.ki, args.kd, args.deadband)

        if not args.auto:
            samples = tuner.step_test(args.step, record=args.record)
            print_metrics(
                f"KP={args.kp:.1f} KI={args.ki:.2f} KD={args.kd:.1f}",
                step_metrics(samples, args.step),
            )
            return 0

        results = []
        for kp in (15.0, 20.0, 25.0, 30.0, 40.0, 50.0):
            for kd in (0.0, 1.0, 2.0, 3.0):
                tuner.set_gains(kp, 0.0, kd, args.deadband)
                samples = tuner.step_test(args.step, record=args.record)
                m = step_metrics(samples, args.step)
                print_metrics(f"KP={kp:.1f} KD={kd:.1f}", m)
                results.append((score(m), kp, kd, m))

        results.sort(key=lambda item: item[0])
        best = results[0]
        print(
            f"\n最佳: KP={best[1]:.1f} KD={best[2]:.1f} "
            f"score={best[0]:.1f}"
        )
        tuner.set_gains(best[1], 0.0, best[2], args.deadband)
        samples = tuner.step_test(args.step, record=args.record)
        print_metrics("最终验证", step_metrics(samples, args.step))
        return 0
    finally:
        tuner.close()


if __name__ == "__main__":
    sys.exit(main())
