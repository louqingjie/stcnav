#!/usr/bin/env python3
"""云台串口调参脚本（配合 vehicle_tune 固件）。

接线：
  CH340 RX -> STC P1.1 (UART2 TX)
  CH340 TX -> STC P1.0 (UART2 RX)
  GND 共地
  波特率 230400（UART2 与 UART1 共用 Timer2）
"""

import argparse
import statistics
import sys
import time

import serial


TELEMETRY_FIELDS = [
    "ms", "yaw", "rate", "target", "error", "duty", "dir",
    "kp", "ki", "kd", "mode",
]


def parse_telemetry(line):
    if not line.startswith("T,"):
        return None
    parts = line.split(",")
    if len(parts) != 12:
        return None
    try:
        values = [int(parts[1]), float(parts[2]) / 10.0, float(parts[3]) / 10.0,
                  float(parts[4]) / 10.0, float(parts[5]) / 10.0,
                  int(parts[6]), int(parts[7]), float(parts[8]) / 10.0,
                  float(parts[9]) / 100.0, float(parts[10]) / 10.0,
                  int(parts[11])]
    except ValueError:
        return None
    return dict(zip(TELEMETRY_FIELDS, values))


class Tuner:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.last_keepalive = 0.0

    def close(self):
        try:
            self.send("M0")
            self.send("E0")
        except Exception:
            pass
        self.ser.close()

    def send(self, command):
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()

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
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            sample = parse_telemetry(line)
            if sample is not None:
                samples.append(sample)
            elif line == "READY":
                pass
        return samples

    def wait_ready(self, timeout=25.0):
        end = time.time() + timeout
        while time.time() < end:
            self.keepalive()
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            if line:
                print(f"[board] {line}")
            if line == "READY":
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

    if delta > 0:
        peak = max(yaw)
    else:
        peak = min(yaw)
    overshoot = (peak - target) / delta * 100.0
    if overshoot < 0:
        overshoot = 0.0

    rise_time = None
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
    settle_time = settle_index * 0.02

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
    oscillations = crossings // 2

    dir_changes = 0
    for a, b in zip(samples, samples[1:]):
        if a["dir"] != b["dir"]:
            dir_changes += 1

    return {
        "initial": initial,
        "final": final,
        "peak": peak,
        "overshoot": overshoot,
        "rise": rise_time if rise_time is not None else -1.0,
        "settle": settle_time,
        "oscillations": oscillations,
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
    parser.add_argument("--port", default="COM21")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--kp", type=float, default=30.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--deadband", type=float, default=1.0)
    parser.add_argument("--step", type=float, default=10.0)
    parser.add_argument("--record", type=float, default=2.0)
    parser.add_argument("--auto", action="store_true")
    args = parser.parse_args()

    tuner = Tuner(args.port, args.baud)
    try:
        if not tuner.wait_ready():
            print("没有等到 READY，检查 UART2 接线和固件")
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
