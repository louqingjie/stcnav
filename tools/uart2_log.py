#!/usr/bin/env python3
"""串口原始日志采集：把收到的字节带时间戳写入文件，供后续离线脚本分析。

用法（后台运行，不阻塞）：
    python tools/uart2_log.py --port COM23 --plan "115200:15,230400:600" \
        --out build/uart2_live.log

日志格式（每行一个接收块）：
    <epoch 秒> <hex 字节，空格分隔>

--plan 用逗号分隔 `波特率:秒数` 段，用来跨"启动期 115200 / 运行期 230400"
切换（同一时刻只能有一个进程占用串口，所以由本进程依次切换）。
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")


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
    args = parser.parse_args()

    total_bytes = 0
    start = time.time()
    with open(args.out, "a", encoding="utf-8", buffering=1) as log:
        log.write(f"# port={args.port} plan={args.plan} start={start:.3f}\n")
        for baud, seconds in args.plan:
            try:
                ser = serial.Serial(args.port, baud, timeout=0.05)
            except serial.SerialException as error:
                log.write(f"# open {baud} failed: {error}\n")
                print(f"[log] open {baud} failed: {error}")
                continue
            print(f"[log] {baud} baud for {seconds:.0f}s ...")
            log.write(f"# segment start baud={baud}\n")
            deadline = time.time() + seconds
            try:
                while time.time() < deadline:
                    chunk = ser.read(args.chunk)
                    if chunk:
                        now = time.time()
                        total_bytes += len(chunk)
                        log.write(f"{now:.3f} {chunk.hex(' ')}\n")
            finally:
                ser.close()
                log.write(f"# segment end baud={baud} t={time.time() - start:.1f}s\n")
    elapsed = time.time() - start
    print(f"[log] done: {total_bytes} bytes in {elapsed:.1f}s -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
