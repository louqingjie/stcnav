#!/usr/bin/env python3
"""离线分析 tools/uart2_log.py 采集的串口原始日志。

输出：每个波特率段的字节数、V2 帧统计、按 boot_id 分段的频率、
     心跳字段、编码拒发/阶段标记、静默区间。
"""

from __future__ import annotations

import argparse
import struct
import sys
from collections import Counter

EXPECTED_LEN = {0x03: 13, 0x82: 17, 0x85: 19, 0x88: 55}
TYPE_NAMES = {0x03: "0x03 Heartbeat", 0x82: "0x82 GimbalState",
              0x85: "0x85 HeadingState", 0x88: "0x88 AttitudeState"}


def crc16(buf: bytes) -> int:
    crc = 0xFFFF
    for byte in buf:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def load_log(path: str) -> list[dict]:
    segments: list[dict] = []
    with open(path, "r", encoding="utf-8", errors="replace") as log:
        for line in log:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if "segment start baud=" in line:
                    segments.append({"baud": int(line.split("=")[1]), "chunks": []})
                continue
            if not segments:
                segments.append({"baud": 0, "chunks": []})
            ts_text, _, hex_text = line.partition(" ")
            try:
                data = bytes.fromhex(hex_text)
            except ValueError:
                continue
            segments[-1]["chunks"].append((float(ts_text), data))
    return segments


def analyze_segment(segment: dict) -> None:
    baud = segment["baud"]
    chunks = segment["chunks"]
    total = sum(len(d) for _, d in chunks)
    if not chunks:
        print(f"\n== baud={baud}: 0 bytes")
        return
    t0 = chunks[0][0]
    t1 = chunks[-1][0]
    print(f"\n== baud={baud}: {total} B, {len(chunks)} chunks, "
          f"wall {t1 - t0:.1f}s")

    frames = []   # (t, type, seq, payload)
    markers = []  # (t, char)
    buffer = bytearray()
    times = []
    for ts, data in chunks:
        buffer.extend(data)
        times.extend([ts] * len(data))
    i = 0
    frame_mask = bytearray(len(buffer))
    while i + 8 <= len(buffer):
        if buffer[i] == 0x52 and buffer[i + 1] == 0x4D:
            version = buffer[i + 2]
            mtype = buffer[i + 3]
            length = buffer[i + 4] | (buffer[i + 5] << 8)
            seq = buffer[i + 6] | (buffer[i + 7] << 8)
            end = i + 8 + length + 2
            if version == 2 and end <= len(buffer):
                if crc16(buffer[i + 2:i + 8 + length]) == (buffer[end - 2] | (buffer[end - 1] << 8)):
                    frames.append((times[i], mtype, seq, bytes(buffer[i + 8:i + 8 + length])))
                    for k in range(i, end):
                        frame_mask[k] = 1
                    i = end
                    continue
        i += 1

    counts = Counter(mtype for _, mtype, _, _ in frames)
    print(f"   frames={len(frames)} types={{{', '.join(f'{hex(k)}:{v}' for k, v in sorted(counts.items()))}}}")
    bad_len = sum(1 for _, mt, _, p in frames if mt in EXPECTED_LEN and len(p) != EXPECTED_LEN[mt])
    print(f"   payload 长度不符={bad_len}")

    beats = [(t, seq, p) for t, mt, seq, p in frames if mt == 0x03 and len(p) == 13]
    if beats:
        sessions = []
        for t, seq, payload in beats:
            uptime, boot, caps, ready = struct.unpack("<IIIB", payload)
            if not sessions or sessions[-1]["boot"] != boot:
                sessions.append({"boot": boot, "t0": t, "t1": t, "u0": uptime, "u1": uptime, "n": 1, "ready": ready})
            else:
                s = sessions[-1]
                s["t1"] = t
                s["u1"] = uptime
                s["n"] += 1
                s["ready"] = ready
        print(f"   sessions={len(sessions)}:")
        for s in sessions[-8:]:
            dur = max(s["t1"] - s["t0"], 1e-3)
            print(f"     boot={s['boot']} wall {s['t0'] - t0:6.1f}..{s['t1'] - t0:6.1f}s "
                  f"uptime {s['u0']}..{s['u1']} ms beats={s['n']} "
                  f"({(s['n'] - 1) / dur:.1f} Hz) ready={s['ready']}")
    else:
        print("   no valid heartbeat")

    for _, mt in ((0, 0x03), (0, 0x82), (0, 0x85), (0, 0x88)):
        group = [p for t, mt2, _, p in frames if mt2 == mt]
        if not group:
            continue
        # 速率按“首个到最后一个该类型帧”估算
        ts = [t for t, mt2, _, _ in frames if mt2 == mt]
        span = max(ts[-1] - ts[0], 1e-3)
        print(f"   {TYPE_NAMES[mt]}: {len(group)} 帧, {(len(group) - 1) / span:.1f} Hz")

    reason = {str(k): 0 for k in range(1, 8)}
    for index, byte in enumerate(buffer):
        if frame_mask[index]:
            continue
        if 0x21 <= byte <= 0x7E:
            markers.append((times[index], chr(byte)))
    freq = Counter(c for _, c in markers)
    for key in list(reason):
        reason[key] = freq.get(key, 0)
    print(f"   编码拒发标记 1..7: {reason}")
    print(f"   阶段标记频次: {dict(sorted(freq.items(), key=lambda kv: -kv[1])[:12])}")
    print(f"   最后 60 个标记: {''.join(c for _, c in markers[-60:])}")


def main() -> int:
    parser = argparse.ArgumentParser(description="离线分析串口日志")
    parser.add_argument("log", help="tools/uart2_log.py 写出的日志文件")
    args = parser.parse_args()
    segments = load_log(args.log)
    if not segments:
        print("日志为空")
        return 1
    for segment in segments:
        analyze_segment(segment)
    return 0


if __name__ == "__main__":
    sys.exit(main())
