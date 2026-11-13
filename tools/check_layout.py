#!/usr/bin/env python3
"""HEX/MAP 布局校验。

规则（计划文档第 2 节）：
  - 应用代码/常量：0xFE0000-0xFEFFFF
  - 中断向量：    0xFF0000-0xFF0FFF
  - 禁止写入：    >= 0xFF1000
  - EDATA：       < 0x1000
  - XRAM：        < 0x012000
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

APP_START = 0xFE0000
APP_END = 0xFEFFFF
VECTOR_START = 0xFF0000
VECTOR_END = 0xFF0FFF
HIGH_GUARD = 0xFF1000
CODE_START_DEFAULT = 0xFE4000
CODE_END_DEFAULT = 0xFEFFFF

EDATA_END = 0x0FFF
XRAM_END = 0x011FFF
XRAM_SIZE = 0x2000

HEX_LINE_RE = re.compile(r"^:([0-9A-Fa-f]+)$")
MEM_ROM_RE = re.compile(
    r"ROM/EPROM/FLASH\s+0x([0-9A-Fa-f]+)\s+0x([0-9A-Fa-f]+)"
)
MEM_XRAM_RE = re.compile(r"EXTERNAL RAM\s+(\d+)\s+(\d+)")
MEM_STACK_RE = re.compile(
    r"initial stack starts at:\s*0x([0-9A-Fa-f]+).*?with\s+(\d+)\s+bytes"
)
MAP_HEAP_RE = re.compile(r"^\s*C:\s+([0-9A-Fa-f]{8})\s+_ucHeap\b")
MAP_SYMBOL_RE = re.compile(r"^\s*C:\s+([0-9A-Fa-f]{8})\s+(\S+)\s*$")


class LayoutError(Exception):
    pass


def parse_hex(path: Path) -> dict[int, int]:
    """解析 Intel HEX，返回 {地址: 字节}。"""
    image: dict[int, int] = {}
    base = 0
    seen_eof = False

    for lineno, raw_line in enumerate(
        path.read_text(encoding="ascii", errors="replace").splitlines(), 1
    ):
        line = raw_line.strip()
        if not line:
            continue
        match = HEX_LINE_RE.match(line)
        if not match:
            raise LayoutError(f"{path}:{lineno}: 非法 HEX 行")
        try:
            record = bytes.fromhex(match.group(1))
        except ValueError as error:
            raise LayoutError(f"{path}:{lineno}: HEX 编码错误：{error}") from error
        if len(record) < 5:
            raise LayoutError(f"{path}:{lineno}: HEX 记录过短")
        length = record[0]
        address = (record[1] << 8) | record[2]
        record_type = record[3]
        if len(record) != length + 5:
            raise LayoutError(f"{path}:{lineno}: HEX 长度字段不符")
        if (sum(record) & 0xFF) != 0:
            raise LayoutError(f"{path}:{lineno}: HEX 校验和错误")
        payload = record[4:-1]

        if record_type == 0x00:
            for offset, value in enumerate(payload):
                target = base + address + offset
                if target > 0xFFFFFF:
                    raise LayoutError(f"{path}:{lineno}: 地址超出 24 位")
                image[target] = value
        elif record_type == 0x01:
            seen_eof = True
            break
        elif record_type == 0x02:
            if len(payload) != 2:
                raise LayoutError(f"{path}:{lineno}: 扩展段地址记录非法")
            base = ((payload[0] << 8) | payload[1]) << 4
        elif record_type == 0x04:
            if len(payload) != 2:
                raise LayoutError(f"{path}:{lineno}: 扩展线性地址记录非法")
            base = ((payload[0] << 8) | payload[1]) << 16
        elif record_type in (0x03, 0x05):
            continue
        else:
            raise LayoutError(f"{path}:{lineno}: 不支持的记录类型 {record_type:#04x}")

    if not image:
        raise LayoutError(f"{path}: 没有数据记录")
    if not seen_eof:
        raise LayoutError(f"{path}: 缺少 EOF 记录")
    return image


def check_image(
    image: dict[int, int],
    code_start: int = CODE_START_DEFAULT,
    code_end: int = CODE_END_DEFAULT,
) -> dict[str, int]:
    app = [address for address in image if code_start <= address <= code_end]
    outside = [
        address
        for address in image
        if APP_START <= address <= APP_END and not (code_start <= address <= code_end)
    ]
    vector = [address for address in image if VECTOR_START <= address <= VECTOR_END]
    high = [address for address in image if address >= HIGH_GUARD]
    low = [address for address in image if address < APP_START]

    if high:
        raise LayoutError(
            f"存在 0xFF1000 以上的数据（最低地址 0x{min(high):06X}）；"
            "ROM USB-ISP 对高区擦写不可靠"
        )
    if low:
        raise LayoutError(
            f"存在 0xFE0000 以下的数据（最低地址 0x{min(low):06X}）"
        )
    if outside:
        raise LayoutError(
            f"存在 0x{code_start:06X}-0x{code_end:06X} 以外的代码数据"
            f"（最低地址 0x{min(outside):06X}）"
        )
    if not app:
        raise LayoutError(
            f"HEX 中没有 0x{code_start:06X}-0x{code_end:06X} 的应用数据"
        )

    return {
        "app_min": min(app),
        "app_max": max(app),
        "app_bytes": len(app),
        "vector_min": min(vector) if vector else 0,
        "vector_max": max(vector) if vector else 0,
        "vector_bytes": len(vector),
    }


def check_mem(
    mem_path: Path,
    code_start: int = CODE_START_DEFAULT,
    code_end: int = CODE_END_DEFAULT,
) -> dict[str, int]:
    result: dict[str, int] = {}
    if not mem_path.is_file():
        return result

    text = mem_path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        rom = MEM_ROM_RE.search(line)
        if rom:
            start = int(rom.group(1), 16)
            end = int(rom.group(2), 16)
            result["rom_start"] = start
            result["rom_end"] = end
            if start < code_start:
                raise LayoutError(
                    f"ROM 起点 0x{start:06X} 低于 0x{code_start:06X}"
                )
            if end >= HIGH_GUARD:
                raise LayoutError(
                    f"ROM 末尾 0x{end:06X} 进入 0xFF1000 以上区域"
                )
            continue

        xram = MEM_XRAM_RE.search(line)
        if xram:
            used = int(xram.group(1))
            maximum = int(xram.group(2))
            result["xram_used"] = used
            result["xram_max"] = maximum
            if maximum > XRAM_SIZE:
                raise LayoutError(f"XRAM 上限 {maximum} 超过 0x2000")
            if used > XRAM_SIZE:
                raise LayoutError(f"XRAM 占用 {used} 超过 0x2000")
            continue

        stack = MEM_STACK_RE.search(line)
        if stack:
            start = int(stack.group(1), 16)
            available = int(stack.group(2))
            result["stack_start"] = start
            result["stack_bytes"] = available
            if start + available > EDATA_END + 1:
                raise LayoutError(
                    f"EDATA 栈越界：0x{start:04X}+{available} 超过 0x0FFF"
                )

    return result


def check_map(map_path: Path) -> dict[str, int]:
    result: dict[str, int] = {}
    if not map_path.is_file():
        return result

    text = map_path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        heap = MAP_HEAP_RE.match(line)
        if heap:
            address = int(heap.group(1), 16)
            result["heap_address"] = address
            if not (0x0080 <= address <= EDATA_END):
                raise LayoutError(
                    f"ucHeap 地址 0x{address:06X} 不在 EDATA 可用范围"
                )
            continue

        symbol = MAP_SYMBOL_RE.match(line)
        if symbol:
            name = symbol.group(2)
            # 链接器生成的 l_*/s_* 是区域基址/尺寸，不是对象地址。
            if name.startswith(("l_", "s_", ".__")):
                continue
            address = int(symbol.group(1), 16)
            # 0x1000-0xFFFF 既不是 EDATA，也不是 XSEG（从 0x010000 起）。
            if 0x1000 <= address < 0x10000:
                raise LayoutError(
                    f"符号 {name} 位于 0x{address:06X}，"
                    "超出 EDATA 0x0000-0x0FFF"
                )

    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="校验 stcnav HEX/MAP 布局")
    parser.add_argument("hex", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--mem", type=Path)
    parser.add_argument(
        "--code-start", type=lambda value: int(value, 0),
        default=CODE_START_DEFAULT,
    )
    parser.add_argument(
        "--code-end", type=lambda value: int(value, 0),
        default=CODE_END_DEFAULT,
    )
    args = parser.parse_args()

    try:
        image = parse_hex(args.hex)
        summary = check_image(image, args.code_start, args.code_end)
        if args.mem:
            summary.update(check_mem(args.mem, args.code_start, args.code_end))
        summary.update(check_map(args.map) if args.map else {})
    except (LayoutError, OSError) as error:
        print(f"FAIL: {error}")
        return 1

    print(
        f"[layout] app    0x{summary['app_min']:06X}-0x{summary['app_max']:06X} "
        f"({summary['app_bytes']} bytes)"
    )
    if summary["vector_bytes"]:
        print(
            f"[layout] vector 0x{summary['vector_min']:06X}-"
            f"0x{summary['vector_max']:06X} ({summary['vector_bytes']} bytes)"
        )
    else:
        print("[layout] vector 缺失（没有中断向量数据）")
    if "stack_start" in summary:
        print(
            f"[layout] edata  stack 0x{summary['stack_start']:04X} "
            f"+{summary['stack_bytes']} bytes"
        )
    if "xram_used" in summary:
        print(
            f"[layout] xram   {summary['xram_used']} / "
            f"{summary['xram_max']} bytes"
        )
    if "heap_address" in summary:
        print(f"[layout] heap   0x{summary['heap_address']:06X}")

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
