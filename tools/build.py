#!/usr/bin/env python3
"""stcnav 构建入口。

读取 tools/build_config.json，逐文件编译为 .rel，链接 .hex/.map，
校验布局，并可选择烧录或运行 QEMU。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO / "tools" / "build_config.json"
GENERATED_HEADER = "generated_interrupt_declarations.h"

INTERRUPT_RE = re.compile(
    r"\bvoid\s+([A-Za-z_]\w*)\s*\([^)]*\)[^;{]*"
    r"__interrupt\s*\(\s*(\d+)\s*\)"
)

QUIET_WARNINGS = (
    "__has_builtin",
    "__STDC_HOSTED__",
    "previous definition",
)


def load_config() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def rel(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else REPO / path


def find_sdcc(config: dict, override: str | None) -> str:
    candidates = []
    if override:
        candidates.append(Path(override))
    if os.environ.get("SDCC"):
        candidates.append(Path(os.environ["SDCC"]))
    candidates.append(rel(config["toolchain"]["sdcc"]))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    found = shutil.which("sdcc")
    if found:
        return found
    sys.exit(
        "找不到 SDCC。请先构建工具链（计划文档第 5 节），"
        "或通过 --sdcc / 环境变量 SDCC 指定路径。"
    )


def collect_sources(config: dict, app_name: str) -> list[Path]:
    app = config["apps"][app_name]
    names = list(config["library_sources"]) + list(app["sources"])
    if app.get("free_rtos"):
        names += config["kernel_sources"] + config["freertos_sources"]
    sources = [rel(name) for name in names]
    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        sys.exit("源文件不存在：\n  " + "\n  ".join(missing))
    return sources


def generate_interrupt_header(sources: list[Path], build_dir: Path) -> Path:
    entries = {}
    for source in sources:
        text = source.read_text(encoding="utf-8", errors="replace")
        for match in INTERRUPT_RE.finditer(text):
            name, number = match.group(1), int(match.group(2))
            entries[number] = name

    header = build_dir / GENERATED_HEADER
    lines = [
        "/* 由 tools/build.py 生成，请勿手改。 */",
        "#ifndef STCNAV_GENERATED_INTERRUPT_DECLARATIONS_H",
        "#define STCNAV_GENERATED_INTERRUPT_DECLARATIONS_H",
        '#include "STC32Gxx.h"',
        "",
    ]
    for number in sorted(entries):
        lines.append(f"void {entries[number]}(void) __interrupt ({number});")
    lines += ["", "#endif", ""]
    header.write_text("\n".join(lines), encoding="utf-8")
    return header


def object_name(source: Path) -> str:
    try:
        relative = source.relative_to(REPO)
    except ValueError:
        relative = Path(source.name)
    return str(relative).replace(os.sep, "_").replace(".", "_") + ".rel"


def run(command: list[str], *, cwd: Path | None = None) -> None:
    process = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = process.stdout or ""
    filtered = "\n".join(
        line for line in output.splitlines()
        if not any(marker in line for marker in QUIET_WARNINGS)
    ).strip()
    if filtered:
        print(filtered)
    if process.returncode != 0:
        sys.exit(f"命令失败（{process.returncode}）：{' '.join(command)}")


def compile_sources(
    config: dict,
    sdcc: str,
    sources: list[Path],
    build_dir: Path,
    header: Path,
    app_main: Path,
    app_flags: list[str],
) -> list[Path]:
    include_dirs = [rel(path) for path in config["include_dirs"]]
    toolchain_dirs = [rel(path) for path in config["toolchain"]["include_dirs"]]
    per_file = config.get("per_file_cflags", {})
    objects = []

    for source in sources:
        obj = build_dir / object_name(source)
        command = [sdcc] + config["cflags"] + app_flags
        command += [f"-I{path}" for path in toolchain_dirs + include_dirs]
        command += per_file.get(source.name, [])
        command += ["-c"]
        if source.resolve() == app_main.resolve():
            command += ["--include", str(header)]
        command += ["-o", str(obj), str(source)]
        run(command, cwd=build_dir)
        objects.append(obj)

    return objects


def link(
    config: dict,
    sdcc: str,
    objects: list[Path],
    build_dir: Path,
    app_name: str,
) -> tuple[Path, Path, Path]:
    lib_dir = rel(config["toolchain"]["lib_dir"])
    if not lib_dir.is_dir():
        sys.exit(
            f"运行库目录不存在：{lib_dir}\n"
            "请先按计划文档第 5 节构建并安装 SDCC-C251。"
        )

    hex_path = build_dir / f"{app_name}.hex"
    command = [sdcc] + config["ldflags"]
    command += config.get("per_app_ldflags", {}).get(
        app_name, ["-Wl-b GSINIT0=0xfe4000"]
    )
    command += [f"-L{lib_dir}"]
    command += [str(obj) for obj in objects]
    command += config["toolchain"]["libraries"]
    command += ["-o", str(hex_path)]
    run(command, cwd=build_dir)

    map_path = hex_path.with_suffix(".map")
    mem_path = hex_path.with_suffix(".mem")
    return hex_path, map_path, mem_path


def run_qemu(qemu: Path, hex_path: Path, timeout_s: int) -> None:
    command = [
        str(qemu),
        "-M", "stc32g144k246",
        "-bios", str(hex_path),
        "-display", "none",
        "-monitor", "none",
        "-serial", "stdio",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        output, _ = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
    print(output or "")
    if "PASS" not in (output or ""):
        sys.exit(f"QEMU 在 {timeout_s}s 内没有输出 PASS")


def print_memory(mem_path: Path) -> None:
    if not mem_path.is_file():
        return
    text = mem_path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(("ROM/EPROM/FLASH", "EXTERNAL RAM")):
            print(f"[mem] {stripped}")
        elif "initial stack starts at" in stripped:
            print(f"[mem] {stripped}")


def main() -> int:
    parser = argparse.ArgumentParser(description="构建 stcnav 固件")
    parser.add_argument("--app", required=True, help="apps/ 下的应用名")
    parser.add_argument("--sdcc", help="sdcc 可执行文件路径")
    parser.add_argument("--flash", action="store_true", help="构建后烧录")
    parser.add_argument("--qemu", action="store_true", help="构建后用 QEMU 运行")
    parser.add_argument(
        "--qemu-timeout",
        type=int,
        default=15,
        help="QEMU 串口观察超时（秒）",
    )
    parser.add_argument("--clean", action="store_true", help="先清理该应用的构建目录")
    args = parser.parse_args()

    config = load_config()
    if args.app not in config["apps"]:
        sys.exit(f"未知应用：{args.app}（可选：{', '.join(config['apps'])}）")

    build_dir = REPO / "build" / args.app
    if args.clean and build_dir.exists():
        shutil.rmtree(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)

    sources = collect_sources(config, args.app)
    app_main = rel(config["apps"][args.app]["sources"][0])
    header = generate_interrupt_header(sources, build_dir)
    sdcc = find_sdcc(config, args.sdcc)

    app_flags = config.get("per_app_cflags", {}).get(args.app, [])
    print(f"[build] app={args.app} sdcc={sdcc}")
    objects = compile_sources(
        config, sdcc, sources, build_dir, header, app_main, app_flags
    )
    hex_path, map_path, mem_path = link(config, sdcc, objects, build_dir, args.app)
    print_memory(mem_path)

    app_config = config["apps"][args.app]
    check = [
        sys.executable,
        str(REPO / "tools" / "check_layout.py"),
        str(hex_path),
        "--map",
        str(map_path),
        "--mem",
        str(mem_path),
        "--code-start",
        app_config.get("code_start", "0xFE2000"),
        "--code-end",
        app_config.get("code_end", "0xFEFFFF"),
    ]
    run(check)

    if args.flash:
        run([sys.executable, str(REPO / "tools" / "flash.py"), str(hex_path)])

    if args.qemu:
        qemu = REPO / "third_party" / "qemu" / "build" / "qemu-system-mcs251"
        if not qemu.is_file():
            sys.exit(
                "找不到 QEMU。请按计划文档第 10.1 节构建 "
                "third_party/qemu/build/qemu-system-mcs251。"
            )
        run_qemu(qemu, hex_path, args.qemu_timeout)

    print(f"[build] 完成：{hex_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
