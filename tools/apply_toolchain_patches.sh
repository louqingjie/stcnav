#!/usr/bin/env bash
# 应用 tools/patches/ 下的工具链补丁（幂等）。
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
submodule="$repo_root/third_party/sdcc-c251"

if [ ! -d "$submodule/.git" ] && [ ! -f "$submodule/.git" ]; then
    echo "缺少子模块：$submodule" >&2
    exit 1
fi

shopt -s nullglob
for patch in "$repo_root"/tools/patches/*.patch; do
    if git -C "$submodule" apply --check "$patch" 2>/dev/null; then
        git -C "$submodule" apply "$patch"
        echo "已应用：$(basename "$patch")"
    elif git -C "$submodule" apply --reverse --check "$patch" 2>/dev/null; then
        echo "已存在：$(basename "$patch")"
    else
        echo "补丁无法应用：$patch" >&2
        exit 1
    fi
done
