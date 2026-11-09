#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="$project_dir/.venv/bin/python"

if [[ ! -x "$python_bin" ]]; then
    printf '未找到项目 Python 环境：%s\n请先在项目目录创建 .venv 并安装项目依赖。\n' "$python_bin" >&2
    exit 1
fi

cd -- "$project_dir"
exec "$python_bin" -B -m gripper_jog.server "$@"
