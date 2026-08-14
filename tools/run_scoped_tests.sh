#!/usr/bin/env bash
# ELECTRI-81：按改动范围裁剪本地测试的唯一入口（内环用；CI 全量兜底不变）。
set -euo pipefail
cd "$(dirname "$0")/.."
exec python3 tools/scoped_tests.py --run "$@"
