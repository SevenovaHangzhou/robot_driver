#!/usr/bin/env bash
# Requires the primary console's shared authority and its explicit jog mode.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$project_dir"
python_bin="$project_dir/.venv/bin/python"
if [[ ! -x "$python_bin" ]]; then python_bin=python3; fi
exec "$python_bin" -B -m gripper_jog.real_server "$@"
