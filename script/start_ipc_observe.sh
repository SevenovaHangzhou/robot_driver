#!/usr/bin/env bash
# Start only the feedback webpage on the IPC; never starts the hardware stack.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$project_dir"
exec python3 -B -m gripper_jog.real_observe --local-runtime --domain-id 78 "$@"
