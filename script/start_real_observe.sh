#!/usr/bin/env bash
# Read feedback only. Does not enable, switch controllers or publish motion.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$project_dir"
exec "$project_dir/.venv/bin/python" -B -m gripper_jog.real_observe --domain-id 78 "$@"
