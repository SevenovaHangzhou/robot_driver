#!/usr/bin/env bash
# Local Mink -> Rolling protocol simulator -> MuJoCo, without ROS.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project_dir/script/start.sh" --backend mink-rolling --viewer "$@"
