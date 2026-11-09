#!/usr/bin/env bash
# Shared authority from the primary console. Cartesian motion stays disabled.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project_dir/script/start_real_observe.sh" --gateway "${JOG_PRIMARY_CONSOLE:-http://192.168.100.40:18121}" "$@"
