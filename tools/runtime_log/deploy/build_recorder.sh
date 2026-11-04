#!/usr/bin/env bash
# Build the optional runtime_log_recorder plugin into an existing RT-Control workspace install
# (merged, symlink install) without touching any other package. Records the source commit.
# Usage: deploy/build_recorder.sh <workspace_root>   e.g. ~/rt-control-electri127
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(git -C "$here" rev-parse --show-toplevel)"
ws="$(cd "$1" && pwd)"
[[ -f "$ws/install/.colcon_install_layout" && "$(cat "$ws/install/.colcon_install_layout")" == merged ]] \
  || { echo "$ws/install is not a merged colcon install" >&2; exit 1; }
commit="$(git -C "$here" rev-parse HEAD)"
dirty="$(git -C "$here" status --porcelain -- src/rt_control/runtime_log_recorder src/rt_control/infra_shm_ring | wc -l)"
set +u; source /opt/ros/humble/setup.bash; source "$ws/install/setup.bash"; set -u
nice -n 10 colcon --log-base "$ws/log" build \
  --base-paths "$repo/src/rt_control/runtime_log_recorder" "$repo/src/rt_control/infra_shm_ring" \
  --build-base "$ws/build" \
  --install-base "$ws/install" \
  --merge-install --symlink-install \
  --packages-up-to runtime_log_recorder \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
share="$ws/install/share/runtime_log_recorder"
printf 'commit=%s\ndirty_files=%s\nbuilt=%s\n' "$commit" "$dirty" "$(date -Is)" > "$share/RUNTIME_LOG_BUILD"
echo "built runtime_log_recorder at ${commit:0:12} (dirty source files: $dirty) -> $share"
