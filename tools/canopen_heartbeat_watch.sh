#!/usr/bin/env bash
set -euo pipefail

interface="can0"
duration="60"
threshold="5.0"

usage()
{
  cat <<'EOF'
Usage: tools/canopen_heartbeat_watch.sh [--interface can0] [--duration seconds] [--threshold seconds]

Read-only CANopen heartbeat gap observer for rt-control commissioning.
Tracks 0x764 master heartbeat and 0x702/0x703 track-drive heartbeats.
EOF
}

while (($# > 0)); do
  case "$1" in
    --interface)
      interface="$2"
      shift 2
      ;;
    --duration)
      duration="$2"
      shift 2
      ;;
    --threshold)
      threshold="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

command -v candump >/dev/null 2>&1 || {
  echo "missing candump from can-utils" >&2
  exit 1
}

capture="$(mktemp)"
cleanup()
{
  rm -f -- "${capture}"
}
trap cleanup EXIT

ip -details -statistics link show "${interface}" | sed -n '1,16p'
set +e
timeout "${duration}" candump -ta "${interface},702:7FF,703:7FF,764:7FF" > "${capture}"
status=$?
set -e
if [[ "${status}" -ne 0 && "${status}" -ne 124 ]]; then
  echo "candump failed with status ${status}" >&2
  exit "${status}"
fi

awk -v threshold="${threshold}" '
function clean_id(raw) {
  raw = toupper(raw)
  sub(/^0+/, "", raw)
  return raw
}
BEGIN {
  ids["702"] = "left_track 0x702"
  ids["703"] = "right_track 0x703"
  ids["764"] = "master 0x764"
}
{
  ts = $1
  gsub(/[()]/, "", ts)
  id = clean_id($3)
  if (!(id in ids)) {
    next
  }
  count[id] += 1
  if (last[id] != "") {
    gap = ts - last[id]
    if (gap > max_gap[id]) {
      max_gap[id] = gap
    }
    if (gap > threshold) {
      over_threshold[id] += 1
      printf("WARN: %s heartbeat gap %.6fs exceeded %.6fs at %.6f\n",
             ids[id], gap, threshold, ts) > "/dev/stderr"
    }
  }
  last[id] = ts
}
END {
  for (id in ids) {
    printf("%s max_gap=%.6fs count=%d over_threshold=%d\n",
           ids[id], max_gap[id], count[id], over_threshold[id])
  }
}
' "${capture}"

ip -details -statistics link show "${interface}" | sed -n '1,22p'
