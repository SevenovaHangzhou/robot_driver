#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'EOF'
Usage: canopen_sdo_archive.sh OUTPUT_DIRECTORY [CAN_INTERFACE]

Read-only commissioning archive for the three production CANopen nodes.
The ros2_canopen driver services must already be running. The script captures
CAN traffic while issuing SDO uploads for the frozen PDO communication and
mapping ranges. It never calls an SDO-write service.

Arguments:
  OUTPUT_DIRECTORY  Must not already exist.
  CAN_INTERFACE     SocketCAN interface (default: can0).
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage >&2
  exit 2
fi

readonly output_directory="$1"
readonly can_interface="${2:-can0}"

if [[ -e "${output_directory}" ]]; then
  echo "Refusing to overwrite existing path: ${output_directory}" >&2
  exit 2
fi

for command_name in candump ros2 date; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Required command not found: ${command_name}" >&2
    exit 2
  fi
done

readonly service_type="canopen_interfaces/srv/CORead"
readonly -a node_names=(updown left_track right_track)

for node_name in "${node_names[@]}"; do
  if ! ros2 service list | grep -Fx "/${node_name}/sdo_read" >/dev/null; then
    echo "Required read service is unavailable: /${node_name}/sdo_read" >&2
    exit 2
  fi
done

mkdir -p "${output_directory}"
readonly sdo_log="${output_directory}/sdo-read.log"
readonly can_log="${output_directory}/candump.log"
readonly metadata_file="${output_directory}/metadata.txt"

{
  echo "captured_at=$(date --iso-8601=seconds)"
  echo "can_interface=${can_interface}"
  echo "service_type=${service_type}"
  echo "nodes=updown:1,left_track:2,right_track:3"
  echo "policy=SDO uploads only; no SDO downloads; no PDO remapping"
} >"${metadata_file}"

candump -L "${can_interface}" >"${can_log}" 2>&1 &
candump_pid=$!

stop_candump() {
  if kill -0 "${candump_pid}" 2>/dev/null; then
    kill -INT "${candump_pid}" 2>/dev/null || true
    wait "${candump_pid}" 2>/dev/null || true
  fi
}
trap stop_candump EXIT INT TERM

read_object_range() {
  local node_name="$1"
  local first_index="$2"
  local last_index="$3"
  local last_subindex="$4"
  local index
  local subindex

  for ((index = first_index; index <= last_index; ++index)); do
    for ((subindex = 0; subindex <= last_subindex; ++subindex)); do
      printf '\nnode=%s index=0x%04X subindex=%u\n' \
        "${node_name}" "${index}" "${subindex}" | tee -a "${sdo_log}"
      ros2 service call "/${node_name}/sdo_read" "${service_type}" \
        "{index: ${index}, subindex: ${subindex}}" 2>&1 | tee -a "${sdo_log}"
      printf 'ros2_exit=%u\n' "${PIPESTATUS[0]}" | tee -a "${sdo_log}"
    done
  done
}

for node_name in "${node_names[@]}"; do
  read_object_range "${node_name}" $((0x1400)) $((0x1403)) 3
  read_object_range "${node_name}" $((0x1600)) $((0x1603)) 8
  read_object_range "${node_name}" $((0x1800)) $((0x1802)) 6
  read_object_range "${node_name}" $((0x1A00)) $((0x1A02)) 8
done

stop_candump
trap - EXIT INT TERM

echo "Read-only CANopen archive written to ${output_directory}"
