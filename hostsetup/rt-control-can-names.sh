#!/usr/bin/env bash
set -euo pipefail

readonly RT_CONTROL_SERIAL="004D00675230500720333159"
readonly BMS_SERIAL="003000265230500720333159"
readonly BITRATE="500000"
readonly TXQUEUELEN="128"

wait_seconds=0
configure=false

usage() {
  cat <<'EOF'
usage: rt-control-can-names [--wait SECONDS] [--configure]

Bind the two fixed gs_usb SocketCAN adapters by USB serial:
  can0: rt-control CANopen bus
  can1: BMS CAN bus

--wait waits for both fixed serials to appear.
--configure also forces 500 kbit/s, txqueuelen 128 and UP.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wait)
      [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]] || {
        echo "--wait requires an integer number of seconds" >&2
        exit 2
      }
      wait_seconds="$2"
      shift 2
      ;;
    --configure)
      configure=true
      shift
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

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

command -v ip >/dev/null
command -v udevadm >/dev/null

interface_for_serial() {
  local expected_serial="$1"
  local interface_path
  local interface
  local serial
  local match=""

  for interface_path in /sys/class/net/*; do
    interface="${interface_path##*/}"
    [[ -r "${interface_path}/type" ]] || continue
    [[ "$(<"${interface_path}/type")" == "280" ]] || continue
    serial="$(udevadm info -q property -p "${interface_path}" |
      sed -n 's/^ID_SERIAL_SHORT=//p')"
    [[ "${serial}" == "${expected_serial}" ]] || continue
    if [[ -n "${match}" ]]; then
      echo "multiple CAN interfaces report USB serial ${expected_serial}" >&2
      return 1
    fi
    match="${interface}"
  done

  [[ -n "${match}" ]] || return 1
  printf '%s\n' "${match}"
}

wait_for_serial() {
  local label="$1"
  local expected_serial="$2"
  local deadline=$((SECONDS + wait_seconds))
  local interface=""

  while true; do
    if interface="$(interface_for_serial "${expected_serial}")"; then
      printf '%s\n' "${interface}"
      return
    fi
    if (( wait_seconds == 0 || SECONDS >= deadline )); then
      echo "missing ${label} CAN adapter: expected USB serial ${expected_serial}" >&2
      return 1
    fi
    udevadm settle --timeout=1 >/dev/null 2>&1 || true
    sleep 0.5
  done
}

verify_reserved_name_not_unknown() {
  local interface="$1"
  local actual_serial

  [[ -e "/sys/class/net/${interface}" ]] || return
  actual_serial="$(udevadm info -q property -p "/sys/class/net/${interface}" |
    sed -n 's/^ID_SERIAL_SHORT=//p')"
  case "${actual_serial}" in
    "${RT_CONTROL_SERIAL}"|"${BMS_SERIAL}")
      return 0
      ;;
  esac
  echo "${interface} already exists with unapproved USB serial ${actual_serial:-unknown}" >&2
  return 1
}

configure_can_interface() {
  local interface="$1"
  ip link set dev "${interface}" down
  ip link set dev "${interface}" type can bitrate "${BITRATE}"
  ip link set dev "${interface}" txqueuelen "${TXQUEUELEN}"
  ip link set dev "${interface}" up
}

verify_can_interface() {
  local interface="$1"
  local expected_serial="$2"
  local label="$3"
  local actual_serial
  local output

  [[ -e "/sys/class/net/${interface}" ]] ||
    { echo "${label} CAN adapter did not bind to ${interface}" >&2; return 1; }
  actual_serial="$(udevadm info -q property -p "/sys/class/net/${interface}" |
    sed -n 's/^ID_SERIAL_SHORT=//p')"
  [[ "${actual_serial}" == "${expected_serial}" ]] ||
    { echo "${interface} serial mismatch for ${label}: got ${actual_serial:-unknown}, expected ${expected_serial}" >&2; return 1; }
  output="$(ip -details -statistics link show "${interface}")"
  grep -Fq 'state UP' <<< "${output}" ||
    { echo "${interface} is not UP after CAN preflight" >&2; return 1; }
  grep -Fq 'can state ERROR-ACTIVE' <<< "${output}" ||
    { echo "${interface} is not ERROR-ACTIVE after CAN preflight" >&2; return 1; }
  grep -Fq "bitrate ${BITRATE}" <<< "${output}" ||
    { echo "${interface} is not ${BITRATE} bit/s after CAN preflight" >&2; return 1; }
  grep -Fq "qlen ${TXQUEUELEN}" <<< "${output}" ||
    { echo "${interface} txqueuelen is not ${TXQUEUELEN} after CAN preflight" >&2; return 1; }
}

verify_reserved_name_not_unknown can0
verify_reserved_name_not_unknown can1

rt_interface="$(wait_for_serial "rt-control/can0" "${RT_CONTROL_SERIAL}")"
bms_interface="$(wait_for_serial "BMS/can1" "${BMS_SERIAL}")"
if [[ "${rt_interface}" == "${bms_interface}" ]]; then
  echo "CAN serials unexpectedly resolved to the same interface" >&2
  exit 1
fi

ip link set dev "${rt_interface}" down
ip link set dev "${bms_interface}" down
if [[ "${rt_interface}" != "rtcan_tmp" ]]; then
  ip link set dev "${rt_interface}" name rtcan_tmp
fi
if [[ "${bms_interface}" != "bmscan_tmp" ]]; then
  ip link set dev "${bms_interface}" name bmscan_tmp
fi
ip link set dev rtcan_tmp name can0
ip link set dev bmscan_tmp name can1

if ${configure}; then
  configure_can_interface can0
  configure_can_interface can1
else
  ip link set dev can0 up
  ip link set dev can1 up
fi

verify_can_interface can0 "${RT_CONTROL_SERIAL}" "rt-control/can0"
verify_can_interface can1 "${BMS_SERIAL}" "BMS/can1"
