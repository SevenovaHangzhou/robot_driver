#!/usr/bin/env bash
set -euo pipefail

readonly BMS_SERIAL="003000265230500720333159"
readonly BITRATE="500000"
readonly TXQUEUELEN="128"

wait_seconds=0
configure=false

usage() {
  cat <<'EOF'
usage: rt-control-can-names [--wait SECONDS] [--configure]

Bind the fixed BMS gs_usb SocketCAN adapter by USB serial:
  can1: BMS CAN bus

--wait waits for the BMS serial to appear.
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
  [[ "${actual_serial}" == "${BMS_SERIAL}" ]] && return 0
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
  local deadline=$((SECONDS + 5))
  local last_error=""
  local output

  while true; do
    if [[ ! -e "/sys/class/net/${interface}" ]]; then
      last_error="${label} CAN adapter did not bind to ${interface}"
    else
      actual_serial="$(udevadm info -q property -p "/sys/class/net/${interface}" |
        sed -n 's/^ID_SERIAL_SHORT=//p')"
      [[ "${actual_serial}" == "${expected_serial}" ]] ||
        { echo "${interface} serial mismatch for ${label}: got ${actual_serial:-unknown}, expected ${expected_serial}" >&2; return 1; }
      output="$(ip -details -statistics link show "${interface}")"
      if ! grep -Fq 'state UP' <<< "${output}"; then
        last_error="${interface} is not UP after CAN preflight"
      elif ! grep -Fq 'can state ERROR-ACTIVE' <<< "${output}"; then
        last_error="${interface} is not ERROR-ACTIVE after CAN preflight"
      elif ! grep -Fq "bitrate ${BITRATE}" <<< "${output}"; then
        last_error="${interface} is not ${BITRATE} bit/s after CAN preflight"
      elif ! grep -Fq "qlen ${TXQUEUELEN}" <<< "${output}"; then
        last_error="${interface} txqueuelen is not ${TXQUEUELEN} after CAN preflight"
      else
        return 0
      fi
    fi
    if (( SECONDS >= deadline )); then
      echo "${last_error}" >&2
      return 1
    fi
    sleep 0.2
  done
}

verify_reserved_name_not_unknown can1

bms_interface="$(wait_for_serial "BMS/can1" "${BMS_SERIAL}")"

ip link set dev "${bms_interface}" down
if [[ "${bms_interface}" != "bmscan_tmp" ]]; then
  ip link set dev "${bms_interface}" name bmscan_tmp
fi
ip link set dev bmscan_tmp name can1

if ${configure}; then
  configure_can_interface can1
else
  ip link set dev can1 up
fi

verify_can_interface can1 "${BMS_SERIAL}" "BMS/can1"
