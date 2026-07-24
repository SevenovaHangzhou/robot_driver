#!/usr/bin/env bash
set -euo pipefail

readonly RT_CONTROL_SERIAL="004D00675230500720333159"
readonly BMS_SERIAL="003000265230500720333159"

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

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

  if [[ -z "${match}" ]]; then
    echo "no CAN interface reports USB serial ${expected_serial}" >&2
    return 1
  fi
  printf '%s\n' "${match}"
}

rt_interface="$(interface_for_serial "${RT_CONTROL_SERIAL}")"
bms_interface="$(interface_for_serial "${BMS_SERIAL}")"
if [[ "${rt_interface}" == "${bms_interface}" ]]; then
  echo "CAN serials unexpectedly resolved to the same interface" >&2
  exit 1
fi

rt_was_up=false
bms_was_up=false
ip -o link show dev "${rt_interface}" | grep -q '<[^>]*UP[,>]' && rt_was_up=true
ip -o link show dev "${bms_interface}" | grep -q '<[^>]*UP[,>]' && bms_was_up=true

ip link set dev "${rt_interface}" down
ip link set dev "${bms_interface}" down
ip link set dev "${rt_interface}" name rtcan_tmp
ip link set dev "${bms_interface}" name bmscan_tmp
ip link set dev rtcan_tmp name can0
ip link set dev bmscan_tmp name can1

${rt_was_up} && ip link set dev can0 up
${bms_was_up} && ip link set dev can1 up

test "$(udevadm info -q property -p /sys/class/net/can0 |
  sed -n 's/^ID_SERIAL_SHORT=//p')" = "${RT_CONTROL_SERIAL}"
test "$(udevadm info -q property -p /sys/class/net/can1 |
  sed -n 's/^ID_SERIAL_SHORT=//p')" = "${BMS_SERIAL}"
