#!/usr/bin/env bash
set -euo pipefail

# alfa-two temporary hardware quarantine. The endpoint functions can continue
# raising AER after their sysfs removal, so quarantine their dedicated upstream
# root port after validating the complete, machine-specific topology.
readonly ROOT_PORT_BDF="0000:00:01.0"
readonly ROOT_PORT_VENDOR="0x8086"
readonly ROOT_PORT_DEVICE="0xa70d"

verify_location() {
  local expected_bdf="$1"
  local expected_device="$2"
  local path
  local actual_bdf

  for path in /sys/bus/pci/devices/*; do
    [[ -r "${path}/vendor" && -r "${path}/device" ]] || continue
    if [[ "$(<"${path}/vendor")" == "0x10de" ]] &&
       [[ "$(<"${path}/device")" == "${expected_device}" ]]; then
      actual_bdf="${path##*/}"
      if [[ "${actual_bdf}" != "${expected_bdf}" ]]; then
        echo "refusing quarantine: NVIDIA ${expected_device} moved to ${actual_bdf}" >&2
        return 1
      fi
    fi
  done
}

verify_function_if_present() {
  local bdf="$1"
  local expected_device="$2"
  local path="/sys/bus/pci/devices/${bdf}"

  if [[ ! -d "${path}" ]]; then
    return 0
  fi
  if [[ "$(<"${path}/vendor")" != "0x10de" ]] ||
     [[ "$(<"${path}/device")" != "${expected_device}" ]]; then
    echo "refusing quarantine: ${bdf} is not NVIDIA ${expected_device}" >&2
    return 1
  fi
}

verify_root_port() {
  local path="/sys/bus/pci/devices/${ROOT_PORT_BDF}"

  if [[ ! -d "${path}" ]]; then
    return 0
  fi
  if [[ "$(<"${path}/vendor")" != "${ROOT_PORT_VENDOR}" ]] ||
     [[ "$(<"${path}/device")" != "${ROOT_PORT_DEVICE}" ]]; then
    echo "refusing quarantine: ${ROOT_PORT_BDF} identity changed" >&2
    return 1
  fi

  local child
  for child in /sys/bus/pci/devices/0000:01:*; do
    [[ -e "${child}" ]] || continue
    case "${child##*/}" in
      0000:01:00.0|0000:01:00.1) ;;
      *)
        echo "refusing quarantine: unexpected device ${child##*/} below ${ROOT_PORT_BDF}" >&2
        return 1
        ;;
    esac
  done
}

remove_root_port() {
  local path="/sys/bus/pci/devices/${ROOT_PORT_BDF}"

  if [[ ! -d "${path}" ]]; then
    return 0
  fi

  echo 1 > "${path}/remove"
}

verify_location 0000:01:00.1 0x2291
verify_location 0000:01:00.0 0x25b8
verify_function_if_present 0000:01:00.1 0x2291
verify_function_if_present 0000:01:00.0 0x25b8
verify_root_port
remove_root_port
