#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
start_bus=false

if [[ "${1:-}" == "--start" ]]; then
  start_bus=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--start]" >&2
  exit 2
fi
if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

if [[ -e /etc/systemd/system/can0.service ]]; then
  echo "legacy can0.service is still installed; retire it under separate host authorization" >&2
  exit 1
fi

command -v ip >/dev/null
command -v udevadm >/dev/null
bash -n "${script_dir}/rt-control-can-names.sh"

install -o root -g root -m 0755 \
  "${script_dir}/rt-control-can-names.sh" /usr/local/sbin/rt-control-can-names
install -o root -g root -m 0644 \
  "${script_dir}/rt-control-can-names.service" \
  /etc/systemd/system/rt-control-can-names.service
install -o root -g root -m 0644 \
  "${script_dir}/can1.service" /etc/systemd/system/can1.service
systemctl daemon-reload
systemctl disable rt-control-can-names.service can1.service
systemd-analyze verify \
  /etc/systemd/system/rt-control-can-names.service \
  /etc/systemd/system/can1.service

if ${start_bus}; then
  /usr/local/sbin/rt-control-can-names --wait 30 --configure
  ip -details -statistics link show can1
else
  echo "BMS CAN unit installed but not enabled at boot; V3 encoder CAN remains unconfigured"
fi
