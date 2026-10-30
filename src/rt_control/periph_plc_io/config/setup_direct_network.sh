#!/usr/bin/env bash
set -euo pipefail

# 当前工控机的直连规划：
#   LAN1 / eno1   本机 192.168.1.105 -> 数字量模块 192.168.1.12
#   LAN3 / enp4s0 本机 192.168.1.104 -> 模拟量模块 192.168.1.13
# 两条链路使用精确 /32 路由，因此程序不需要 SO_BINDTODEVICE，也不会误走 Wi-Fi。

if [[ ${EUID} -ne 0 ]]; then
  echo "请使用 sudo 运行此脚本" >&2
  exit 1
fi

configure_direct_connection() {
  local connection_name=$1
  local interface_name=$2
  local local_address=$3
  local module_address=$4

  if ! nmcli connection show "${connection_name}" >/dev/null 2>&1; then
    nmcli connection add \
      type ethernet \
      ifname "${interface_name}" \
      con-name "${connection_name}"
  fi

  nmcli connection modify "${connection_name}" \
    connection.interface-name "${interface_name}" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    ipv4.method manual \
    ipv4.addresses "${local_address}/32" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes \
    ipv4.routes "${module_address}/32" \
    ipv6.method disabled

  nmcli connection up "${connection_name}"
}

configure_direct_connection \
  "plc-io-digital" "eno1" "192.168.1.105" "192.168.1.12"
configure_direct_connection \
  "plc-io-analog" "enp4s0" "192.168.1.104" "192.168.1.13"

echo "直连网络已配置。路由结果："
ip route get 192.168.1.12
ip route get 192.168.1.13
