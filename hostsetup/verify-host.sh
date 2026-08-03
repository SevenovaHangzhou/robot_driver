#!/usr/bin/env bash
set -euo pipefail

readonly ethercat_mac="8c:59:3c:14:ff:d3"
readonly rt_control_can_serial="004D00675230500720333159"
readonly bms_can_serial="003000265230500720333159"
readonly rt_control_cpu="14"
readonly housekeeping_cpus="0,2,4,6,8,10,12,16-27"
readonly ec_master_run_on_cpu_line="options ec_master run_on_cpu=14"

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

contains_cpu14() {
  local list="$1"
  local part
  local first
  local last
  local old_ifs="${IFS}"
  IFS=,
  for part in ${list}; do
    case "${part}" in
      *-*)
        first="${part%-*}"
        last="${part#*-}"
        if (( 14 >= first && 14 <= last )); then
          IFS="${old_ifs}"
          return 0
        fi
        ;;
      14)
        IFS="${old_ifs}"
        return 0
        ;;
    esac
  done
  IFS="${old_ifs}"
  return 1
}

[[ "$(uname -r)" == *-realtime ]] || fail "running kernel is not realtime"
[[ "$(< /sys/kernel/realtime)" == "1" ]] || fail "PREEMPT_RT is not active"
[[ "$(< /sys/devices/system/cpu/isolated)" == "14" ]] || fail "CPU 14 is not isolated"
[[ "$(< /sys/devices/system/cpu/nohz_full)" == "14" ]] || fail "CPU 14 is not full-nohz"
grep -Eq '(^|,)15($|,)' /sys/devices/system/cpu/offline || fail "CPU 15 is not offline"
for argument in \
  nosmt \
  isolcpus=domain,managed_irq,14 \
  nohz_full=14 \
  rcu_nocbs=14 \
  irqaffinity=${housekeeping_cpus}; do
  grep -qw "${argument}" /proc/cmdline || fail "missing kernel argument ${argument}"
done
[[ -r /etc/modprobe.d/ec_master.conf ]] || fail "missing /etc/modprobe.d/ec_master.conf"
grep -Fxq "${ec_master_run_on_cpu_line}" /etc/modprobe.d/ec_master.conf ||
  fail "ec_master is not configured with run_on_cpu=${rt_control_cpu}"

for irq_directory in /proc/irq/[0-9]*; do
  affinity="$(cat -- "${irq_directory}/effective_affinity_list" 2>/dev/null || true)"
  if contains_cpu14 "${affinity}"; then
    fail "IRQ ${irq_directory##*/} targets CPU 14 via ${affinity}"
  fi
done

systemctl is-active --quiet apparmor.service docker.service containerd.service ethercat.service
systemctl is-enabled --quiet rt-control-can-names.service &&
  fail "rt-control-can-names.service must not be enabled at boot"
systemctl is-enabled --quiet can0.service &&
  fail "can0.service must not be enabled at boot"
systemctl is-enabled --quiet can1.service &&
  fail "can1.service must not be enabled at boot"
[[ "$(systemctl --failed --no-legend | wc -l)" -eq 0 ]] || fail "systemd has failed units"

[[ "$(ethercat version)" == "IgH EtherCAT master 1.6.10 unknown" ]] ||
  fail "unexpected IgH version"
master_output="$(ethercat master)"
grep -Fq "Main: ${ethercat_mac} (attached)" <<< "${master_output}" || fail "master MAC mismatch"
grep -Fq 'Link: UP' <<< "${master_output}" || fail "EtherCAT link is down"
grep -Fq 'Slaves: 16' <<< "${master_output}" || fail "EtherCAT does not report 16 slaves"
grep -Fq 'Lost frames: 0' <<< "${master_output}" || fail "EtherCAT has lost frames"
[[ "$(ethercat slaves | wc -l)" -eq 16 ]] || fail "EtherCAT scan does not contain 16 positions"

can_serial="$(udevadm info -q property -p /sys/class/net/can0 |
  sed -n 's/^ID_SERIAL_SHORT=//p')"
[[ "${can_serial}" == "${rt_control_can_serial}" ]] || fail "can0 USB serial mismatch"
can_output="$(ip -details -statistics link show can0)"
grep -Fq 'state UP' <<< "${can_output}" || fail "can0 is down"
grep -Fq 'can state ERROR-ACTIVE' <<< "${can_output}" || fail "can0 is not ERROR-ACTIVE"
grep -Fq 'bitrate 500000' <<< "${can_output}" || fail "can0 is not 500 kbit/s"
bms_serial="$(udevadm info -q property -p /sys/class/net/can1 |
  sed -n 's/^ID_SERIAL_SHORT=//p')"
[[ "${bms_serial}" == "${bms_can_serial}" ]] || fail "can1 USB serial mismatch"
bms_output="$(ip -details -statistics link show can1)"
grep -Fq 'state UP' <<< "${bms_output}" || fail "can1 is down"
grep -Fq 'can state ERROR-ACTIVE' <<< "${bms_output}" || fail "can1 is not ERROR-ACTIVE"
grep -Fq 'bitrate 500000' <<< "${bms_output}" || fail "can1 is not 500 kbit/s"
heartbeat_log="$(mktemp)"
cleanup() {
  rm -f -- "${heartbeat_log}"
}
trap cleanup EXIT
timeout 6 candump -L can0 > "${heartbeat_log}" || [[ $? -eq 124 ]]
for cob_id in 702 703; do
  grep -Fq "can0 ${cob_id}#" "${heartbeat_log}" || fail "missing heartbeat 0x${cob_id}"
done

[[ "$(dpkg-query -W -f='${Version}' docker-ce)" == \
  "5:29.6.2-1~ubuntu.22.04~jammy" ]] || fail "Docker CE version drift"
[[ "$(dpkg-query -W -f='${Version}' containerd.io)" == \
  "2.2.6-1~ubuntu.22.04~jammy" ]] || fail "containerd version drift"

if journalctl -b -k --no-pager | grep -Eq 'PCIe Bus Error|NVRM: Xid'; then
  fail "current boot contains PCIe Bus Error or NVIDIA Xid"
fi
nvidia-smi --query-gpu=name,driver_version,pci.bus_id --format=csv,noheader

printf '%s\n' \
  "PASS: realtime CPU14 isolation" \
  "PASS: IgH 1.6.10, master ${ethercat_mac}, 16 slaves, zero lost frames" \
  "PASS: can0 500 kbit/s on serial ${rt_control_can_serial}, heartbeats 0x702/703" \
  "PASS: can1 500 kbit/s on serial ${bms_can_serial}" \
  "PASS: Docker/containerd frozen versions, healthy systemd and GPU boot log"
