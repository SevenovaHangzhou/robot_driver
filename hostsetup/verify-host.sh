#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly script_dir
repository_root="$(cd -- "${script_dir}/.." && pwd)"
readonly repository_root
# shellcheck disable=SC1091
source "${repository_root}/versions.env"
# shellcheck source=hostsetup/legacy-units.sh
source "${script_dir}/legacy-units.sh"

readonly ethercat_mac="8c:59:3c:15:01:f8"
readonly bms_can_serial="003000265230500720333159"
readonly rt_control_cpu="14"
readonly housekeeping_cpus="0,2,4,6,8,10,12,16-27"
readonly ec_master_run_on_cpu_line="options ec_master run_on_cpu=14"
readonly igh_patch="${repository_root}/patches/igh/0001-preserve-verified-pdo-config.patch"
readonly igh_dc_patch="${repository_root}/patches/igh/0002-dc-offset-use-sent-application-time.patch"
readonly igh_preop_patch="${repository_root}/patches/igh/0003-preop-only-coe.patch"
readonly igh_metadata="/usr/local/share/rt-control/dependency-versions.env"

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -r "${igh_patch}" ]] || fail "missing IgH PDO-preservation patch ${igh_patch}"
[[ -r "${igh_dc_patch}" ]] || fail "missing IgH DC offset patch ${igh_dc_patch}"
[[ -r "${igh_metadata}" ]] || fail "missing installed IgH dependency identity"
grep -Fxq "IGH_VERSION=${IGH_VERSION}" "${igh_metadata}" ||
  fail "installed IgH version does not match versions.env"
grep -Fxq "IGH_COMMIT=${IGH_COMMIT}" "${igh_metadata}" ||
  fail "installed IgH commit does not match versions.env"
grep -Fxq "IGH_PATCHES=0001-preserve-verified-pdo-config.patch 0002-dc-offset-use-sent-application-time.patch 0003-preop-only-coe.patch" "${igh_metadata}" ||
  fail "installed IgH master was not built with the approved patch set"
grep -Fxq 'IGH_HRTIMER=1' "${igh_metadata}" ||
  fail "IgH requires --enable-hrtimer before FIFO scheduling"
[[ -r "${igh_preop_patch}" ]] || fail "missing IgH PREOP CoE policy patch"

# "0-7,9,16-27" -> "0 1 2 3 4 5 6 7 9 16 ... 27"
expand_cpu_list() {
  local part
  local cpu
  local old_ifs="${IFS}"
  IFS=,
  for part in $1; do
    if [[ "${part}" == *-* ]]; then
      for (( cpu = ${part%-*}; cpu <= ${part#*-}; cpu++ )); do
        printf '%s ' "${cpu}"
      done
    elif [[ -n "${part}" ]]; then
      printf '%s ' "${part}"
    fi
  done
  IFS="${old_ifs}"
}

cpu_in_list() {
  [[ " $(expand_cpu_list "$2") " == *" $1 "* ]]
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
  rcu_nocbs=14; do
  grep -qw "${argument}" /proc/cmdline || fail "missing kernel argument ${argument}"
done
# irqaffinity is judged by its effect on the online CPUs, not by its text: interrupts may land
# only on housekeeping CPUs, never on the realtime CPU, and on at least one CPU.
irq_cpus="$(sed -n 's/.*\birqaffinity=\([0-9,-]*\).*/\1/p' /proc/cmdline)"
[[ -n "${irq_cpus}" ]] || fail "missing kernel argument irqaffinity"
online_cpus="$(< /sys/devices/system/cpu/online)"
effective_irq_cpus=""
for cpu in $(expand_cpu_list "${irq_cpus}"); do
  cpu_in_list "${cpu}" "${online_cpus}" && effective_irq_cpus+="${cpu} "
done
[[ -n "${effective_irq_cpus}" ]] || fail "irqaffinity=${irq_cpus} leaves no online CPU for interrupts"
for cpu in ${effective_irq_cpus}; do
  [[ "${cpu}" != "${rt_control_cpu}" ]] ||
    fail "irqaffinity=${irq_cpus} allows interrupts on realtime CPU ${rt_control_cpu}"
  cpu_in_list "${cpu}" "${housekeeping_cpus}" ||
    fail "irqaffinity=${irq_cpus} allows interrupts on CPU ${cpu} outside ${housekeeping_cpus}"
done

[[ -r /etc/modprobe.d/ec_master.conf ]] || fail "missing /etc/modprobe.d/ec_master.conf"
grep -Fxq "${ec_master_run_on_cpu_line}" /etc/modprobe.d/ec_master.conf ||
  fail "ec_master is not configured with run_on_cpu=${rt_control_cpu}"

for irq_directory in /proc/irq/[0-9]*; do
  affinity="$(cat -- "${irq_directory}/effective_affinity_list" 2>/dev/null || true)"
  if [[ -n "${affinity}" ]] && cpu_in_list "${rt_control_cpu}" "${affinity}"; then
    fail "IRQ ${irq_directory##*/} targets CPU 14 via ${affinity}"
  fi
done

systemctl is-active --quiet apparmor.service docker.service containerd.service ethercat.service
systemctl is-enabled --quiet rt-control-can-names.service &&
  fail "rt-control-can-names.service must not be enabled at boot"
legacy_can0="$(legacy_unit_problem can0.service)"
[[ -z "${legacy_can0}" ]] || fail "legacy ${legacy_can0}; disable it before V3 host acceptance"
systemctl is-enabled --quiet can1.service &&
  fail "can1.service must not be enabled at boot"
[[ "$(systemctl --failed --no-legend | wc -l)" -eq 0 ]] || fail "systemd has failed units"

[[ "$(ethercat version)" == "IgH EtherCAT master 1.6.10 unknown" ]] ||
  fail "unexpected IgH version"
master_output="$(ethercat master)"
grep -Fq "Main: ${ethercat_mac} (attached)" <<< "${master_output}" || fail "master MAC mismatch"
grep -Fq 'Link: UP' <<< "${master_output}" || fail "EtherCAT link is down"
grep -Fq 'Slaves: 18' <<< "${master_output}" || fail "EtherCAT does not report 18 slaves"
grep -Fq 'Lost frames: 0' <<< "${master_output}" || fail "EtherCAT has lost frames"
[[ "$(ethercat slaves | wc -l)" -eq 18 ]] || fail "EtherCAT scan does not contain 18 positions"

bms_serial="$(udevadm info -q property -p /sys/class/net/can1 |
  sed -n 's/^ID_SERIAL_SHORT=//p')"
[[ "${bms_serial}" == "${bms_can_serial}" ]] || fail "can1 USB serial mismatch"
bms_output="$(ip -details -statistics link show can1)"
grep -Fq 'state UP' <<< "${bms_output}" || fail "can1 is down"
grep -Fq 'can state ERROR-ACTIVE' <<< "${bms_output}" || fail "can1 is not ERROR-ACTIVE"
grep -Fq 'bitrate 250000' <<< "${bms_output}" || fail "can1 is not 250 kbit/s"
[[ "$(dpkg-query -W -f='${Version}' docker-ce)" == \
  "5:29.6.2-1~ubuntu.22.04~jammy" ]] || fail "Docker CE version drift"
[[ "$(dpkg-query -W -f='${Version}' containerd.io)" == \
  "2.2.6-1~ubuntu.22.04~jammy" ]] || fail "containerd version drift"

if journalctl -b -k --no-pager | grep -Eq 'PCIe Bus Error|NVRM: Xid'; then
  fail "current boot contains PCIe Bus Error or NVIDIA Xid"
fi
nvidia-smi --query-gpu=name,driver_version,pci.bus_id --format=csv,noheader

printf '%s\n' \
  "PASS: realtime CPU14 isolation; interrupts on online CPUs ${effective_irq_cpus% }" \
  "PASS: IgH 1.6.10 with fixed-PDO verification, master ${ethercat_mac}, 18 slaves, zero lost frames" \
  "PASS: can1 250 kbit/s on serial ${bms_can_serial}" \
  "PASS: Docker/containerd frozen versions, healthy systemd and GPU boot log"
