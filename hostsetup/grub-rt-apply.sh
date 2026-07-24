#!/usr/bin/env bash
set -euo pipefail

readonly drop_in="/etc/default/grub.d/99-rt-control.cfg"

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi
if [[ "$(uname -r)" != *-realtime ]]; then
  echo "the running kernel is not PREEMPT_RT" >&2
  exit 1
fi
topology_verified=false
if [[ -r /sys/devices/system/cpu/cpu14/topology/thread_siblings_list ]] &&
   [[ "$(< /sys/devices/system/cpu/cpu14/topology/thread_siblings_list)" == "14-15" ]]; then
  # First application, before nosmt removes the secondary hardware threads.
  topology_verified=true
elif grep -qw nosmt /proc/cmdline &&
     [[ "$(< /sys/devices/system/cpu/isolated)" == "14" ]] &&
     [[ "$(< /sys/devices/system/cpu/nohz_full)" == "14" ]] &&
     grep -Eq '(^|,)15($|,)' /sys/devices/system/cpu/offline; then
  # Idempotent reapplication after the exact verified policy is already live.
  topology_verified=true
fi
if ! ${topology_verified}; then
  echo "CPU 14/15 first-run topology or active CPU-14 isolation cannot be proven" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/default/grub
read -r -a current_args <<< "${GRUB_CMDLINE_LINUX_DEFAULT:-}"
filtered_args=()
for argument in "${current_args[@]}"; do
  case "${argument}" in
    nosmt|isolcpus=*|nohz_full=*|rcu_nocbs=*|irqaffinity=*) ;;
    *) filtered_args+=("${argument}") ;;
  esac
done
filtered_args+=(
  nosmt
  isolcpus=domain,managed_irq,14
  nohz_full=14
  rcu_nocbs=14
  irqaffinity=0-13,15-27
)

install -d -m 0755 /etc/default/grub.d
temporary="$(mktemp)"
printf 'GRUB_CMDLINE_LINUX_DEFAULT="%s"\n' "${filtered_args[*]}" > "${temporary}"
install -o root -g root -m 0644 "${temporary}" "${drop_in}"
rm -f -- "${temporary}"
update-grub

echo "installed ${drop_in}; reboot is required"
