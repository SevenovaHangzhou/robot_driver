#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

# shellcheck disable=SC1091
source "${repository_root}/versions.env"

: "${IGH_VERSION:?IGH_VERSION is required in versions.env}"
: "${IGH_COMMIT:?IGH_COMMIT is required in versions.env}"

if [[ "${IGH_VERSION}" != "stable-1.6" ]]; then
  echo "IGH_VERSION must remain frozen at stable-1.6" >&2
  exit 1
fi

ethercat_mac="8c:59:3c:14:ff:d3"
etherlab_prefix="/usr/local/etherlab"
source_root="/usr/local/src/rt-control/igh-ethercat"
source_url="https://gitlab.com/etherlab.org/ethercat.git"
kernel_release="$(uname -r)"
kernel_series="$(printf '%s\n' "${kernel_release}" | grep -oE '^[0-9]+\.[0-9]+')"
case "${kernel_series}" in
  5.15)
    compiler_package="gcc-11"
    compiler_binary="/usr/bin/gcc-11"
    ;;
  6.8)
    compiler_package="gcc-12"
    compiler_binary="/usr/bin/gcc-12"
    ;;
  *)
    echo "unsupported kernel series ${kernel_series}; select a compiler from kernel build evidence" >&2
    exit 1
    ;;
esac
start_master=false

if [[ "${1:-}" == "--start" ]]; then
  start_master=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--start]" >&2
  exit 2
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

if [[ ! "${IGH_COMMIT}" =~ ^[0-9a-f]{40}$ ]]; then
  echo "IGH_COMMIT must be a full 40-character lowercase commit" >&2
  exit 1
fi

if [[ ! -d "/lib/modules/${kernel_release}/build" ]]; then
  echo "missing headers for running kernel ${kernel_release}" >&2
  exit 1
fi

ethercat_interface=""
for interface_path in /sys/class/net/*; do
  [[ -r "${interface_path}/address" ]] || continue
  if [[ "$(<"${interface_path}/address")" == "${ethercat_mac}" ]]; then
    if [[ -n "${ethercat_interface}" ]]; then
      echo "multiple interfaces report EtherCAT MAC ${ethercat_mac}" >&2
      exit 1
    fi
    ethercat_interface="${interface_path##*/}"
  fi
done

interface_claimed_by_master=false
if [[ -n "${ethercat_interface}" ]]; then
  actual_mac="$(<"/sys/class/net/${ethercat_interface}/address")"
  if [[ "${actual_mac,,}" != "${ethercat_mac}" ]]; then
    echo "${ethercat_interface} MAC ${actual_mac} does not match ${ethercat_mac}" >&2
    exit 1
  fi

  if ip -o address show dev "${ethercat_interface}" | grep -Eq '[[:space:]]inet6?[[:space:]]'; then
    echo "${ethercat_interface} has an IP address; remove it before EtherCAT installation" >&2
    exit 1
  fi
else
  # ec_igb removes the native netdev while it owns the I210. A repeat build must
  # validate that exact ownership without stopping a potentially live master.
  # The second path is the read-only legacy 1.5 installation being replaced on
  # the original 5.15-rt IPC; new 1.6 installations always use /etc.
  existing_config=""
  for candidate in /etc/ethercat.conf "${etherlab_prefix}/etc/ethercat.conf"; do
    if [[ -r "${candidate}" ]] &&
       grep -Fqx "MASTER0_DEVICE=\"${ethercat_mac}\"" "${candidate}"; then
      existing_config="${candidate}"
      break
    fi
  done
  master_output="$(${etherlab_prefix}/bin/ethercat master 2>/dev/null || true)"
  if ! systemctl is-active --quiet ethercat.service ||
     [[ ! -x "${etherlab_prefix}/bin/ethercat" ]] ||
     [[ -z "${existing_config}" ]] ||
     ! grep -Fq "Main: ${ethercat_mac} (attached)" <<< "${master_output}"; then
    echo "EtherCAT netdev is absent and exact ec_igb ownership cannot be proven" >&2
    exit 1
  fi
  interface_claimed_by_master=true
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  autoconf \
  automake \
  build-essential \
  can-utils \
  ethtool \
  "${compiler_package}" \
  git \
  "linux-headers-${kernel_release}" \
  libtool \
  pkg-config

install -d -m 0755 "$(dirname -- "${source_root}")"
if [[ ! -d "${source_root}/.git" ]]; then
  if [[ -e "${source_root}" ]]; then
    echo "refusing to replace non-git path ${source_root}" >&2
    exit 1
  fi
  git init "${source_root}"
  git -C "${source_root}" remote add origin "${source_url}"
fi

if [[ "$(git -C "${source_root}" remote get-url origin)" != "${source_url}" ]]; then
  echo "unexpected IgH origin in ${source_root}" >&2
  exit 1
fi

if [[ -n "$(git -C "${source_root}" status --short)" ]]; then
  echo "refusing to use dirty IgH source at ${source_root}" >&2
  exit 1
fi

if ! git -C "${source_root}" cat-file -e "${IGH_COMMIT}^{commit}" 2>/dev/null; then
  git -C "${source_root}" fetch --depth 1 origin "${IGH_COMMIT}"
fi
git -C "${source_root}" checkout --detach "${IGH_COMMIT}"
test "$(git -C "${source_root}" rev-parse HEAD)" = "${IGH_COMMIT}"

if [[ ! -f "${source_root}/devices/igb/igb_main-${kernel_series}-orig.c" ]]; then
  echo "IgH ${IGH_COMMIT} has no ec_igb source for kernel series ${kernel_series}" >&2
  exit 1
fi

build_root="$(mktemp -d /var/tmp/rt-control-igh.XXXXXX)"
cleanup() {
  case "${build_root}" in
    /var/tmp/rt-control-igh.*) rm -rf -- "${build_root}" ;;
  esac
}
trap cleanup EXIT

git -C "${source_root}" archive "${IGH_COMMIT}" | tar -x -C "${build_root}"
cd "${build_root}"
./bootstrap
./configure \
  --prefix="${etherlab_prefix}" \
  --sysconfdir=/etc \
  --disable-initd \
  --disable-rt-syslog \
  --enable-generic \
  --enable-igb \
  --with-systemdsystemunitdir=/lib/systemd/system \
  --with-igb-kernel="${kernel_series}" \
  --with-linux-dir="/lib/modules/${kernel_release}/build"
make -j"$(nproc)" CC="${compiler_binary}" all modules
make CC="${compiler_binary}" modules_install install
depmod -a "${kernel_release}"

install -d -m 0755 /etc/ld.so.conf.d
printf '%s\n' "${etherlab_prefix}/lib" > /etc/ld.so.conf.d/etherlab.conf
ldconfig

metadata_tmp="$(mktemp)"
printf 'IGH_VERSION=%s\nIGH_COMMIT=%s\n' \
  "${IGH_VERSION}" "${IGH_COMMIT}" > "${metadata_tmp}"
install -d -m 0755 /usr/local/share/rt-control
install -o root -g root -m 0644 \
  "${metadata_tmp}" /usr/local/share/rt-control/dependency-versions.env
rm -f -- "${metadata_tmp}"

if [[ -e /usr/local/bin/ethercat && ! -L /usr/local/bin/ethercat ]]; then
  echo "refusing to replace existing /usr/local/bin/ethercat" >&2
  exit 1
fi
ln -sfn "${etherlab_prefix}/bin/ethercat" /usr/local/bin/ethercat

config_tmp="$(mktemp)"
cat > "${config_tmp}" <<EOF
# Managed by rt-control hostsetup/igh-install.sh.
MASTER0_DEVICE="${ethercat_mac}"
DEVICE_MODULES="igb"
UPDOWN_INTERFACES=""
EOF
install -o root -g root -m 0644 "${config_tmp}" /etc/ethercat.conf
rm -f -- "${config_tmp}"

nm_tmp="$(mktemp)"
cat > "${nm_tmp}" <<EOF
# EtherCAT is a fieldbus interface. It must not carry IP, DHCP, or DDS traffic.
[keyfile]
unmanaged-devices=mac:${ethercat_mac}
EOF
install -d -m 0755 /etc/NetworkManager/conf.d
install -o root -g root -m 0644 \
  "${nm_tmp}" /etc/NetworkManager/conf.d/90-rt-control-ethercat.conf
rm -f -- "${nm_tmp}"

override_tmp="$(mktemp)"
cat > "${override_tmp}" <<'EOF'
[Unit]
Before=network-pre.target
Wants=network-pre.target
EOF
install -d -m 0755 /etc/systemd/system/ethercat.service.d
install -o root -g root -m 0644 \
  "${override_tmp}" /etc/systemd/system/ethercat.service.d/50-dependencies.conf
rm -f -- "${override_tmp}"

systemctl daemon-reload
systemctl enable ethercat.service

modinfo -k "${kernel_release}" ec_master >/dev/null
modinfo -k "${kernel_release}" ec_igb >/dev/null
"${etherlab_prefix}/bin/ethercat" version

if ${start_master}; then
  nmcli general reload
  if ! ${interface_claimed_by_master}; then
    nmcli device set "${ethercat_interface}" managed no
  fi
  systemctl restart ethercat.service
  systemctl --no-pager --full status ethercat.service
  "${etherlab_prefix}/bin/ethercat" master
else
  echo "installation complete; rerun with --start only when the EtherCAT ring is connected"
fi
