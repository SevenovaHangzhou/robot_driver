#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_root="${repo_root}/src/rt_control/robot_hw_ethercat/config/slaves"
shutdown_patch="${repo_root}/patches/ecat_icube/0003-orderly-master-deactivation.patch"

fail()
{
  echo "FAIL: $*" >&2
  exit 1
}

assert_sync_limit()
{
  local file="$1"
  local type="$2"
  local expected="  - {index: 0x10f1, sub_index: 2, type: ${type}, value: 100}"
  local count

  count="$(rg -Fxc -- "${expected}" "${file}" || true)"
  [[ "${count}" == "1" ]] ||
    fail "$(basename "${file}") must contain exactly one ${expected}"
}

for profile in \
  left_j6.yaml \
  right_j6.yaml \
  turn.yaml \
  xmc_updown_sw511.yaml \
  zeroerr_j1.yaml \
  zeroerr_j4.yaml \
  zeroerr_j5.yaml
do
  assert_sync_limit "${config_root}/${profile}" uint16
done

for profile in ti5_j2.yaml ti5_j3.yaml
do
  assert_sync_limit "${config_root}/${profile}" uint32
done

[[ -f "${shutdown_patch}" ]] || fail "missing ${shutdown_patch}"

rg -Fq 'int deactivate(uint32_t preop_timeout_ms);' "${shutdown_patch}" ||
  fail "EcMaster must expose a bounded non-RT deactivate operation"
rg -Fq 'ecrt_master_deactivate(master_)' "${shutdown_patch}" ||
  fail "EcMaster deactivate must call the IgH deactivate API"
rg -Fq 'ecrt_master_get_slave(master_' "${shutdown_patch}" ||
  fail "EcMaster deactivate must confirm configured slave AL states"
rg -Fq 'master_.deactivate(' "${shutdown_patch}" ||
  fail "EthercatDriver on_deactivate must invoke EcMaster::deactivate"
rg -Fq 'ecrt_release_master(master_)' "${shutdown_patch}" ||
  fail "EcMaster destruction must release the requested master"

dockerfile="${repo_root}/docker/rt-control/Dockerfile"
rg -Fq '0003-orderly-master-deactivation.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the orderly-deactivation patch"

if [[ -n "${ECAT_ICUBE_SOURCE:-}" ]]; then
  [[ -d "${ECAT_ICUBE_SOURCE}/.git" ]] ||
    fail "ECAT_ICUBE_SOURCE is not a Git checkout: ${ECAT_ICUBE_SOURCE}"

  scratch="$(mktemp -d)"
  trap 'rm -rf "${scratch}"' EXIT
  git clone --quiet --local "${ECAT_ICUBE_SOURCE}" "${scratch}/ecat_icube"
  for patch in \
    0001-rt-control-preload-and-diagnostics.patch \
    0002-wait-for-complete-bus-before-preload.patch \
    0003-orderly-master-deactivation.patch
  do
    git -C "${scratch}/ecat_icube" apply --check \
      "${repo_root}/patches/ecat_icube/${patch}"
    git -C "${scratch}/ecat_icube" apply \
      "${repo_root}/patches/ecat_icube/${patch}"
  done
fi

echo "PASS: EtherCAT 400 ms sync tolerance and orderly shutdown policy"
