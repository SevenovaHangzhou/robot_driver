#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_root="${repo_root}/src/rt_control/robot_hw_ethercat/config/slaves"
shutdown_patch="${repo_root}/patches/ecat_icube/0003-orderly-master-deactivation.patch"
preserve_pdo_patch="${repo_root}/patches/ecat_icube/0004-preserve-fixed-pdo-config.patch"
igh_preserve_pdo_patch="${repo_root}/patches/igh/0001-preserve-verified-pdo-config.patch"
canopen_lifecycle_patch="${repo_root}/patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch"
canopen_quiescence_patch="${repo_root}/patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch"
shutdown_client="${repo_root}/src/rt_control/enable_manager/src/rt_disable_once.cpp"

fail()
{
  echo "FAIL: $*" >&2
  exit 1
}

fixed_line_count()
{
  local expected="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -Fxc -- "${expected}" "${file}" || true
  else
    grep -Fxc -- "${expected}" "${file}" || true
  fi
}

fixed_contains()
{
  local expected="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -Fq -- "${expected}" "${file}"
  else
    grep -Fq -- "${expected}" "${file}"
  fi
}

assert_sync_limit()
{
  local file="$1"
  local type="$2"
  local expected="  - {index: 0x10f1, sub_index: 2, type: ${type}, value: 250}"
  local count

  count="$(fixed_line_count "${expected}" "${file}")"
  [[ "${count}" == "1" ]] ||
    fail "$(basename "${file}") must contain exactly one ${expected}"
}

assert_passive_profile_has_no_startup_sdo()
{
  local file="$1"
  local count
  count="$(fixed_line_count 'sdo:' "${file}")"
  [[ "${count:-0}" == "0" ]] ||
    fail "$(basename "${file}") is passive but configures startup SDO writes"
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

assert_passive_profile_has_no_startup_sdo "${config_root}/x503_right.yaml"
assert_passive_profile_has_no_startup_sdo "${config_root}/x503_left.yaml"

[[ -f "${shutdown_patch}" ]] || fail "missing ${shutdown_patch}"

fixed_contains 'int deactivate(uint32_t preop_timeout_ms);' "${shutdown_patch}" ||
  fail "EcMaster must expose a bounded non-RT deactivate operation"
fixed_contains 'ecrt_master_deactivate(master_)' "${shutdown_patch}" ||
  fail "EcMaster deactivate must call the IgH deactivate API"
fixed_contains 'ecrt_master_get_slave(master_' "${shutdown_patch}" ||
  fail "EcMaster deactivate must confirm configured slave AL states"
fixed_contains 'master_.deactivate(' "${shutdown_patch}" ||
  fail "EthercatDriver on_deactivate must invoke EcMaster::deactivate"
fixed_contains 'ecrt_release_master(master_)' "${shutdown_patch}" ||
  fail "EcMaster destruction must release the requested master"

[[ -f "${preserve_pdo_patch}" ]] || fail "missing ${preserve_pdo_patch}"
[[ -f "${igh_preserve_pdo_patch}" ]] || fail "missing ${igh_preserve_pdo_patch}"
fixed_contains 'use_slave_pdo_defaults' "${preserve_pdo_patch}" ||
  fail "generic EtherCAT slaves must load the fixed-PDO policy"
fixed_contains 'ecrt_slave_config_flag(' "${preserve_pdo_patch}" ||
  fail "ecat adapter must pass the fixed-PDO policy to IgH"
fixed_contains '"PreservePdoConfig", 1' "${preserve_pdo_patch}" ||
  fail "ecat adapter must enable IgH fixed-PDO verification"
fixed_contains 'int pdos_status = ecrt_slave_config_pdos(' "${preserve_pdo_patch}" ||
  fail "fixed-PDO slaves must register the complete expected layout for verification"
fixed_contains 'without writing CoE mapping objects' "${preserve_pdo_patch}" ||
  fail "fixed-PDO policy must document that desired registration does not authorize writes"
fixed_contains 'ec_fsm_pdo_conf_preserve_config' "${igh_preserve_pdo_patch}" ||
  fail "IgH must implement fixed-PDO preservation"
fixed_contains 'ec_pdo_equal_entries' "${igh_preserve_pdo_patch}" ||
  fail "IgH must fail closed on a fixed PDO mapping mismatch"
fixed_contains 'ec_pdo_list_equal' "${igh_preserve_pdo_patch}" ||
  fail "IgH must fail closed on a fixed PDO assignment mismatch"

[[ -f "${canopen_lifecycle_patch}" ]] || fail "missing ${canopen_lifecycle_patch}"
[[ -f "${canopen_quiescence_patch}" ]] || fail "missing ${canopen_quiescence_patch}"

python3 - \
  "${canopen_lifecycle_patch}" \
  "${canopen_quiescence_patch}" \
  "${shutdown_client}" <<'PY'
import sys
from pathlib import Path

lifecycle = Path(sys.argv[1]).read_text(encoding="utf-8")
quiescence = Path(sys.argv[2]).read_text(encoding="utf-8")
shutdown = Path(sys.argv[3]).read_text(encoding="utf-8")

deactivate = lifecycle.index("     this->deactivate(true);\n+    this->remove_from_master();")
regression = lifecycle.index("EXPECT_CALL(*node_canopen_driver, deactivate(true))")
cancel = quiescence.index("+  stop_callback_executor();")
release = quiescence.index("   if (!device_container_->shutdown_drivers()")

if not (deactivate >= 0 and regression >= 0 and cancel < release):
    raise SystemExit("CANopen callback quiescence policy is incomplete or out of order")

main = shutdown.index("int main(")
disable_axes = shutdown.index("disableEthercatAxes(node, deadline)", main)
quiesce_controllers = shutdown.index("quiesceEthercatControllers(node, deadline)", main)
deactivate_hardware = shutdown.index("deactivateEthercatHardware(node, deadline)", main)
if not disable_axes < quiesce_controllers < deactivate_hardware:
    raise SystemExit("full-stack shutdown must disable axes, quiesce controllers, then deactivate EtherCAT")
for required in (
    'std::chrono::seconds(30)',
    '"enable_manager", "rt_internal_state_broadcaster", "joint_state_broadcaster"',
    'kEthercatHardwareName[] = "ecat_arms"',
    'PRIMARY_STATE_INACTIVE',
):
    if required not in shutdown:
        raise SystemExit(f"shutdown orchestration policy is missing: {required}")
PY

dockerfile="${repo_root}/docker/rt-control/Dockerfile"
fixed_contains '0003-orderly-master-deactivation.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the orderly-deactivation patch"
fixed_contains '0004-preserve-fixed-pdo-config.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the ecat fixed-PDO patch"
fixed_contains '0001-preserve-verified-pdo-config.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the IgH fixed-PDO patch"
fixed_contains '0003-quiesce-callbacks-before-driver-removal.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the CANopen callback-quiescence patch"
fixed_contains '0004-name-canopen-master-loop-thread.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the CANopen master thread identity patch"

if [[ -n "${ECAT_ICUBE_SOURCE:-}" ]]; then
  [[ -d "${ECAT_ICUBE_SOURCE}/.git" ]] ||
    fail "ECAT_ICUBE_SOURCE is not a Git checkout: ${ECAT_ICUBE_SOURCE}"

  scratch="$(mktemp -d)"
  trap 'rm -rf "${scratch}"' EXIT
  git clone --quiet --local --no-hardlinks "${ECAT_ICUBE_SOURCE}" "${scratch}/ecat_icube"
  for patch in \
    0001-rt-control-preload-and-diagnostics.patch \
    0002-wait-for-complete-bus-before-preload.patch \
    0003-orderly-master-deactivation.patch \
    0004-preserve-fixed-pdo-config.patch
  do
    git -C "${scratch}/ecat_icube" apply --check \
      "${repo_root}/patches/ecat_icube/${patch}"
    git -C "${scratch}/ecat_icube" apply \
      "${repo_root}/patches/ecat_icube/${patch}"
  done
fi

if [[ -n "${IGH_SOURCE:-}" ]]; then
  [[ -d "${IGH_SOURCE}/.git" ]] ||
    fail "IGH_SOURCE is not a Git checkout: ${IGH_SOURCE}"

  # shellcheck disable=SC1091
  source "${repo_root}/versions.env"
  igh_scratch="$(mktemp -d)"
  if ! git clone --quiet --local --no-hardlinks "${IGH_SOURCE}" "${igh_scratch}/igh"; then
    rm -rf -- "${igh_scratch}"
    fail "failed to clone IGH_SOURCE"
  fi
  git -C "${igh_scratch}/igh" checkout --quiet --detach "${IGH_COMMIT}"
  git -C "${igh_scratch}/igh" apply --check "${igh_preserve_pdo_patch}"
  rm -rf -- "${igh_scratch}"
fi

echo "PASS: EtherCAT motion sync tolerance, passive-SDO exclusion, fixed-PDO verification, and ordered shutdown policy"
