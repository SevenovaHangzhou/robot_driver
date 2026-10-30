#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
shutdown_patch="${repo_root}/patches/ecat_icube/0003-orderly-master-deactivation.patch"
preserve_pdo_patch="${repo_root}/patches/ecat_icube/0004-preserve-fixed-pdo-config.patch"
igh_preserve_pdo_patch="${repo_root}/patches/igh/0001-preserve-verified-pdo-config.patch"
canopen_lifecycle_patch="${repo_root}/patches/ros2_canopen/0001-shared-canopen-lifecycle.patch"
canopen_rpdo_patch="${repo_root}/patches/ros2_canopen/0006-expose-rpdo-receive-hook.patch"
shutdown_client="${repo_root}/src/rt_control/rt_control_bringup/rt_control_bringup/scope_lifecycle.py"

fail()
{
  echo "FAIL: $*" >&2
  exit 1
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
[[ -f "${canopen_rpdo_patch}" ]] || fail "missing ${canopen_rpdo_patch}"

python3 - \
  "${canopen_lifecycle_patch}" \
  "${canopen_rpdo_patch}" \
  "${shutdown_client}" <<'PY'
import sys
from pathlib import Path

lifecycle = Path(sys.argv[1]).read_text(encoding="utf-8")
rpdo_patch = Path(sys.argv[2]).read_text(encoding="utf-8")
shutdown = Path(sys.argv[3]).read_text(encoding="utf-8")

deactivate = lifecycle.index("     this->deactivate(true);\n+    this->remove_from_master();")
regression = lifecycle.index("EXPECT_CALL(*node_canopen_driver, deactivate(true))")
cancel = lifecycle.index("+  if (!stop_callback_executor())")
release = lifecycle.index("+    if (!device_container_->shutdown_drivers())")
rpdo = rpdo_patch.index("+      on_rpdo_received(data, id, std::chrono::steady_clock::now());")

if not (deactivate >= 0 and regression >= 0 and cancel < release and rpdo >= 0):
    raise SystemExit("CANopen lifecycle or RPDO receive policy is incomplete")
for retired in ("canopen_402_driver/", "cia402_system.cpp", "Track node", "track motor"):
    if retired in lifecycle:
        raise SystemExit(f"retired CANopen motor behavior remains: {retired}")

# Every staged scope stops through scope_lifecycle.shutdown_sequence: motion controllers,
# then the drive disable, then the remaining controllers, and hardware only afterwards.
sequence = shutdown[shutdown.index("def shutdown_sequence("):shutdown.index("class RosLifecycleBackend")]
order = [sequence.index(f'"{label}"') for label in (
    "motion_inactive", "drives_disabled", "controllers_inactive")]
if order != sorted(order):
    raise SystemExit("scope shutdown must stop motion, disable drives, then quiesce controllers")
if sequence.index("controllers_inactive") > sequence.index("hardware_state(name, INACTIVE)"):
    raise SystemExit("scope shutdown must deactivate hardware only after the controllers")
if "if results[-1]:" not in sequence:
    raise SystemExit("hardware must stay up while controllers may still claim its interfaces")
PY

dockerfile="${repo_root}/docker/rt-control/Dockerfile"
fixed_contains '0003-orderly-master-deactivation.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the orderly-deactivation patch"
fixed_contains '0004-preserve-fixed-pdo-config.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the ecat fixed-PDO patch"
fixed_contains '0001-preserve-verified-pdo-config.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the IgH fixed-PDO patch"
fixed_contains '0001-shared-canopen-lifecycle.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the shared CANopen lifecycle patch"
fixed_contains '0006-expose-rpdo-receive-hook.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the CANopen RPDO receive hook patch"
fixed_contains '0005-use-component-parameters-for-ec-modules.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the typed EtherCAT HardwareInfo patch"
fixed_contains '0006-validate-component-module-parameters.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the strict EtherCAT module validation patch"

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
    0004-preserve-fixed-pdo-config.patch \
    0005-use-component-parameters-for-ec-modules.patch \
    0006-validate-component-module-parameters.patch
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

echo "PASS: EtherCAT fixed-PDO verification and ordered shutdown policy"
