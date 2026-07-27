#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_root="${repo_root}/src/rt_control/robot_hw_ethercat/config/slaves"
shutdown_patch="${repo_root}/patches/ecat_icube/0003-orderly-master-deactivation.patch"
canopen_lifecycle_patch="${repo_root}/patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch"
canopen_quiescence_patch="${repo_root}/patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch"
shutdown_client="${repo_root}/src/rt_control/enable_manager/src/rt_disable_once.cpp"

fail()
{
  echo "FAIL: $*" >&2
  exit 1
}

assert_sync_limit()
{
  local file="$1"
  local type="$2"
  local expected="  - {index: 0x10f1, sub_index: 2, type: ${type}, value: 250}"
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
    '"enable_manager", "joint_state_broadcaster"',
    'kEthercatHardwareName[] = "ecat_arms"',
    'PRIMARY_STATE_INACTIVE',
):
    if required not in shutdown:
        raise SystemExit(f"shutdown orchestration policy is missing: {required}")
PY

dockerfile="${repo_root}/docker/rt-control/Dockerfile"
rg -Fq '0003-orderly-master-deactivation.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the orderly-deactivation patch"
rg -Fq '0003-quiesce-callbacks-before-driver-removal.patch' "${dockerfile}" ||
  fail "Dockerfile must apply the CANopen callback-quiescence patch"

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

echo "PASS: EtherCAT 1000 ms sync tolerance and ordered full-stack shutdown policy"
