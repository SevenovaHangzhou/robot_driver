#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launcher="${script_dir}/rt_control_ipc.sh"
compose_wrapper="${script_dir}/rt_control_compose.sh"

fail()
{
  echo "FAIL: $*" >&2
  exit 1
}

for required in \
  'readonly expected_hostname="ar-Default-string"' \
  'readonly expected_cpuset="14"' \
  'readonly expected_housekeeping_cpuset="0,2,4,6,8,10,12,16-27"' \
  'readonly expected_container_cpuset="0,2,4,6,8,10,12,14,16-27"' \
  'readonly expected_ethercat_mac="8c:59:3c:14:ff:d3"' \
  'readonly expected_can_serial="004D00675230500720333159"' \
  'readonly expected_bms_can_serial="003000265230500720333159"' \
  'readonly runtime_sha="ad7436b9321d92141899b5dbf2bbb6844df704cb"' \
  'readonly runtime_image_id="sha256:685ddc7ac8623403fb42f19db7f71e0a1bbd2fd82a851eabf0494057bc7511d5"' \
  'ENABLE_RT_CONTROL' \
  'call_rt_service enable' \
  'call_rt_service disable' \
  'call_rt_service reset_fault' \
  'recover-power-loss) recover_power_loss' \
  'RECOVER_RT_CONTROL' \
  'RT_CONTROL_START_CPUSET="${expected_housekeeping_cpuset}"' \
  'prepare_can_interfaces' \
  'pin_controller_update_thread' \
  'controller_manager_running_in_container' \
  'terminate_container_launch_tree' \
  'compose up -d --no-build rt-control' \
  'compose stop rt-control' \
  '"${can_setup_tool}" --wait 30 --configure' \
  'ros2 service type /control/set_enabled' \
  'alfa_control_interfaces/srv/SetControlEnabled' \
  'can1 3FC#' \
  'ros2 topic echo /plc/io_state --once' \
  'ros2 topic echo /battery_state --once' \
  '已记录但按现行裁决不自动停机'
do
  grep -Fq -- "${required}" "${launcher}" || fail "launcher policy is missing: ${required}"
done

if grep -Eq '(^|[[:space:]])docker[[:space:]]+compose' "${launcher}"; then
  fail "launcher must call the approved Compose wrapper"
fi

for required in \
  'RT_CONTROL_PROJECT_ROOT' \
  'Set RT_CONTROL_IMAGE_TAG to the audited release SHA for a non-Git export' \
  'RT_CONTROL_IMAGE_TAG must be a full 40-character lowercase Git SHA'
do
  grep -Fq -- "${required}" "${compose_wrapper}" ||
    fail "Compose wrapper export policy is missing: ${required}"
done

python3 - "${launcher}" <<'PY'
import sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(encoding="utf-8")

start_begin = text.index("start_rt_control()")
start_end = text.index("recover_power_loss()", start_begin)
start = text[start_begin:start_end]
confirm = start.index("confirm_hardware_authorization")
prepare_can = start.index("prepare_can_interfaces")
verify_idle = start.index("verify_idle_bus_inputs")
compose_up = start.index("compose up -d --no-build rt-control")
enable = start.index("call_rt_service enable")
if not confirm < prepare_can < verify_idle < compose_up < enable:
    raise SystemExit("hardware confirmation, CAN preflight, bus verification, container start and enable are misordered")
if "call_rt_service reset_fault" in start:
    raise SystemExit("ordinary one-command start must never reset faults automatically")

cleanup_begin = text.index("on_start_exit()")
cleanup_end = text.index("verify_target_identity()", cleanup_begin)
cleanup = text[cleanup_begin:cleanup_end]
if not cleanup.index("controller_manager_running_in_container") < cleanup.index("call_rt_service disable") < cleanup.index("compose stop rt-control"):
    raise SystemExit("failed startup must check controller_manager before trying disable/stop")

stop_begin = text.index("stop_rt_control()")
stop_end = text.index("print_help()", stop_begin)
stop = text[stop_begin:stop_end]
if not stop.index("controller_manager_running_in_container") < stop.index("call_rt_service disable") < stop.index("compose stop rt-control"):
    raise SystemExit("normal stop must verify controller_manager before disable/stop")

recovery_begin = text.index("recover_power_loss()")
recovery_end = text.index("status_rt_control()", recovery_begin)
recovery = text[recovery_begin:recovery_end]
ordered_recovery = (
    "call_rt_service disable",
    "compose stop rt-control",
    "confirm_power_loss_recovery_authorization",
    "prepare_can_interfaces",
    "verify_idle_bus_inputs",
    "compose up -d --no-build rt-control",
    "call_rt_service reset_fault",
    "check_axis_states disabled",
    "call_rt_service enable",
    "check_axis_states enabled",
)
offsets = [recovery.index(item) for item in ordered_recovery]
if offsets != sorted(offsets):
    raise SystemExit("power-loss recovery sequence violates the frozen order")
PY

echo "PASS: current-IPC launcher is identity locked, confirmation gated and fail-closed"
