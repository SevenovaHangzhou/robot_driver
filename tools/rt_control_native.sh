#!/usr/bin/env bash
set -euo pipefail

readonly expected_user="ar"
readonly expected_hostname="ar-Default-string"
readonly expected_kernel="5.15.0-1032-realtime"
readonly expected_cpuset="14"
readonly expected_housekeeping_cpuset="0,2,4,6,8,10,12,16-27"
readonly expected_ros_domain_id="0"
readonly expected_ethercat_mac="8c:59:3c:14:ff:d3"
readonly expected_can_serial="004D00675230500720333159"
readonly expected_bms_can_serial="003000265230500720333159"
readonly container_name="robot-rt-control-1"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_root="${RT_CONTROL_NATIVE_WS:-$(cd -- "${repository_root}/.." && pwd)}"
install_root="${workspace_root}/install"
runtime_root="${workspace_root}/.rt-control-native"
runtime_log_root="${workspace_root}/log/native"
pid_file="${runtime_root}/rt_control_start.pid"
latest_log_link="${runtime_root}/latest.log"
installed_start="${install_root}/lib/rt_control_bringup/rt_control_start"
axis_state_checker="${repository_root}/tools/rt_control_axis_state_check.py"
realtime_cpu_guard="${repository_root}/tools/rt_cpu_contamination_check.sh"
thread_affinity_tool="${repository_root}/tools/rt_control_thread_affinity.py"
can_setup_tool="${repository_root}/hostsetup/rt-control-can-names.sh"

readonly repository_root workspace_root install_root runtime_root runtime_log_root
readonly pid_file latest_log_link installed_start axis_state_checker
readonly realtime_cpu_guard thread_affinity_tool can_setup_tool

info()
{
  printf '[rt-control-native] %s\n' "$*"
}

fail()
{
  printf '[rt-control-native] FAIL: %s\n' "$*" >&2
  exit 1
}

usage()
{
  cat <<'EOF'
Native rt-control development runtime for ar-Default-string:

  ./tools/rt_control_native.sh doctor
  ./tools/rt_control_native.sh start
      Start the real control stack without calling /rt/enable.
  ./tools/rt_control_native.sh start-and-enable
      Start the real control stack and call /rt/enable after explicit approval.
  ./tools/rt_control_native.sh recover-power-loss
      Unattended recovery start: stop any old native session, start fresh,
      auto-clear resettable faults once, verify disabled state, call /rt/enable,
      then keep monitoring WARN/ERROR runtime logs in the foreground.
  ./tools/rt_control_native.sh enable
      Call /rt/enable on an already running native stack.
  ./tools/rt_control_native.sh stop
      Call /rt/disable, then signal the installed rt_control_start gate.
  ./tools/rt_control_native.sh status
  ./tools/rt_control_native.sh logs

The runtime is fixed to ROS_DOMAIN_ID=0 and rmw_fastrtps_cpp. It explicitly
removes DDS XML environment variables so Fast DDS keeps its default UDP/SHM
transports. This script never changes the calling shell environment.
EOF
}

source_runtime_environment()
{
  [[ -f /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble is not installed"
  [[ -f "${install_root}/setup.bash" ]] ||
    fail "native workspace is not built: ${install_root}/setup.bash"
  # These sources only affect this script process and its children.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${install_root}/setup.bash"
  set -u
  export PATH="/usr/local/etherlab/bin:${PATH}"
  export LD_LIBRARY_PATH="/usr/local/etherlab/lib:${LD_LIBRARY_PATH:-}"
}

runtime_env()
{
  env \
    -u FASTRTPS_DEFAULT_PROFILES_FILE \
    -u FASTDDS_DEFAULT_PROFILES_FILE \
    -u CYCLONEDDS_URI \
    ROS_DOMAIN_ID="${expected_ros_domain_id}" \
    ROS_LOCALHOST_ONLY="0" \
    RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    "$@"
}

run_ros2_timeout()
{
  local seconds="$1"
  shift
  timeout "${seconds}" env \
    -u FASTRTPS_DEFAULT_PROFILES_FILE \
    -u FASTDDS_DEFAULT_PROFILES_FILE \
    -u CYCLONEDDS_URI \
    ROS_DOMAIN_ID="${expected_ros_domain_id}" \
    ROS_LOCALHOST_ONLY="0" \
    RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    ROS2CLI_NO_DAEMON="1" \
    "$@"
}

runtime_limits_ok()
{
  local memlock_limit
  local realtime_limit
  realtime_limit="$(ulimit -r)"
  memlock_limit="$(ulimit -l)"
  [[ "${realtime_limit}" =~ ^[0-9]+$ ]] && (( realtime_limit >= 98 )) &&
    [[ "${memlock_limit}" == "unlimited" ]]
}

ensure_runtime_limits()
{
  local memlock_limit
  local realtime_limit
  if runtime_limits_ok; then
    return
  fi

  realtime_limit="$(ulimit -r)"
  memlock_limit="$(ulimit -l)"
  info "runtime limits are rtprio=${realtime_limit}, memlock=${memlock_limit}; requesting sudo prlimit for this launcher"
  if [[ -t 0 || -r /dev/tty ]]; then
    sudo prlimit --pid "$$" \
      --rtprio=98:98 \
      --memlock=unlimited:unlimited ||
      fail "sudo prlimit could not raise rtprio/memlock for this launcher"
  else
    sudo -n prlimit --pid "$$" \
      --rtprio=98:98 \
      --memlock=unlimited:unlimited ||
      fail "rtprio/memlock are too low and sudo is not available non-interactively"
  fi

  runtime_limits_ok ||
    fail "runtime limits remain rtprio=$(ulimit -r), memlock=$(ulimit -l); expected rtprio>=98 and memlock=unlimited"
}

call_rt_service()
{
  local operation="$1"
  case "${operation}" in
    enable|disable|reset_fault) ;;
    *) fail "unsupported native lifecycle operation: ${operation}" ;;
  esac
  if [[ "${operation}" == "enable" ]]; then
    pin_controller_update_thread
  fi
  run_ros2_timeout 40 ros2 service call \
    "/rt/${operation}" robot_interfaces/srv/RtEnable '{}'
}

verify_target_identity()
{
  [[ "$(id -un)" == "${expected_user}" ]] ||
    fail "must run as ${expected_user}"
  [[ "$(hostname)" == "${expected_hostname}" ]] ||
    fail "this real-hardware launcher is locked to ${expected_hostname}"
  [[ "$(uname -r)" == "${expected_kernel}" ]] ||
    fail "kernel mismatch: expected ${expected_kernel}"
}

verify_realtime_host()
{
  local memlock_limit
  local realtime_limit
  [[ -r /sys/kernel/realtime && "$(< /sys/kernel/realtime)" == "1" ]] ||
    fail "PREEMPT_RT is not active"
  [[ "$(< /sys/devices/system/cpu/isolated)" == "${expected_cpuset}" ]] ||
    fail "CPU ${expected_cpuset} is not the frozen isolated CPU"
  [[ "$(< /sys/devices/system/cpu/nohz_full)" == "${expected_cpuset}" ]] ||
    fail "CPU ${expected_cpuset} is not full-nohz"
  grep -Eq '(^|,)15($|,)' /sys/devices/system/cpu/offline ||
    fail "CPU 15, the sibling of CPU 14, is not offline"
  verify_igh_run_on_cpu_config
  ensure_runtime_limits

  realtime_limit="$(ulimit -r)"
  [[ "${realtime_limit}" =~ ^[0-9]+$ ]] && (( realtime_limit >= 98 )) ||
    fail "rtprio limit is ${realtime_limit}; expected at least 98"
  memlock_limit="$(ulimit -l)"
  [[ "${memlock_limit}" == "unlimited" ]] ||
    fail "memlock limit is ${memlock_limit}; expected unlimited"
}

verify_igh_run_on_cpu_config()
{
  local config="/etc/modprobe.d/ec_master.conf"
  local expected_line="options ec_master run_on_cpu=${expected_cpuset}"
  [[ -r "${config}" ]] || fail "missing IgH CPU binding config: ${config}"
  grep -Fxq "${expected_line}" "${config}" ||
    fail "IgH ec_master is not pinned to isolated CPU ${expected_cpuset}"
}

verify_workspace()
{
  [[ -x "${installed_start}" ]] || fail "missing installed signal gate: ${installed_start}"
  [[ -x "${install_root}/lib/enable_manager/rt_disable_once" ]] ||
    fail "missing installed shutdown helper"
  [[ -f /usr/local/share/rt-control/dependency-versions.env ]] ||
    fail "missing frozen IgH dependency identity"
  [[ -e /dev/EtherCAT0 ]] || fail "missing /dev/EtherCAT0"
  [[ -x "${realtime_cpu_guard}" ]] ||
    fail "missing executable realtime CPU guard: ${realtime_cpu_guard}"
  [[ -x "${thread_affinity_tool}" ]] ||
    fail "missing executable thread affinity helper: ${thread_affinity_tool}"
  [[ -x "${can_setup_tool}" ]] ||
    fail "missing executable CAN setup helper: ${can_setup_tool}"
  command -v taskset >/dev/null 2>&1 || fail "missing taskset"
  command -v setsid >/dev/null 2>&1 || fail "missing setsid"
  command -v ethercat >/dev/null 2>&1 || fail "missing ethercat CLI"
}

verify_realtime_cpu_guard()
{
  "${realtime_cpu_guard}" --cpu "${expected_cpuset}" --mode strict
}

pin_controller_update_thread()
{
  python3 "${thread_affinity_tool}" \
    --rt-cpu "${expected_cpuset}" \
    --housekeeping-cpus "${expected_housekeeping_cpuset}" \
    --required-rt-thread-name rtcan-master \
    --rt-priority 80 \
    --deadline 5
}

verify_bus_services()
{
  systemctl is-active --quiet ethercat.service ||
    fail "EtherCAT service must be active"
}

prepare_can_interfaces()
{
  info "preparing can0/can1 by fixed gs_usb serials at 500 kbit/s, txqueuelen 128"
  if [[ ${EUID} -eq 0 ]]; then
    "${can_setup_tool}" --wait 30 --configure ||
      fail "CAN preflight failed; check the missing adapter message above"
  elif [[ -t 0 || -r /dev/tty ]]; then
    sudo "${can_setup_tool}" --wait 30 --configure ||
      fail "CAN preflight failed; check the missing adapter message above"
  else
    sudo -n "${can_setup_tool}" --wait 30 --configure ||
      fail "CAN preflight failed and sudo is not available non-interactively"
  fi
}

verify_can_interface()
{
  local expected_serial="$2"
  local interface="$1"
  local observed_serial
  local output
  observed_serial="$(
    udevadm info -q property -p "/sys/class/net/${interface}" 2>/dev/null |
      sed -n 's/^ID_SERIAL_SHORT=//p'
  )"
  [[ "${observed_serial}" == "${expected_serial}" ]] ||
    fail "${interface} USB serial mismatch"
  output="$(ip -details -statistics link show "${interface}")"
  grep -Fq 'state UP' <<< "${output}" || fail "${interface} is not UP"
  grep -Fq 'can state ERROR-ACTIVE' <<< "${output}" ||
    fail "${interface} is not ERROR-ACTIVE"
  grep -Fq 'bitrate 500000' <<< "${output}" ||
    fail "${interface} is not 500 kbit/s"
}

verify_idle_ethercat()
{
  local master
  local slaves
  master="$(ethercat master)"
  slaves="$(ethercat slaves)"
  grep -Fq "Main: ${expected_ethercat_mac} (attached)" <<< "${master}" ||
    fail "EtherCAT master MAC mismatch"
  grep -Fq 'Link: UP' <<< "${master}" || fail "EtherCAT link is not UP"
  grep -Fq 'Phase: Idle' <<< "${master}" || fail "EtherCAT master is not Idle"
  grep -Fq 'Active: no' <<< "${master}" || fail "EtherCAT master is already active"
  grep -Fq 'Slaves: 16' <<< "${master}" || fail "EtherCAT does not report 16 slaves"
  [[ "$(wc -l <<< "${slaves}")" -eq 16 ]] || fail "EtherCAT scan is not 16 positions"
}

native_pid()
{
  [[ -f "${pid_file}" ]] || return 1
  local pid
  pid="$(< "${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null | grep -Fq "${installed_start}" ||
    return 1
  printf '%s\n' "${pid}"
}

cleanup_stale_pid_file()
{
  if [[ -f "${pid_file}" ]] && ! native_pid >/dev/null 2>&1; then
    rm -f -- "${pid_file}"
  fi
}

reject_running_container()
{
  local state
  command -v docker >/dev/null 2>&1 || fail "Docker CLI is required for coexistence checks"
  docker ps --format '{{.Names}}' >/dev/null 2>&1 ||
    fail "cannot inspect Docker; native/container coexistence cannot be ruled out"
  state="$(docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "${state}" != "running" && "${state}" != "restarting" ]] ||
    fail "${container_name} is ${state}; stop the container before native start"
}

confirm_start_authorization()
{
  local answer
  [[ -t 0 && -r /dev/tty ]] || fail "real-hardware native start requires an interactive terminal"
  printf '%s\n' \
    "Native start accesses real EtherCAT/CANopen hardware but does not call /rt/enable." \
    "Confirm the emergency stop and exclusion zone are ready." \
    "Type START_RT_CONTROL_NATIVE to continue:"
  IFS= read -r answer < /dev/tty
  [[ "${answer}" == "START_RT_CONTROL_NATIVE" ]] || fail "native start not authorized"
}

confirm_enable_authorization()
{
  local answer
  [[ -t 0 && -r /dev/tty ]] || fail "native enable requires an interactive terminal"
  printf '%s\n' \
    "This operation starts real buses if needed and calls /rt/enable." \
    "Confirm all 14 axes and tracks may be energized." \
    "Type ENABLE_RT_CONTROL_NATIVE to continue:"
  IFS= read -r answer < /dev/tty
  [[ "${answer}" == "ENABLE_RT_CONTROL_NATIVE" ]] || fail "native enable not authorized"
}

launch_native()
{
  local log_file
  local pid
  mkdir -p "${runtime_root}" "${runtime_log_root}"
  log_file="${runtime_log_root}/rt-control-$(date +%Y%m%d-%H%M%S).log"
  ln -sfn -- "${log_file}" "${latest_log_link}"

  info "starting installed rt_control_start on housekeeping CPUs ${expected_housekeeping_cpuset}; rt CPU=${expected_cpuset}; log=${log_file}"
  nohup setsid env \
    -u FASTRTPS_DEFAULT_PROFILES_FILE \
    -u FASTDDS_DEFAULT_PROFILES_FILE \
    -u CYCLONEDDS_URI \
    ROS_DOMAIN_ID="${expected_ros_domain_id}" \
    ROS_LOCALHOST_ONLY="0" \
    RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
    RT_CONTROL_START_PLC="${RT_CONTROL_START_PLC:-true}" \
    RT_CONTROL_START_BMS="${RT_CONTROL_START_BMS:-true}" \
    PATH="${PATH}" \
    LD_LIBRARY_PATH="${LD_LIBRARY_PATH}" \
    taskset --cpu-list "${expected_housekeeping_cpuset}" \
    "${installed_start}" > "${log_file}" 2>&1 < /dev/null &
  pid=$!
  printf '%s\n' "${pid}" > "${pid_file}"
}

wait_for_enable_service()
{
  local deadline=$((SECONDS + 90))
  local pid
  while (( SECONDS < deadline )); do
    pid="$(native_pid 2>/dev/null || true)"
    [[ -n "${pid}" ]] || return 1
    if controller_manager_died_during_boot; then
      print_first_boot_error
      terminate_failed_start
      fail "ros2_control_node exited during boot; launch/spawner process tree was terminated"
    fi
    if run_ros2_timeout 3 ros2 service type /rt/enable 2>/dev/null |
      grep -Fq 'robot_interfaces/srv/RtEnable' &&
      run_ros2_timeout 3 ros2 service type /control/set_enabled 2>/dev/null |
      grep -Fq 'alfa_control_interfaces/srv/SetControlEnabled'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

terminate_failed_start()
{
  local pid
  pid="$(native_pid 2>/dev/null || true)"
  if [[ -n "${pid}" ]]; then
    terminate_native_launch_tree "${pid}"
  fi
  rm -f -- "${pid_file}"
}

strip_ansi()
{
  sed -E $'s/\033\[[0-9;]*[mK]//g'
}

native_descendants()
{
  local parent="$1"
  local child
  for child in $(pgrep -P "${parent}" 2>/dev/null || true); do
    printf '%s\n' "${child}"
    native_descendants "${child}"
  done
}

controller_manager_died_during_boot()
{
  [[ -e "${latest_log_link}" ]] || return 1
  strip_ansi < "${latest_log_link}" | grep -Eq \
    'ros2_control_node-[0-9]+.*process has died|process\[ros2_control_node-[0-9]+\].*died|controller_manager.*(Segmentation fault|terminate called|Exception|exception|FATAL|ERROR)'
}

controller_manager_running_under_native()
{
  local pid
  pid="$(native_pid 2>/dev/null || true)"
  [[ -n "${pid}" ]] || return 1
  while read -r child; do
    [[ -r "/proc/${child}/cmdline" ]] || continue
    tr '\0' ' ' < "/proc/${child}/cmdline" |
      grep -Fq '/controller_manager/ros2_control_node' && return 0
  done < <(native_descendants "${pid}")
  return 1
}

print_first_boot_error()
{
  local first_error=""
  if [[ -e "${latest_log_link}" ]]; then
    first_error="$(strip_ansi < "${latest_log_link}" |
      grep -m 1 -E '(\[ERROR\]|\[FATAL\]|process has died|Segmentation fault|terminate called|Exception|exception|Traceback|Failed|failed|abort|what\(\):)' || true)"
  fi
  if [[ -n "${first_error}" ]]; then
    info "first boot error: ${first_error}"
  else
    info "first boot error: unavailable in ${latest_log_link}"
  fi
}

terminate_native_launch_tree()
{
  local pid="$1"
  local child
  local deadline
  local pgid
  local pids=()
  mapfile -t pids < <(native_descendants "${pid}")
  for child in "${pids[@]}"; do
    [[ -n "${child}" ]] || continue
    pgid="$(ps -o pgid= -p "${child}" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ -n "${pgid}" && "${pgid}" != "$$" ]] || continue
    kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for child in "${pids[@]}"; do
    kill -TERM "${child}" 2>/dev/null || true
  done
  deadline=$((SECONDS + 10))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if kill -0 "${pid}" 2>/dev/null; then
    for child in "${pids[@]}"; do
      kill -KILL "${child}" 2>/dev/null || true
    done
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

wait_for_idle_ethercat()
{
  local deadline=$((SECONDS + 20))
  local master
  while (( SECONDS < deadline )); do
    master="$(ethercat master 2>/dev/null || true)"
    if grep -Fq 'Phase: Idle' <<< "${master}" &&
      grep -Fq 'Active: no' <<< "${master}"; then
      return
    fi
    sleep 1
  done
  fail "EtherCAT master did not return to Idle/Inactive within 20 seconds"
}

best_effort_stop_existing_native_for_recovery()
{
  local deadline
  local pid
  local response
  source_runtime_environment
  pid="$(native_pid 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    cleanup_stale_pid_file
    wait_for_idle_ethercat
    return
  fi

  info "best-effort disabling old native session before recovery"
  if response="$(call_rt_service disable)"; then
    printf '%s\n' "${response}"
    if ! grep -Fq 'ok=True' <<< "${response}"; then
      info "WARN: old native /rt/disable did not confirm success; continuing recovery teardown"
    fi
  else
    info "WARN: old native /rt/disable was unavailable; continuing recovery teardown"
  fi

  kill -TERM "${pid}" 2>/dev/null || true
  deadline=$((SECONDS + 100))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    fail "old native runtime exceeded the 100-second recovery shutdown deadline; no SIGKILL was sent"
  fi
  rm -f -- "${pid_file}"
  if [[ -e "${latest_log_link}" ]] &&
    tail -n 200 "${latest_log_link}" | grep -Fq 'UNCLEAN_SHUTDOWN'; then
    info "WARN: old native session reported UNCLEAN_SHUTDOWN; fresh recovery session will continue"
  fi
  wait_for_idle_ethercat
}

wait_for_enable_manager_reset_ready()
{
  local deadline=$((SECONDS + 120))
  local snapshot
  while (( SECONDS < deadline )); do
    snapshot="$(
      run_ros2_timeout 8 ros2 topic echo --once /diagnostics \
        diagnostic_msgs/msg/DiagnosticArray \
        --filter "any(s.name == '/robot/rt_control/enable_manager' for s in m.status)" \
        2>/dev/null || true
    )"
    if grep -Fq 'value: FAILED' <<< "${snapshot}" ||
      grep -Fq 'value: restart_required' <<< "${snapshot}"; then
      if grep -Fq 'value: fault_requires_reset' <<< "${snapshot}"; then
        return
      fi
      printf '%s\n' "${snapshot}" >&2
      fail "enable_manager failed for a non-resettable stage; refusing to hide it with fault reset"
    fi
    if grep -Fq 'value: IDLE' <<< "${snapshot}"; then
      return
    fi
    sleep 1
  done
  fail "enable_manager did not reach IDLE or fault_requires_reset within 120 seconds"
}

enable_manager_diagnostic_snapshot()
{
  run_ros2_timeout 8 ros2 topic echo --once /diagnostics \
    diagnostic_msgs/msg/DiagnosticArray \
    --filter "any(s.name == '/robot/rt_control/enable_manager' for s in m.status)" \
    2>/dev/null || true
}

print_fault_recovery_context()
{
  local diagnostic_snapshot
  local reset_response="$1"
  printf '[rt-control-native] ERROR: resettable fault could not be cleared automatically.\n' >&2
  printf '[rt-control-native] ERROR: /rt/reset_fault response follows:\n%s\n' "${reset_response}" >&2
  diagnostic_snapshot="$(enable_manager_diagnostic_snapshot)"
  if [[ -n "${diagnostic_snapshot}" ]]; then
    printf '[rt-control-native] ERROR: enable_manager diagnostic follows:\n%s\n' "${diagnostic_snapshot}" >&2
  else
    printf '[rt-control-native] ERROR: enable_manager diagnostic was unavailable.\n' >&2
  fi
  printf '[rt-control-native] ERROR: do not enable; inspect the named drive/statusword, then power-cycle the main contactor and restart rt-control if the fault remains latched.\n' >&2
}

clear_resettable_faults_before_enable()
{
  local diagnostic_snapshot
  local reset_response
  diagnostic_snapshot="$(enable_manager_diagnostic_snapshot)"
  if ! grep -Fq 'value: fault_requires_reset' <<< "${diagnostic_snapshot}"; then
    info "no resettable enable_manager fault reported before enable"
    return
  fi

  info "resettable enable_manager fault reported; calling one group /rt/reset_fault"
  reset_response="$(call_rt_service reset_fault)" || {
    print_fault_recovery_context "/rt/reset_fault service call did not return"
    best_effort_stop_existing_native_for_recovery
    fail "fault could not be cleared; power-cycle the main contactor and restart rt-control"
  }
  printf '%s\n' "${reset_response}"
  if ! grep -Fq 'ok=True' <<< "${reset_response}" ||
    ! grep -Eq "stage='(success|already_clear)'" <<< "${reset_response}"; then
    print_fault_recovery_context "${reset_response}"
    best_effort_stop_existing_native_for_recovery
    fail "fault could not be cleared; power-cycle the main contactor and restart rt-control"
  fi
}

verify_enable_manager_diagnostic_state()
{
  local expected_state="$1"
  local expected_stage="$2"
  local deadline=$((SECONDS + 15))
  local snapshot
  snapshot=""
  while (( SECONDS < deadline )); do
    snapshot="$(enable_manager_diagnostic_snapshot)"
    if grep -Fq 'name: /robot/rt_control/enable_manager' <<< "${snapshot}" &&
      grep -Fq "value: ${expected_state}" <<< "${snapshot}" &&
      grep -Fq "value: ${expected_stage}" <<< "${snapshot}" &&
      grep -Fq 'message: operational' <<< "${snapshot}"; then
      return
    fi
    sleep 1
  done
  printf '%s\n' "${snapshot}" >&2
  fail "enable_manager did not report ${expected_state}/${expected_stage} within 15 seconds"
}

verify_controllers_before_enable()
{
  local deadline=$((SECONDS + 30))
  local controller_output
  controller_output=""
  while (( SECONDS < deadline )); do
    controller_output="$(run_ros2_timeout 8 ros2 control list_controllers 2>/dev/null | strip_ansi || true)"
    if grep -Eq '^joint_state_broadcaster[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^diff_drive_controller[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^enable_manager[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^whole_body_jtc[[:space:]].*inactive' <<< "${controller_output}"; then
      return
    fi
    sleep 1
  done
  printf '%s\n' "${controller_output}" >&2
  fail "controllers did not reach pre-enable states within 30 seconds: JSB/diff-drive/enable-manager active and whole_body_jtc inactive"
}

verify_enabled_controllers()
{
  local controller_output
  controller_output="$(run_ros2_timeout 15 ros2 control list_controllers | strip_ansi)"
  grep -Eq '^diff_drive_controller[[:space:]].*active' <<< "${controller_output}" ||
    fail "diff_drive_controller is not active after enable"
  grep -Eq '^joint_state_broadcaster[[:space:]].*active' <<< "${controller_output}" ||
    fail "joint_state_broadcaster is not active after enable"
  grep -Eq '^enable_manager[[:space:]].*active' <<< "${controller_output}" ||
    fail "enable_manager is not active after enable"
  grep -Eq '^whole_body_jtc[[:space:]].*active' <<< "${controller_output}" ||
    fail "whole_body_jtc is not active after enable"
}

check_axis_states()
{
  local deadline=$((SECONDS + 30))
  local expected="$1"
  local last_error=""
  local snapshot
  [[ -f "${axis_state_checker}" && -r "${axis_state_checker}" ]] ||
    fail "missing or unreadable axis-state checker: ${axis_state_checker}"
  while (( SECONDS < deadline )); do
    if ! snapshot="$(
      run_ros2_timeout 8 ros2 topic echo --once /dynamic_joint_states \
        control_msgs/msg/DynamicJointState 2>&1
    )"; then
      last_error="${snapshot}"
      sleep 1
      continue
    fi
    if printf '%s\n' "${snapshot}" | python3 "${axis_state_checker}" --expected "${expected}"; then
      return
    fi
    last_error="${snapshot}"
    sleep 1
  done
  printf '%s\n' "${last_error}" >&2
  fail "14-axis ${expected} CiA 402 contract is not satisfied after 30 seconds"
}

verify_operational_ethercat()
{
  local master
  local slaves
  master="$(ethercat master)"
  slaves="$(ethercat slaves)"
  grep -Fq 'Phase: Operation' <<< "${master}" || fail "EtherCAT master is not Operation"
  grep -Fq 'Active: yes' <<< "${master}" || fail "EtherCAT master is not Active"
  grep -Fq 'Slaves: 16' <<< "${master}" || fail "EtherCAT does not report 16 slaves"
  [[ "$(grep -Ec '^[[:space:]]*[0-9]+[[:space:]]+[^[:space:]]+[[:space:]]+OP[[:space:]]+\+' <<< "${slaves}")" -ge 14 ]] ||
    fail "EtherCAT drive slaves are not OP"
  verify_ethercat_op_thread_cpu
}

verify_ethercat_op_thread_cpu()
{
  local allowed
  local cls
  local found=0
  local rtprio
  local status_file
  local tid
  while read -r tid cls rtprio; do
    [[ -n "${tid}" ]] || continue
    status_file="/proc/${tid}/status"
    [[ -r "${status_file}" ]] || continue
    found=1
    allowed="$(sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' "${status_file}")"
    [[ "${allowed}" == "${expected_cpuset}" ]] ||
      fail "EtherCAT-OP thread ${tid} CPU affinity is ${allowed}, expected ${expected_cpuset}"
    case "${cls}" in
      TS)
        info "EtherCAT-OP thread ${tid} is SCHED_OTHER; field acceptance requires FIFO80 update duty <70% under navigation load"
        ;;
      FF|RR)
        info "EtherCAT-OP thread ${tid} is ${cls} rtprio=${rtprio}; if rtprio >80, verify it does not cut the control cycle"
        ;;
      *)
        info "EtherCAT-OP thread ${tid} scheduler class=${cls} rtprio=${rtprio}"
        ;;
    esac
  done < <(ps -eLo tid=,cls=,rtprio=,comm= | awk '$4 == "EtherCAT-OP" {print $1, $2, $3}')
  (( found == 1 )) || fail "EtherCAT-OP thread is not visible"
}

start_native()
{
  local authorization="${1:-interactive}"
  verify_target_identity
  verify_realtime_host
  verify_workspace
  verify_bus_services
  reject_running_container
  cleanup_stale_pid_file
  if native_pid >/dev/null 2>&1; then
    fail "native rt-control is already running"
  fi
  verify_realtime_cpu_guard
  verify_idle_ethercat
  if [[ "${authorization}" != "preauthorized" ]]; then
    confirm_start_authorization
  fi
  prepare_can_interfaces
  verify_can_interface can0 "${expected_can_serial}"
  verify_can_interface can1 "${expected_bms_can_serial}"
  source_runtime_environment
  launch_native
  if ! wait_for_enable_service; then
    terminate_failed_start
    fail "native stack did not expose /rt/enable and /control/set_enabled within 90 seconds; stop was requested"
  fi
  info "READY: native stack is running in Domain ${expected_ros_domain_id}; drives were not enabled; public service /control/set_enabled is available"
}

enable_native()
{
  local response
  source_runtime_environment
  native_pid >/dev/null 2>&1 || fail "native rt-control is not running"
  confirm_enable_authorization
  response="$(call_rt_service enable)" || fail "/rt/enable did not return"
  printf '%s\n' "${response}"
  grep -Fq 'ok=True' <<< "${response}" || fail "/rt/enable returned failure"
  info "ENABLED: all enable-manager acceptance checks passed"
}

start_and_enable_native()
{
  local response
  confirm_enable_authorization
  start_native preauthorized
  response="$(call_rt_service enable)" || {
    stop_native best-effort
    fail "/rt/enable did not return; native stop was requested"
  }
  printf '%s\n' "${response}"
  if ! grep -Fq 'ok=True' <<< "${response}"; then
    stop_native best-effort
    fail "/rt/enable returned failure; native stop was requested"
  fi
  info "ENABLED: native stack is ready"
}

recover_power_loss_native()
{
  local enable_response
  verify_target_identity
  verify_realtime_host
  verify_workspace
  verify_bus_services
  reject_running_container
  best_effort_stop_existing_native_for_recovery
  prepare_can_interfaces
  verify_can_interface can0 "${expected_can_serial}"
  verify_can_interface can1 "${expected_bms_can_serial}"

  start_native preauthorized
  wait_for_enable_manager_reset_ready
  verify_controllers_before_enable
  verify_operational_ethercat

  clear_resettable_faults_before_enable
  verify_enable_manager_diagnostic_state IDLE success

  info "calling one /rt/enable after reset and disabled-state proof"
  enable_response="$(call_rt_service enable)" || {
    best_effort_stop_existing_native_for_recovery
    fail "/rt/enable did not return; native recovery session was stopped"
  }
  printf '%s\n' "${enable_response}"
  if ! grep -Fq 'ok=True' <<< "${enable_response}" ||
    ! grep -Eq "stage='(success|already_enabled)'" <<< "${enable_response}"; then
    best_effort_stop_existing_native_for_recovery
    fail "/rt/enable returned failure; native recovery session was stopped"
  fi
  verify_enable_manager_diagnostic_state ENABLED success
  verify_enabled_controllers
  verify_operational_ethercat
  info "RECOVERED: native stack is running and enabled"
  if should_monitor_native_logs; then
    monitor_native_warning_logs
  else
    info "WARN/ERROR runtime log monitor disabled by RT_CONTROL_NATIVE_MONITOR=${RT_CONTROL_NATIVE_MONITOR}"
  fi
}

stop_native()
{
  local mode="${1:-strict}"
  local disable_ok=0
  local pid
  local response
  local deadline
  source_runtime_environment
  pid="$(native_pid 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    cleanup_stale_pid_file
    info "native rt-control is already stopped"
    return
  fi

  if ! controller_manager_running_under_native; then
    print_first_boot_error
    terminate_native_launch_tree "${pid}"
    rm -f -- "${pid_file}"
    fail "controller_manager is not running; launch/spawner process tree was terminated without waiting for /rt/disable"
  fi

  info "requesting /rt/disable before signalling rt_control_start"
  if response="$(call_rt_service disable)"; then
    printf '%s\n' "${response}"
    if grep -Fq 'ok=True' <<< "${response}"; then
      disable_ok=1
    fi
  else
    info "WARN: /rt/disable was unavailable; the signal gate will retry orderly shutdown"
  fi

  kill -TERM "${pid}"
  deadline=$((SECONDS + 100))
  while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    fail "native runtime exceeded the 100-second shutdown deadline; no SIGKILL was sent"
  fi
  rm -f -- "${pid_file}"

  if [[ -e "${latest_log_link}" ]] &&
    tail -n 200 "${latest_log_link}" | grep -Fq 'UNCLEAN_SHUTDOWN'; then
    fail "native runtime stopped but reported UNCLEAN_SHUTDOWN"
  fi
  if (( disable_ok == 0 )) && [[ "${mode}" != "best-effort" ]]; then
    fail "native runtime stopped without an explicit successful /rt/disable"
  fi
  info "STOPPED: native rt-control process exited"
}

status_native()
{
  local pid
  local state
  cleanup_stale_pid_file
  pid="$(native_pid 2>/dev/null || true)"
  state="$(docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  printf 'workspace=%s\n' "${workspace_root}"
  printf 'ros_domain_id=%s\n' "${expected_ros_domain_id}"
  printf 'rt_cpu=%s\n' "${expected_cpuset}"
  printf 'housekeeping_cpus=%s\n' "${expected_housekeeping_cpuset}"
  printf 'container_state=%s\n' "${state:-absent}"
  if [[ -z "${pid}" ]]; then
    printf 'native_state=stopped\n'
    return
  fi
  printf 'native_state=running\n'
  printf 'pid=%s\n' "${pid}"
  taskset -pc "${pid}" 2>/dev/null || true
  tr '\0' '\n' < "/proc/${pid}/environ" 2>/dev/null |
    grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION)=' || true
  printf 'log=%s\n' "$(readlink -f "${latest_log_link}")"
}

logs_native()
{
  [[ -e "${latest_log_link}" ]] || fail "no native runtime log exists"
  tail -n 200 -f "${latest_log_link}"
}

should_monitor_native_logs()
{
  case "${RT_CONTROL_NATIVE_MONITOR:-true}" in
    0|false|FALSE|no|NO|off|OFF)
      return 1
      ;;
    *)
      return 0
      ;;
  esac
}

monitor_native_warning_logs()
{
  [[ -e "${latest_log_link}" ]] || fail "no native runtime log exists"
  info "monitoring WARN/ERROR runtime logs from $(readlink -f "${latest_log_link}"); Ctrl+C exits monitor only"
  tail -n 0 -F "${latest_log_link}" 2>/dev/null |
    grep --line-buffered -Eai 'WARN|ERROR|FAIL|Fault|fault|EMCY|timeout|timed out|UNCLEAN_SHUTDOWN|bus-off|bus off|throttl|overrun|deadline|exception|segmentation|abort'
}

doctor_native()
{
  verify_target_identity
  verify_realtime_host
  verify_workspace
  reject_running_container
  source_runtime_environment
  runtime_env ros2 pkg prefix rt_control_bringup >/dev/null
  runtime_env ros2 pkg prefix enable_manager >/dev/null
  runtime_env ros2 pkg prefix alfa_control_interfaces >/dev/null
  runtime_env ros2 pkg prefix control_api_adapter >/dev/null
  info "PASS: native runtime is installed for Domain ${expected_ros_domain_id} with Fast DDS default transports"
}

case "${1:-}" in
  doctor) doctor_native ;;
  start) start_native ;;
  start-and-enable) start_and_enable_native ;;
  recover-power-loss) recover_power_loss_native ;;
  enable) enable_native ;;
  stop) stop_native ;;
  status) status_native ;;
  logs) logs_native ;;
  -h|--help|help|"") usage ;;
  *) usage >&2; exit 2 ;;
esac
