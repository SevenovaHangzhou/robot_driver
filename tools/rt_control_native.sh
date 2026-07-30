#!/usr/bin/env bash
set -euo pipefail

readonly expected_user="ar"
readonly expected_hostname="ar-Default-string"
readonly expected_kernel="5.15.0-1032-realtime"
readonly expected_cpuset="14"
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

readonly repository_root workspace_root install_root runtime_root runtime_log_root
readonly pid_file latest_log_link installed_start

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
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${install_root}/setup.bash"
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

call_rt_service()
{
  local operation="$1"
  case "${operation}" in
    enable|disable) ;;
    *) fail "unsupported native lifecycle operation: ${operation}" ;;
  esac
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

  realtime_limit="$(ulimit -r)"
  [[ "${realtime_limit}" =~ ^[0-9]+$ ]] && (( realtime_limit >= 98 )) ||
    fail "rtprio limit is ${realtime_limit}; expected at least 98"
  memlock_limit="$(ulimit -l)"
  [[ "${memlock_limit}" == "unlimited" ]] ||
    fail "memlock limit is ${memlock_limit}; expected unlimited"
}

verify_workspace()
{
  [[ -x "${installed_start}" ]] || fail "missing installed signal gate: ${installed_start}"
  [[ -x "${install_root}/lib/enable_manager/rt_disable_once" ]] ||
    fail "missing installed shutdown helper"
  [[ -f /usr/local/share/rt-control/dependency-versions.env ]] ||
    fail "missing frozen IgH dependency identity"
  [[ -e /dev/EtherCAT0 ]] || fail "missing /dev/EtherCAT0"
  command -v taskset >/dev/null 2>&1 || fail "missing taskset"
  command -v ethercat >/dev/null 2>&1 || fail "missing ethercat CLI"
}

verify_bus_services()
{
  systemctl is-active --quiet \
    ethercat.service rt-control-can-names.service can0.service can1.service ||
    fail "EtherCAT, CAN naming, can0 and can1 services must be active"
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

  info "starting installed rt_control_start on CPU ${expected_cpuset}; log=${log_file}"
  nohup env \
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
    taskset --cpu-list "${expected_cpuset}" \
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
    if run_ros2_timeout 3 ros2 service type /rt/enable 2>/dev/null |
      grep -Fq 'robot_interfaces/srv/RtEnable'; then
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
    kill -TERM "${pid}" 2>/dev/null || true
  fi
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
  verify_idle_ethercat
  verify_can_interface can0 "${expected_can_serial}"
  verify_can_interface can1 "${expected_bms_can_serial}"
  if [[ "${authorization}" != "preauthorized" ]]; then
    confirm_start_authorization
  fi
  source_runtime_environment
  launch_native
  if ! wait_for_enable_service; then
    terminate_failed_start
    fail "native stack did not expose /rt/enable within 90 seconds; stop was requested"
  fi
  info "READY: native stack is running in Domain ${expected_ros_domain_id}; drives were not enabled"
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

doctor_native()
{
  verify_target_identity
  verify_realtime_host
  verify_workspace
  reject_running_container
  source_runtime_environment
  runtime_env ros2 pkg prefix rt_control_bringup >/dev/null
  runtime_env ros2 pkg prefix enable_manager >/dev/null
  info "PASS: native runtime is installed for Domain ${expected_ros_domain_id} with Fast DDS default transports"
}

case "${1:-}" in
  doctor) doctor_native ;;
  start) start_native ;;
  start-and-enable) start_and_enable_native ;;
  enable) enable_native ;;
  stop) stop_native ;;
  status) status_native ;;
  logs) logs_native ;;
  -h|--help|help|"") usage ;;
  *) usage >&2; exit 2 ;;
esac
