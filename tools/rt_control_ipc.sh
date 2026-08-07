#!/usr/bin/env bash
set -euo pipefail

readonly expected_user="ar"
readonly expected_hostname="ar-Default-string"
readonly expected_kernel="5.15.0-1032-realtime"
readonly expected_cpuset="14"
readonly expected_housekeeping_cpuset="0,2,4,6,8,10,12,16-27"
readonly expected_container_cpuset="0,2,4,6,8,10,12,14,16-27"
readonly expected_ethercat_mac="8c:59:3c:14:ff:d3"
readonly expected_can_serial="004D00675230500720333159"
readonly expected_bms_can_serial="003000265230500720333159"
readonly runtime_sha="ff730e3bc51e726f95ffd402c1793b114e41733a"
readonly runtime_image="rt-control:${runtime_sha}"
readonly runtime_image_id="sha256:02ef48587b51bd4ab261f56e71c5274e4a95726f04587dd41176fcb08e7f4beb"
readonly runtime_root="/home/ar/rt-control-releases/${runtime_sha}/robot"
readonly compose_project="robot"
readonly container_name="robot-rt-control-1"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
compose_wrapper="${script_dir}/rt_control_compose.sh"
axis_state_checker="${script_dir}/rt_control_axis_state_check.py"
thread_affinity_tool="${script_dir}/rt_control_thread_affinity.py"
can_setup_tool="${repository_root}/hostsetup/rt-control-can-names.sh"
internal_dynamic_joint_states_topic="/rt_internal_state_broadcaster/dynamic_joint_states"
temporary_directory=""
startup_started=0
startup_complete=0

info()
{
  printf '[rt-control] %s\n' "$*"
}

fail()
{
  printf '[rt-control] FAIL: %s\n' "$*" >&2
  exit 1
}

compose()
{
  sudo env \
    RT_CONTROL_CPUSET="${expected_container_cpuset}" \
    RT_CONTROL_START_CPUSET="${expected_housekeeping_cpuset}" \
    RT_CONTROL_IMAGE_TAG="${runtime_sha}" \
    RT_CONTROL_PROJECT_ROOT="${runtime_root}" \
    "${compose_wrapper}" --project-name "${compose_project}" "$@"
}

run_ros2_timeout()
{
  local seconds="$1"
  local command="$2"
  sudo timeout "${seconds}" docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source /opt/rt_control_ws/install/setup.bash && ${command}"
}

call_rt_service()
{
  local operation="$1"
  case "${operation}" in
    enable|disable|reset_fault) ;;
    *) fail "unsupported lifecycle operation: ${operation}" ;;
  esac
  if [[ "${operation}" == "enable" ]]; then
    pin_controller_update_thread
  fi
  run_ros2_timeout 40 \
    "ros2 service call /rt/${operation} rt_control_interfaces/srv/RtEnable '{}'"
}

cleanup_temporary_directory()
{
  if [[ -n "${temporary_directory}" && -d "${temporary_directory}" ]]; then
    rm -rf -- "${temporary_directory}"
    temporary_directory=""
  fi
}

on_start_exit()
{
  local exit_code=$?
  trap - EXIT INT TERM
  if (( startup_started != 0 && startup_complete == 0 )); then
    info "启动未完成，正在尝试整组失能并有序停止容器。"
    if controller_manager_running_in_container; then
      call_rt_service disable >/dev/null 2>&1 || true
      compose stop rt-control >/dev/null 2>&1 || true
    else
      print_first_boot_error
      terminate_container_launch_tree
    fi
  fi
  cleanup_temporary_directory
  exit "${exit_code}"
}

verify_target_identity()
{
  [[ "$(id -un)" == "${expected_user}" ]] ||
    fail "必须使用 ${expected_user} 账号运行。"
  [[ "$(hostname)" == "${expected_hostname}" ]] ||
    fail "主机不匹配；本命令只允许在 ${expected_hostname} 上运行。"
}

cpu_list_contains()
{
  local cpu_list="$1"
  local wanted_cpu="$2"
  local first
  local last
  local part
  local -a parts
  IFS=',' read -r -a parts <<< "${cpu_list}"
  for part in "${parts[@]}"; do
    if [[ "${part}" == *-* ]]; then
      first="${part%-*}"
      last="${part#*-}"
      if [[ "${first}" =~ ^[0-9]+$ && "${last}" =~ ^[0-9]+$ ]] &&
        (( wanted_cpu >= first && wanted_cpu <= last )); then
        return 0
      fi
    elif [[ "${part}" == "${wanted_cpu}" ]]; then
      return 0
    fi
  done
  return 1
}

verify_rt_host_configuration()
{
  local affinity
  local argument
  local irq_directory
  [[ "$(uname -r)" == "${expected_kernel}" ]] ||
    fail "实时内核不匹配，期望 ${expected_kernel}。"
  [[ -r /sys/kernel/realtime && "$(< /sys/kernel/realtime)" == "1" ]] ||
    fail "PREEMPT_RT 未启用。"
  [[ "$(< /sys/devices/system/cpu/isolated)" == "${expected_cpuset}" ]] ||
    fail "CPU ${expected_cpuset} 不是唯一隔离核。"
  [[ "$(< /sys/devices/system/cpu/nohz_full)" == "${expected_cpuset}" ]] ||
    fail "CPU ${expected_cpuset} 未处于 full-nohz。"
  grep -Eq '(^|,)15($|,)' /sys/devices/system/cpu/offline ||
    fail "CPU 14 的 SMT sibling CPU 15 未下线。"
  for argument in \
    nosmt \
    isolcpus=domain,managed_irq,14 \
    nohz_full=14 \
    rcu_nocbs=14 \
    irqaffinity=0-13,15-27
  do
    grep -qw -- "${argument}" /proc/cmdline || fail "缺少内核参数 ${argument}。"
  done
  for irq_directory in /proc/irq/[0-9]*; do
    affinity="$(cat -- "${irq_directory}/effective_affinity_list" 2>/dev/null || true)"
    if cpu_list_contains "${affinity}" "${expected_cpuset}"; then
      fail "IRQ ${irq_directory##*/} 的 affinity ${affinity} 可能包含 CPU 14。"
    fi
  done
}

verify_release_identity()
{
  [[ -x "${compose_wrapper}" ]] || fail "缺少 Compose 包装器 ${compose_wrapper}。"
  [[ -f "${axis_state_checker}" ]] || fail "缺少状态字检查器 ${axis_state_checker}。"
  [[ -f "${runtime_root}/docker/compose.yaml" ]] ||
    fail "缺少已验证运行副本 ${runtime_root}。"
  [[ "$(basename -- "$(dirname -- "${runtime_root}")")" == "${runtime_sha}" ]] ||
    fail "运行副本目录与锁定 SHA 不一致。"

  command -v docker >/dev/null 2>&1 || fail "缺少命令 docker。"
  command -v python3 >/dev/null 2>&1 || fail "缺少命令 python3。"
  python3 -c 'import yaml' >/dev/null 2>&1 || fail "缺少 Python yaml 模块。"
  sudo -v
}

verify_runtime_image()
{
  local observed_image_id
  observed_image_id="$(sudo docker image inspect "${runtime_image}" --format '{{.Id}}')" ||
    fail "缺少镜像 ${runtime_image}。"
  [[ "${observed_image_id}" == "${runtime_image_id}" ]] ||
    fail "镜像 ID 不匹配，拒绝启动未验收镜像。"
}

verify_start_dependencies()
{
  for command in ethercat candump ip udevadm systemctl timeout python3; do
    command -v "${command}" >/dev/null 2>&1 || fail "缺少命令 ${command}。"
  done
  [[ -x "${thread_affinity_tool}" ]] ||
    fail "缺少线程绑核工具 ${thread_affinity_tool}。"
  [[ -x "${can_setup_tool}" ]] ||
    fail "缺少 CAN 启动前检查工具 ${can_setup_tool}。"
  systemctl is-active --quiet docker.service containerd.service ethercat.service ||
    fail "Docker、containerd 或 EtherCAT systemd 服务未运行。"
}

prepare_can_interfaces()
{
  info "按固定 USB 序列号准备 can0/can1：500 kbit/s，队列 128，接口 UP。"
  sudo "${can_setup_tool}" --wait 30 --configure ||
    fail "CAN 启动前检查失败；请查看上方缺少哪只适配器。"
}

pin_controller_update_thread()
{
  sudo python3 "${thread_affinity_tool}" \
    --rt-cpu "${expected_cpuset}" \
    --housekeeping-cpus "${expected_housekeeping_cpuset}" \
    --required-rt-thread-name rtcan-master \
    --rt-priority 80 \
    --deadline 5
}

verify_idle_bus_inputs()
{
  local master_output
  local slave_output
  local can_output
  local bms_can_output
  local can_serial
  local bms_can_serial
  local capture_file
  local bms_capture_file
  local capture_result

  master_output="$(sudo ethercat master)"
  grep -Fq "Main: ${expected_ethercat_mac} (attached)" <<< "${master_output}" ||
    fail "EtherCAT 主站 MAC 不匹配。"
  grep -Fq 'Link: UP' <<< "${master_output}" || fail "EtherCAT 链路未 UP。"
  grep -Fq 'Slaves: 16' <<< "${master_output}" || fail "EtherCAT 从站数量不是 16。"
  grep -Fq 'Tx errors:   0' <<< "${master_output}" || fail "EtherCAT 主站存在 Tx error。"
  grep -Fq 'Phase: Idle' <<< "${master_output}" || fail "启动前 EtherCAT master 不是 Idle。"
  grep -Fq 'Active: no' <<< "${master_output}" || fail "启动前 EtherCAT master 已被其他进程占用。"

  slave_output="$(sudo ethercat slaves)"
  [[ "$(wc -l <<< "${slave_output}")" -eq 16 ]] || fail "EtherCAT 扫描结果不是 16 行。"
  if grep -Eq '[[:space:]](INIT|BOOT|SAFEOP)[[:space:]]' <<< "${slave_output}"; then
    fail "启动前存在 INIT、BOOT 或 SAFEOP 从站。"
  fi

  can_serial="$(udevadm info -q property -p /sys/class/net/can0 |
    sed -n 's/^ID_SERIAL_SHORT=//p')"
  [[ "${can_serial}" == "${expected_can_serial}" ]] || fail "can0 USB 序列号不匹配。"
  can_output="$(ip -details -statistics link show can0)"
  grep -Fq 'state UP' <<< "${can_output}" || fail "can0 未 UP。"
  grep -Fq 'can state ERROR-ACTIVE' <<< "${can_output}" || fail "can0 不是 ERROR-ACTIVE。"
  grep -Fq 'bitrate 500000' <<< "${can_output}" || fail "can0 不是 500 kbit/s。"
  grep -Eq 'bus-errors[[:space:]]+arbit-lost[[:space:]]+error-warn[[:space:]]+error-pass[[:space:]]+bus-off' \
    <<< "${can_output}" || fail "无法读取 can0 错误统计。"
  grep -Eq '^[[:space:]]*0[[:space:]]+0[[:space:]]+0[[:space:]]+0[[:space:]]+0[[:space:]]+0[[:space:]]+numtxqueues' \
    <<< "${can_output}" || fail "can0 存在 restarted、bus error、warning、passive 或 bus-off 计数。"

  temporary_directory="$(mktemp -d /tmp/rt-control-ipc.XXXXXX)"
  capture_file="${temporary_directory}/can-before-start.log"
  set +e
  timeout 4 candump -L can0 > "${capture_file}"
  capture_result=$?
  set -e
  [[ ${capture_result} -eq 0 || ${capture_result} -eq 124 ]] ||
    fail "无法被动读取 can0 心跳。"
  for cob_id in 702 703; do
    grep -Fq "can0 ${cob_id}#" "${capture_file}" || fail "未收到 0x${cob_id} 心跳。"
  done

  bms_can_serial="$(udevadm info -q property -p /sys/class/net/can1 |
    sed -n 's/^ID_SERIAL_SHORT=//p')"
  [[ "${bms_can_serial}" == "${expected_bms_can_serial}" ]] ||
    fail "can1 BMS USB 序列号不匹配。"
  bms_can_output="$(ip -details -statistics link show can1)"
  grep -Fq 'state UP' <<< "${bms_can_output}" || fail "can1 未 UP。"
  grep -Fq 'can state ERROR-ACTIVE' <<< "${bms_can_output}" ||
    fail "can1 不是 ERROR-ACTIVE。"
  grep -Fq 'bitrate 500000' <<< "${bms_can_output}" ||
    fail "can1 不是 500 kbit/s。"
  bms_capture_file="${temporary_directory}/bms-before-start.log"
  set +e
  timeout 6 candump -L can1 > "${bms_capture_file}"
  capture_result=$?
  set -e
  [[ ${capture_result} -eq 0 || ${capture_result} -eq 124 ]] ||
    fail "无法被动读取 can1 BMS 总线。"
  grep -Fq 'can1 3FC#' "${bms_capture_file}" || fail "can1 未收到 BMS 0x3FC 状态帧。"
}

confirm_hardware_authorization()
{
  local answer
  [[ -t 0 && -r /dev/tty ]] || fail "自动使能必须在现场交互终端运行。"
  printf '%s\n' \
    "本命令将启动真实 EtherCAT/CANopen，并自动调用 /rt/enable。" \
    "确认：实体急停可用；14 轴和履带运动范围内无人、无障碍；当前允许执行器上电。" \
    "输入 ENABLE_RT_CONTROL 继续：" > /dev/tty
  IFS= read -r answer < /dev/tty
  [[ "${answer}" == "ENABLE_RT_CONTROL" ]] || fail "未获得现场使能确认。"
}

confirm_power_loss_recovery_authorization()
{
  local answer
  [[ -t 0 && -r /dev/tty ]] || fail "掉电恢复必须在现场交互终端运行。"
  printf '%s\n' \
    "旧控制会话已停止，EtherCAT 主站已释放。" \
    "确认：主接触器已恢复；实体急停可用；14 轴和履带运动范围内无人、无障碍；当前允许复位并使能执行器。" \
    "本流程将只调用一次 /rt/reset_fault 和一次 /rt/enable，不会自动重试。" \
    "输入 RECOVER_RT_CONTROL 继续：" > /dev/tty
  IFS= read -r answer < /dev/tty
  [[ "${answer}" == "RECOVER_RT_CONTROL" ]] || fail "未获得现场掉电恢复确认。"
}

wait_for_enable_service()
{
  local deadline=$((SECONDS + 120))
  local container_state
  local controller_manager_seen=0
  while (( SECONDS < deadline )); do
    container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
    if [[ "${container_state}" == "exited" || "${container_state}" == "dead" ]]; then
      print_first_boot_error
      fail "rt-control 容器在就绪前退出，请运行日志命令。"
    fi
    if controller_manager_running_in_container; then
      controller_manager_seen=1
    elif (( controller_manager_seen != 0 )) || controller_manager_died_during_boot; then
      print_first_boot_error
      terminate_container_launch_tree
      fail "ros2_control_node 在启动阶段提前退出；已结束 launch/spawner，不继续等待 /rt/enable。"
    fi
    if [[ "${container_state}" == "running" ]] &&
      run_ros2_timeout 6 'ros2 service type /rt/enable' 2>/dev/null |
      grep -Fq 'rt_control_interfaces/srv/RtEnable' &&
      run_ros2_timeout 6 'ros2 service type /control/set_enabled' 2>/dev/null |
      grep -Fq 'robot_rt_control_interfaces/srv/SetControlEnabled'; then
      return
    fi
    sleep 2
  done
  print_first_boot_error
  terminate_container_launch_tree
  fail "120 秒内未发现 /rt/enable 和 /control/set_enabled。"
}

wait_for_enable_manager_reset_ready()
{
  local deadline=$((SECONDS + 120))
  local snapshot
  while (( SECONDS < deadline )); do
    snapshot="$(run_ros2_timeout 8 \
      "ros2 topic echo --once /diagnostics diagnostic_msgs/msg/DiagnosticArray --filter \"any(s.name == '/robot/rt_control/enable_manager' for s in m.status)\"" \
      2>/dev/null || true)"
    if grep -Fq 'value: FAILED' <<< "${snapshot}" ||
      grep -Fq 'value: restart_required' <<< "${snapshot}"; then
      if grep -Fq 'value: fault_requires_reset' <<< "${snapshot}"; then
        return
      fi
      printf '%s\n' "${snapshot}" >&2
      fail "enable_manager 以非 Fault-reset 阶段失败，拒绝用 reset 掩盖原始故障。"
    fi
    if grep -Fq 'value: IDLE' <<< "${snapshot}"; then
      return
    fi
    sleep 1
  done
  fail "120 秒内 enable_manager 未进入 IDLE 或 fault_requires_reset。"
}

strip_ansi()
{
  sed -E $'s/\033\[[0-9;]*[mK]//g'
}

controller_manager_running_in_container()
{
  local container_state
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "${container_state}" == "running" ]] || return 1
  sudo docker top "${container_name}" -eo pid,args 2>/dev/null |
    grep -Fq '/controller_manager/ros2_control_node'
}

controller_manager_died_during_boot()
{
  local container_state
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "${container_state}" == "running" ]] || return 1
  compose logs --no-color --tail=400 rt-control 2>/dev/null | strip_ansi | grep -Eq \
    'ros2_control_node-[0-9]+.*process has died|process\[ros2_control_node-[0-9]+\].*died|controller_manager.*(Segmentation fault|terminate called|Exception|exception|FATAL|ERROR)'
}

print_first_boot_error()
{
  local first_error=""
  first_error="$(compose logs --no-color --tail=1000 rt-control 2>/dev/null | strip_ansi |
    grep -m 1 -E '(\[ERROR\]|\[FATAL\]|process has died|Segmentation fault|terminate called|Exception|exception|Traceback|Failed|failed|abort|what\(\):|BOOT_ERROR)' || true)"
  if [[ -n "${first_error}" ]]; then
    info "first boot error: ${first_error}"
  else
    info "first boot error: 容器日志中未找到 ERROR/FATAL/进程退出标记。"
  fi
}

terminate_container_launch_tree()
{
  local container_state
  local deadline
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "${container_state}" == "running" ]] || return
  sudo docker exec "${container_name}" bash -lc '
    pkill -INT -f "ros2 launch rt_control_bringup rt_control.launch.py" || true
    pkill -TERM -f "controller_manager/ros2_control_node|controller_manager/spawner|robot_state_publisher|rt_diagnostics_node|control_enable_adapter|plc_node|bms_node" || true
  ' >/dev/null 2>&1 || true
  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
    [[ "${container_state}" != "running" ]] && return
    sleep 1
  done
  sudo docker kill "${container_name}" >/dev/null 2>&1 || true
}

verify_operational_ethercat()
{
  local master_output
  local slave_output
  local position
  master_output="$(sudo ethercat master)"
  grep -Fq 'Phase: Operation' <<< "${master_output}" || fail "EtherCAT master 未进入 Operation。"
  grep -Fq 'Active: yes' <<< "${master_output}" || fail "EtherCAT master 未激活。"
  grep -Fq 'Slaves: 16' <<< "${master_output}" || fail "EtherCAT 从站数量不是 16。"

  slave_output="$(sudo ethercat slaves)"
  for position in {0..12} 14 15; do
    grep -Eq "^[[:space:]]*${position}[[:space:]].*[[:space:]]OP[[:space:]]+\\+" \
      <<< "${slave_output}" || fail "EtherCAT position ${position} 未进入 OP。"
  done
  grep -Eq '^[[:space:]]*13[[:space:]].*[[:space:]]PREOP[[:space:]]+\+' \
    <<< "${slave_output}" || fail "观察 hub position 13 未保持 PREOP。"
}

verify_canopen_operational_heartbeats()
{
  local capture_file="${temporary_directory}/can-after-start.log"
  local capture_result
  set +e
  timeout 4 candump -L can0 > "${capture_file}"
  capture_result=$?
  set -e
  [[ ${capture_result} -eq 0 || ${capture_result} -eq 124 ]] ||
    fail "无法读取启动后的 CANopen 心跳。"
  for cob_id in 702 703; do
    grep -Fq "can0 ${cob_id}#05" "${capture_file}" ||
      fail "CANopen 0x${cob_id} 未报告 Operational 心跳。"
  done
}

verify_rt_io_ready()
{
  local plc_state
  local battery_state
  plc_state="$(run_ros2_timeout 15 'ros2 topic echo /plc/io_state --once')" ||
    fail "未收到 /plc/io_state。"
  grep -Fq 'connected: true' <<< "${plc_state}" || fail "PLC 未连接。"
  grep -Fq 'data_fresh: true' <<< "${plc_state}" || fail "PLC 状态不是新鲜数据。"

  battery_state="$(run_ros2_timeout 20 'ros2 topic echo /battery_state --once')" ||
    fail "未收到 /battery_state。"
  grep -Eq '^voltage: [0-9]' <<< "${battery_state}" || fail "BMS 电压无有效数值。"
  grep -Eq '^percentage: (0|1|0\.[0-9]+|1\.0+)$' <<< "${battery_state}" ||
    fail "BMS SOC 无有效数值。"
}

verify_controllers_before_enable()
{
  local deadline=$((SECONDS + 30))
  local controller_output
  controller_output=""
  while (( SECONDS < deadline )); do
    controller_output="$(run_ros2_timeout 8 'ros2 control list_controllers' 2>/dev/null | strip_ansi || true)"
    if grep -Eq '^joint_state_broadcaster[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^rt_internal_state_broadcaster[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^diff_drive_controller[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^enable_manager[[:space:]].*active' <<< "${controller_output}" &&
      grep -Eq '^whole_body_jtc[[:space:]].*inactive' <<< "${controller_output}"; then
      return
    fi
    sleep 1
  done
  printf '%s\n' "${controller_output}" >&2
  fail "30 秒内 controller 未达到使能前状态：public JSB/internal JSB/diff-drive/enable-manager active 且 whole_body_jtc inactive。"
}

check_axis_states()
{
  local expected="$1"
  local snapshot
  snapshot="$(run_ros2_timeout 15 \
    "ros2 topic echo --once ${internal_dynamic_joint_states_topic} control_msgs/msg/DynamicJointState")" ||
    fail "未收到 ${internal_dynamic_joint_states_topic}，无法确认 ${expected} 状态字。"
  if ! printf '%s\n' "${snapshot}" | python3 "${axis_state_checker}" --expected "${expected}"; then
    fail "14 轴 ${expected} 状态字合同不满足。"
  fi
}

wait_for_stopped_container_and_idle_master()
{
  local deadline=$((SECONDS + 20))
  local container_state
  local master_output
  while (( SECONDS < deadline )); do
    container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
    master_output="$(sudo ethercat master 2>/dev/null || true)"
    if [[ "${container_state}" != "running" ]] &&
      grep -Fq 'Phase: Idle' <<< "${master_output}" &&
      grep -Fq 'Active: no' <<< "${master_output}"; then
      return
    fi
    sleep 1
  done
  fail "20 秒内未确认旧容器退出且 EtherCAT master Idle/Inactive。"
}

start_rt_control()
{
  local existing_state
  local lost_frames_before
  local lost_frames_after
  local controller_output
  local enable_response

  trap on_start_exit EXIT INT TERM
  info "检查当前工控机、锁定镜像和现场总线。"
  verify_target_identity
  verify_rt_host_configuration
  verify_release_identity
  verify_runtime_image
  verify_start_dependencies

  existing_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  [[ "${existing_state}" != "running" ]] ||
    fail "${container_name} 已在运行；请使用 status，不要重复启动。"

  confirm_hardware_authorization
  prepare_can_interfaces
  verify_idle_bus_inputs
  lost_frames_before="$(sudo ethercat master | awk '/Lost frames:/{print $3; exit}')"

  info "启动锁定镜像并等待控制器就绪。"
  startup_started=1
  compose up -d --no-build rt-control
  wait_for_enable_service

  verify_controllers_before_enable
  verify_operational_ethercat
  verify_canopen_operational_heartbeats
  verify_rt_io_ready

  info "调用 /rt/enable，自动使能 14 个 EtherCAT 轴。"
  enable_response="$(call_rt_service enable)"
  printf '%s\n' "${enable_response}"
  grep -Fq 'ok=True' <<< "${enable_response}" || fail "/rt/enable 返回失败。"
  grep -Eq "stage='(success|already_enabled)'" <<< "${enable_response}" ||
    fail "/rt/enable 未返回成功阶段。"

  controller_output="$(run_ros2_timeout 15 'ros2 control list_controllers' | strip_ansi)"
  grep -Eq '^whole_body_jtc[[:space:]].*active' <<< "${controller_output}" ||
    fail "/rt/enable 后 whole_body_jtc 未 active。"
  verify_operational_ethercat
  run_ros2_timeout 10 'ros2 topic echo /joint_states --once' >/dev/null

  lost_frames_after="$(sudo ethercat master | awk '/Lost frames:/{print $3; exit}')"
  if [[ -n "${lost_frames_before}" && -n "${lost_frames_after}" ]] &&
    (( lost_frames_after > lost_frames_before )); then
    info "WARN: EtherCAT Lost frames ${lost_frames_before} -> ${lost_frames_after}；已记录但按现行裁决不自动停机。"
  fi

  startup_complete=1
  cleanup_temporary_directory
  trap - EXIT INT TERM
  printf '%s\n' \
    "" \
    "READY: rt-control 已启动并完成 /rt/enable。" \
    "启停: /control/set_enabled" \
    "FJT: /whole_body_jtc/follow_joint_trajectory" \
    "底盘: /cmd_vel_safe" \
    "状态: /joint_states  /wheel/odom  /diagnostics" \
    "IO: /plc/io_state  /battery_state"
}

recover_power_loss()
{
  local container_state
  local disable_response
  local reset_response
  local enable_response
  local controller_output

  trap on_start_exit EXIT INT TERM
  info "检查当前工控机和锁定运行版本。"
  verify_target_identity
  verify_rt_host_configuration
  verify_release_identity
  verify_runtime_image
  verify_start_dependencies

  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  if [[ "${container_state}" == "running" ]]; then
    if controller_manager_running_in_container; then
      info "对旧控制会话最佳努力调用 /rt/disable；掉电导致失败时只记录，不伪报成功。"
      if disable_response="$(call_rt_service disable)"; then
        printf '%s\n' "${disable_response}"
        if ! grep -Fq 'ok=True' <<< "${disable_response}"; then
          info "WARN: 旧会话 /rt/disable 未确认成功；继续销毁旧控制会话。"
        fi
      else
        info "WARN: 旧会话 /rt/disable 不可用；继续销毁旧控制会话。"
      fi
    else
      info "WARN: 旧会话 controller_manager 已退出；跳过 /rt/disable 并直接结束 launch/spawner。"
      print_first_boot_error
      terminate_container_launch_tree
    fi
  else
    info "旧 rt-control 容器未运行，无可调用的 /rt/disable。"
  fi

  info "停止旧容器并确认 EtherCAT 主站已释放。"
  compose stop rt-control
  wait_for_stopped_container_and_idle_master
  confirm_power_loss_recovery_authorization

  info "复电确认完成；重新核对现场总线后启动全新控制会话。"
  prepare_can_interfaces
  verify_idle_bus_inputs
  startup_started=1
  compose up -d --no-build rt-control
  wait_for_enable_service
  verify_controllers_before_enable
  verify_operational_ethercat
  verify_canopen_operational_heartbeats
  verify_rt_io_ready
  wait_for_enable_manager_reset_ready

  info "调用一次全组 /rt/reset_fault。"
  reset_response="$(call_rt_service reset_fault)"
  printf '%s\n' "${reset_response}"
  grep -Fq 'ok=True' <<< "${reset_response}" || fail "/rt/reset_fault 返回失败。"
  grep -Eq "stage='(success|already_clear)'" <<< "${reset_response}" ||
    fail "/rt/reset_fault 未返回 success/already_clear。"
  check_axis_states disabled

  info "调用一次 /rt/enable。"
  enable_response="$(call_rt_service enable)"
  printf '%s\n' "${enable_response}"
  grep -Fq 'ok=True' <<< "${enable_response}" || fail "/rt/enable 返回失败。"
  grep -Eq "stage='(success|already_enabled)'" <<< "${enable_response}" ||
    fail "/rt/enable 未返回成功阶段。"
  check_axis_states enabled

  controller_output="$(run_ros2_timeout 15 'ros2 control list_controllers' | strip_ansi)"
  grep -Eq '^whole_body_jtc[[:space:]].*active' <<< "${controller_output}" ||
    fail "/rt/enable 后 whole_body_jtc 未 active。"
  verify_operational_ethercat

  startup_complete=1
  cleanup_temporary_directory
  trap - EXIT INT TERM
  info "RECOVERED: 新控制会话已通过一次 reset、逐轴状态确认和一次 enable。"
}

status_rt_control()
{
  local container_state
  verify_target_identity
  verify_release_identity
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}} (exit={{.State.ExitCode}})' 2>/dev/null || true)"
  printf 'container: %s\n' "${container_state:-not-created}"
  sudo ethercat master
  ip -details -statistics link show can0
  ip -details -statistics link show can1
  if [[ "${container_state}" == running* ]]; then
    run_ros2_timeout 15 'ros2 control list_controllers' | strip_ansi
  fi
}

logs_rt_control()
{
  verify_target_identity
  verify_release_identity
  compose logs --tail=200 -f rt-control
}

stop_rt_control()
{
  local container_state
  local disable_response
  local final_master
  local final_slaves
  local deadline
  local disable_ok=0
  verify_target_identity
  verify_release_identity
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}}' 2>/dev/null || true)"
  if [[ "${container_state}" != "running" ]]; then
    info "rt-control 已停止。"
    return
  fi

  if ! controller_manager_running_in_container; then
    print_first_boot_error
    terminate_container_launch_tree
    fail "controller_manager 未运行；已结束 launch/spawner，未继续等待 /rt/disable。"
  fi

  info "调用 /rt/disable，然后走 PID 1 有序停止路径。"
  if disable_response="$(call_rt_service disable)"; then
    printf '%s\n' "${disable_response}"
    if grep -Fq 'ok=True' <<< "${disable_response}"; then
      disable_ok=1
    else
      info "WARN: /rt/disable 返回了失败结果；仍继续执行有序停止。"
    fi
  else
    info "WARN: 显式 /rt/disable 未返回；容器停止包装器仍会再次尝试有序失能。"
  fi
  compose stop rt-control
  container_state="$(sudo docker inspect "${container_name}" --format '{{.State.Status}} {{.State.ExitCode}}')"
  [[ "${container_state}" == "exited 0" ]] || fail "容器未以 exited 0 停止：${container_state}。"

  deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    final_master="$(sudo ethercat master)"
    final_slaves="$(sudo ethercat slaves)"
    if grep -Fq 'Phase: Idle' <<< "${final_master}" &&
      grep -Fq 'Active: no' <<< "${final_master}" &&
      [[ "$(grep -Ec '[[:space:]]PREOP[[:space:]]+\+' <<< "${final_slaves}")" -eq 16 ]]; then
      if (( disable_ok == 0 )); then
        fail "最终总线已安全回到 Idle/PREOP，但显式 /rt/disable 未成功；请保留日志排查。"
      fi
      info "STOPPED: rt-control 已有序停止，EtherCAT Idle/Inactive 且 16 个从站全 PREOP。"
      return
    fi
    sleep 1
  done
  fail "容器已停止，但 15 秒内未确认 EtherCAT Idle/Inactive 和 16 个 PREOP 从站。"
}

print_help()
{
  cat <<'EOF'
当前 ar@192.168.0.40 工控机专用命令：

  ./tools/rt_control_ipc.sh          检查、启动并自动调用 /rt/enable
  ./tools/rt_control_ipc.sh status   查看容器、EtherCAT、CANopen/BMS 和 controller 状态
  ./tools/rt_control_ipc.sh logs     持续查看最近 200 行日志
  ./tools/rt_control_ipc.sh stop     /rt/disable 后有序停止 rt-control
  ./tools/rt_control_ipc.sh recover-power-loss
                                      主接触器急停掉电后的交互式全新会话恢复

start 必须在现场交互终端输入 ENABLE_RT_CONTROL；不提供自动 fault reset。
recover-power-loss 必须输入 RECOVER_RT_CONTROL；只在该显式流程调用一次 fault reset。
EOF
}

case "${1:-start}" in
  start) start_rt_control ;;
  status) status_rt_control ;;
  logs) logs_rt_control ;;
  stop) stop_rt_control ;;
  recover-power-loss) recover_power_loss ;;
  help|-h|--help) print_help ;;
  *) print_help >&2; exit 2 ;;
esac
