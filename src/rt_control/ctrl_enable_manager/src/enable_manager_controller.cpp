#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <set>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "hw_semantic_components/feedback_age.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace ctrl_enable_manager
{

void EnableManagerController::writeTrace(std::int64_t now_ns, std::int64_t period_ns)
{
  // Real-time: fixed-size copy of relaxed loads; no allocation, lock or system call.
  if (!trace_.ok()) {
    return;
  }
  double * v = trace_.begin();
  const auto load = [](const auto & value) {
    return static_cast<double>(value.load(std::memory_order_relaxed));
  };
  std::size_t i = 0U;
  v[i++] = static_cast<double>(phase_.load(std::memory_order_relaxed));
  v[i++] = static_cast<double>(owner_.load(std::memory_order_relaxed));
  v[i++] = static_cast<double>(current_batch_);
  v[i++] = static_cast<double>(downward_stage_);
  v[i++] = static_cast<double>(current_control_mode_.load(std::memory_order_relaxed));
  v[i++] = stage_deadline_ns_ == 0 ? std::numeric_limits<double>::quiet_NaN()
                                   : static_cast<double>(stage_deadline_ns_ - now_ns) * 1e-6;
  v[i++] = static_cast<double>(last_failure_stage_.load(std::memory_order_relaxed));
  v[i++] = load(last_failed_batch_);
  v[i++] = load(last_failed_joint_);
  v[i++] = load(last_failed_status_word_);
  v[i++] = static_cast<double>(primary_failure_stage_);
  v[i++] = static_cast<double>(primary_failed_batch_);
  v[i++] = static_cast<double>(primary_failed_joint_);
  v[i++] = load(motion_fault_pending_);
  v[i++] = load(enable_request_);
  v[i++] = load(disable_request_);
  v[i++] = load(reset_request_);
  v[i++] = load(switch_in_progress_);
  v[i++] = load(restart_required_);
  v[i++] = static_cast<double>(state_sources_qualified_);
  v[i++] = load(enable_hardware_ready_);
  v[i++] = load(jtc_deactivate_failed_);
  trace_.commit(now_ns, period_ns);
}

controller_interface::CallbackReturn EnableManagerController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const std::size_t expected_state_count = state_interface_configuration().names.size();
  if (
    command_interfaces_.size() !=
      cia402_axes_.size() + handshake_devices_.size() * 2U + mode_switch_joint_names_.size() ||
    state_interfaces_.size() != expected_state_count) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Unexpected enable-manager command or state interface count");
    return controller_interface::CallbackReturn::ERROR;
  }

  bool interfaces_bound = true;
  for (auto & axis : cia402_axes_) {
    const bool command_bound = axis.assign_loaned_command_interfaces(command_interfaces_);
    const bool state_bound = axis.assign_loaned_state_interfaces(state_interfaces_);
    interfaces_bound = interfaces_bound && command_bound && state_bound;
  }
  for (auto & device : handshake_devices_) {
    interfaces_bound = interfaces_bound && device->bind(command_interfaces_, state_interfaces_);
  }
  if (!interfaces_bound || !bindDriveModeCommands()) {
    releaseCia402Interfaces();
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "CiA402 and handshake interfaces must match each configured joint and device exactly once");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!motion_joint_names_.empty()) {
    const std::size_t motion_offset = cia402_axes_.size();
    for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
      const std::string expected_position = motion_joint_names_[axis] + "/position";
      if (state_interfaces_[motion_offset + axis].get_name() != expected_position) {
        releaseCia402Interfaces();
        RCLCPP_ERROR(get_node()->get_logger(), "Unexpected motion state interface order");
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    if (
      state_interfaces_[motion_offset + motion_joint_names_.size()].get_name() !=
      "ethercat_domain/process_data_age_ms") {
      releaseCia402Interfaces();
      RCLCPP_ERROR(get_node()->get_logger(), "Missing EtherCAT feedback-age interface");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for (std::size_t source = 0; source < ready_state_names_.size(); ++source) {
    std::size_t count = 0;
    for (std::size_t index = 0; index < state_interfaces_.size(); ++index) {
      if (state_interfaces_[index].get_name() == ready_state_names_[source]) {
        ready_state_indices_[source] = index;
        ++count;
      }
    }
    if (count != 1) {
      releaseCia402Interfaces();
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected exactly one ready state %s",
        ready_state_names_[source].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (std::size_t axis = 0; axis < expected_modes_.size(); ++axis) {
    const auto name = joint_names_[axis] + "/mode_of_operation_display";
    std::size_t count = 0;
    for (std::size_t index = 0; index < state_interfaces_.size(); ++index) {
      if (state_interfaces_[index].get_name() == name) {
        mode_state_indices_[axis] = index;
        ++count;
      }
    }
    if (count != 1) {
      releaseCia402Interfaces();
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected exactly one mode display for %s",
        joint_names_[axis].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for (std::size_t source = 0; source < diagnostic_state_names_.size(); ++source) {
    const auto match =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto & interface) {
        return interface.get_name() == diagnostic_state_names_[source];
      });
    if (match == state_interfaces_.end()) {
      releaseCia402Interfaces();
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing diagnostic state %s",
        diagnostic_state_names_[source].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    diagnostic_state_indices_[source] =
      static_cast<std::size_t>(std::distance(state_interfaces_.begin(), match));
  }
  for (std::size_t source = 0; source < offline_source_names_.size(); ++source) {
    const auto match =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto & interface) {
        return interface.get_name() == offline_source_names_[source];
      });
    if (match == state_interfaces_.end()) {
      releaseCia402Interfaces();
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing offline source state %s",
        offline_source_names_[source].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    offline_source_indices_[source] =
      static_cast<std::size_t>(std::distance(state_interfaces_.begin(), match));
  }
  offline_feedback_.store(-1, std::memory_order_relaxed);
  offline_now_.store(-1, std::memory_order_relaxed);
  for (auto & device : handshake_devices_) {
    if (!device->baseline()) {
      releaseCia402Interfaces();
      RCLCPP_ERROR(
        get_node()->get_logger(), "Handshake device %s reports an invalid state",
        device->name().c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  device_stage_ = false;

  setAllControlWords(0x0000U);
  enable_request_.store(false, std::memory_order_release);
  disable_request_.store(false, std::memory_order_release);
  reset_request_.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  jtc_activate_failed_request_.store(false, std::memory_order_release);
  jtc_activated_request_.store(false, std::memory_order_release);
  emergency_jtc_deactivate_request_.store(false, std::memory_order_release);
  jtc_deactivation_required_.store(false, std::memory_order_release);
  restart_required_.store(false, std::memory_order_release);
  mode_callback_active_.store(false, std::memory_order_release);
  mode_abort_requested_.store(false, std::memory_order_release);
  switch_in_progress_.store(false, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
    jtc_command_snapshot_ = CommandSnapshot{};
    rolling_command_snapshot_ = CommandSnapshot{};
  }
  actual_sample_version_.store(0U, std::memory_order_relaxed);
  actual_sample_sequence_.store(0U, std::memory_order_relaxed);
  actual_sample_time_ns_.store(0U, std::memory_order_relaxed);
  if (!motion_joint_names_.empty()) {
    const std::size_t motion_offset = cia402_axes_.size();
    for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
      actual_sample_positions_[axis].store(
        state_interfaces_[motion_offset + axis].get_value(), std::memory_order_relaxed);
    }
    actual_sample_feedback_age_ms_.store(
      state_interfaces_[motion_offset + motion_joint_names_.size()].get_value(),
      std::memory_order_relaxed);
  }
  clearFailure();
  current_batch_ = 0U;
  downward_stage_ = 0U;
  stage_deadline_ns_ = 0;
  enable_preempt_requested_ = false;
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  primary_failure_stage_ = Stage::kSuccess;
  primary_failed_batch_ = -1;
  primary_failed_joint_ = -1;
  primary_failed_status_word_ = 0U;
  owner_.store(Owner::kInternal, std::memory_order_release);
  phase_.store(Phase::kStartupSanitizing, std::memory_order_release);
  active_.store(true, std::memory_order_release);
  trace_.set_state(runtime_log::WriterState::kActive);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EnableManagerController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  mode_abort_requested_.store(true, std::memory_order_release);
  setAllControlWords(0x0000U);
  for (auto & device : handshake_devices_) {
    device->request_enable(false);
    device->write();
  }
  phase_.store(Phase::kInactive, std::memory_order_release);
  owner_.store(Owner::kInternal, std::memory_order_release);
  current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
  releaseCia402Interfaces();
  trace_.set_state(runtime_log::WriterState::kInactive);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EnableManagerController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }

  refreshStatusWords();
  refreshModes();
  for (auto & device : handshake_devices_) {
    device->read();
  }
  for (std::size_t index = 0; index < diagnostic_state_names_.size(); ++index) {
    diagnostic_state_values_[index].store(
      state_interfaces_[diagnostic_state_indices_[index]].get_value(), std::memory_order_relaxed);
  }
  if (!motion_joint_names_.empty()) {
    const std::size_t motion_offset = cia402_axes_.size();
    actual_sample_version_.fetch_add(1U, std::memory_order_acq_rel);
    for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
      actual_sample_positions_[axis].store(
        state_interfaces_[motion_offset + axis].get_value(), std::memory_order_relaxed);
    }
    actual_sample_feedback_age_ms_.store(
      state_interfaces_[motion_offset + motion_joint_names_.size()].get_value(),
      std::memory_order_relaxed);
    actual_sample_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);
    actual_sample_sequence_.fetch_add(1U, std::memory_order_relaxed);
    actual_sample_version_.fetch_add(1U, std::memory_order_release);
  }
  const std::int64_t now_ns = time.nanoseconds();
  Phase phase = phase_.load(std::memory_order_acquire);
  const int offline = offlineSource();
  offline_now_.store(offline, std::memory_order_relaxed);

  if (
    phase == Phase::kEnabled || phase == Phase::kJtcActivating ||
    phase == Phase::kJtcDeactivating) {
    // A drive that lost power keeps its last status word; only its stale feedback or its slave
    // leaving OP shows it. A re-powered slave may not resynchronise in OP: restart the stack.
    if (offline >= 0) {
      offline_feedback_.store(offline, std::memory_order_relaxed);
      restart_required_.store(true, std::memory_order_release);
      startEmergency(Stage::kDeviceOffline, -1, 0U, now_ns);
      phase = Phase::kEmergencyQuickStop;
    }
    for (std::size_t axis = 0; phase != Phase::kEmergencyQuickStop && axis < cia402_axes_.size();
         ++axis) {
      const DriveState state = decodeState(status_words_[axis]);
      if (isFaultState(state)) {
        startEmergency(
          Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
      if (state != DriveState::kOperationEnabled) {
        startEmergency(
          Stage::kUnexpectedDriveState, static_cast<std::int8_t>(axis), status_words_[axis],
          now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
      if (!runningModeConfirmed(axis)) {
        startEmergency(
          Stage::kModeMismatch, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
        phase = Phase::kEmergencyQuickStop;
        break;
      }
    }
    if (phase != Phase::kEmergencyQuickStop && !allDevicesEnabled()) {
      startEmergency(
        anyDeviceFaulted() ? Stage::kFaultDetected : Stage::kUnexpectedDriveState, -1, 0U, now_ns);
      phase = Phase::kEmergencyQuickStop;
    }
  }

  switch (phase) {
    case Phase::kStartupSanitizing:
      if (stage_deadline_ns_ == 0) {
        startDownward(Phase::kStartupSanitizing, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kIdle:
      setAllControlWords(0x0000U);
      requestDevicesEnable(false);
      if (enable_request_.exchange(false, std::memory_order_acq_rel)) {
        if (rejectIfOffline(enable_result_)) {
          break;
        }
        current_batch_ = 0U;
        // A device-only manager has no CiA402 batch and starts with the device stage.
        device_stage_ = enable_batches_.empty();
        ready_since_ns_ = -1;
        state_sources_qualified_ = ready_state_indices_.empty();
        enable_preempt_requested_ = false;
        primary_failure_stage_ = Stage::kSuccess;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        enable_hardware_ready_.store(false, std::memory_order_release);
        jtc_activated_request_.store(false, std::memory_order_release);
        jtc_activate_failed_request_.store(false, std::memory_order_release);
        std::fill(mode_confirmed_.begin(), mode_confirmed_.end(), 0U);
        stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
        phase_.store(Phase::kEnabling, std::memory_order_release);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        if (rejectIfOffline(reset_result_)) {
          break;
        }
        std::fill(reset_targets_.begin(), reset_targets_.end(), 0U);
        bool has_fault = anyDeviceFaulted();
        for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          startDeviceResets();
          setAllControlWords(0x0000U);
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      } else if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        clearFailure();
        publishResult(disable_result_, true, Stage::kAlreadyDisabled);
        owner_.store(Owner::kNone, std::memory_order_release);
      }
      break;
    case Phase::kEnabling:
    case Phase::kInterBatchDelay:
      updateEnable(now_ns);
      break;
    case Phase::kJtcActivating:
      if (jtc_activate_failed_request_.exchange(false, std::memory_order_acq_rel)) {
        primary_failure_stage_ = Stage::kJtcActivateFailed;
        primary_failed_batch_ = -1;
        primary_failed_joint_ = -1;
        primary_failed_status_word_ = 0U;
        recordFailure(Stage::kJtcActivateFailed, -1, -1, 0U);
        startDownward(Phase::kRollback, now_ns);
        break;
      }
      setAllControlWords(0x000FU);
      if (jtc_activated_request_.exchange(false, std::memory_order_acq_rel)) {
        // Commit success on the RT update only after this cycle's state/mode checks.
        clearFailure();
        if (motion_mode_switching_) {
          current_control_mode_.store(motion_controller_mode_, std::memory_order_release);
        }
        phase_.store(Phase::kEnabled, std::memory_order_release);
        publishResult(enable_result_, true, Stage::kSuccess);
        owner_.store(Owner::kNone, std::memory_order_release);
      }
      break;
    case Phase::kJtcDeactivating:
    case Phase::kEnabled:
      setAllControlWords(0x000FU);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        if (motion_fault_pending_.exchange(false, std::memory_order_acq_rel)) {
          primary_failure_stage_ = Stage::kMotionFault;
          primary_failed_batch_ = -1;
          primary_failed_joint_ = -1;
          primary_failed_status_word_ = 0U;
          recordFailure(Stage::kMotionFault, -1, -1, 0U);
        }
        startDownward(Phase::kDisabling, now_ns);
      }
      break;
    case Phase::kDisabling:
    case Phase::kRollback:
    case Phase::kResetDisabling:
    case Phase::kEmergencyDisable:
      if (stage_deadline_ns_ == 0) {
        startDownward(phase, now_ns);
      }
      updateDownward(now_ns);
      break;
    case Phase::kResetLow:
    case Phase::kResetHigh:
      updateReset(now_ns);
      break;
    case Phase::kEmergencyQuickStop:
      updateEmergencyQuickStop(now_ns);
      break;
    case Phase::kFailed:
      setAllControlWords(0x0000U);
      requestDevicesEnable(false);
      if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
        startDownward(Phase::kDisabling, now_ns);
      } else if (reset_request_.exchange(false, std::memory_order_acq_rel)) {
        if (rejectIfOffline(reset_result_)) {
          break;
        }
        std::fill(reset_targets_.begin(), reset_targets_.end(), 0U);
        bool has_fault = anyDeviceFaulted();
        for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
          reset_targets_[axis] = isFaultState(decodeState(status_words_[axis]));
          has_fault = has_fault || reset_targets_[axis];
        }
        if (!has_fault) {
          clearFailure();
          publishResult(reset_result_, true, Stage::kAlreadyClear);
          phase_.store(Phase::kIdle, std::memory_order_release);
          owner_.store(Owner::kNone, std::memory_order_release);
        } else {
          startDeviceResets();
          phase_.store(Phase::kResetLow, std::memory_order_release);
        }
      }
      break;
    case Phase::kInactive:
      break;
  }

  for (auto & device : handshake_devices_) {
    device->write();
  }
  writeDriveModes();
  writeTrace(now_ns, period.nanoseconds());
  return controller_interface::return_type::OK;
}

void EnableManagerController::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.name = "/robot/rt_control/enable_manager";
  status.hardware_id = "robot-001";
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kFailed || restart_required_.load(std::memory_order_acquire)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "failed";
  } else if (phase != Phase::kIdle && phase != Phase::kEnabled && phase != Phase::kInactive) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "operation in progress";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "operational";
  }
  const std::int8_t failed_joint = last_failed_joint_.load(std::memory_order_acquire);
  status.add("state", phaseName(phase));
  status.add("failed_batch", std::to_string(last_failed_batch_.load(std::memory_order_acquire)));
  status.add("failed_joint", failed_joint >= 0 ? joint_names_[failed_joint] : "");
  status.add(
    "failed_status_word", std::to_string(last_failed_status_word_.load(std::memory_order_acquire)));
  status.add("stage", stageName(last_failure_stage_.load(std::memory_order_acquire)));
  const int offline = offline_feedback_.load(std::memory_order_relaxed);
  status.add("offline_feedback", offline >= 0 ? offline_source_names_[offline] : "");
  const int offline_now = offline_now_.load(std::memory_order_relaxed);
  status.add("offline_now", offline_now >= 0 ? offline_source_names_[offline_now] : "");
  status.add(
    "restart_required", restart_required_.load(std::memory_order_acquire) ? "true" : "false");
  const auto actual_mode = last_failed_mode_.load(std::memory_order_acquire);
  status.add(
    "failed_actual_mode", actual_mode == 256 ? "unavailable" : std::to_string(actual_mode));
  status.add(
    "failed_expected_mode", std::to_string(last_expected_mode_.load(std::memory_order_acquire)));
  for (const auto & device : handshake_devices_) {
    const auto device_status = device->status();
    const bool device_fault = !device_status.valid || device_status.fault_latched ||
                              device_status.phase == HandshakeDevice::Phase::fault_latched;
    if (device_fault) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = device_status.valid ? "device fault latched" : "device state invalid";
    }
    const std::string prefix = device->name() + ".";
    status.add(prefix + "phase", std::to_string(static_cast<int>(device_status.phase)));
    status.add(prefix + "fault_latched", device_status.fault_latched ? "true" : "false");
    status.add(prefix + "motion_allowed", device_status.motion_allowed ? "true" : "false");
  }
  for (std::size_t index = 0; index < diagnostic_state_names_.size(); ++index) {
    status.add(
      diagnostic_state_names_[index],
      std::to_string(diagnostic_state_values_[index].load(std::memory_order_relaxed)));
  }
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

void EnableManagerController::publishResult(
  ResultSlot & slot, bool ok, Stage stage, std::int8_t failed_batch, std::int8_t failed_joint,
  std::uint16_t status_word)
{
  slot.ok.store(ok, std::memory_order_relaxed);
  slot.failed_batch.store(failed_batch, std::memory_order_relaxed);
  slot.failed_joint.store(failed_joint, std::memory_order_relaxed);
  slot.status_word.store(status_word, std::memory_order_relaxed);
  slot.stage.store(stage, std::memory_order_relaxed);
  slot.ready.store(true, std::memory_order_release);
}

int EnableManagerController::offlineSource() const noexcept
{
  for (std::size_t source = 0; source < offline_source_indices_.size(); ++source) {
    const double value = state_interfaces_[offline_source_indices_[source]].get_value();
    const bool online = source < age_source_count_
                          ? hw_semantic_components::feedback_fresh(value, feedback_age_limit_ms_)
                          : value == kAlStateOp;
    if (!online) {
      return static_cast<int>(source);
    }
  }
  return -1;
}

// Rejects without changing phase or the recorded failure: the adapter reads the recorded
// stage to decide whether a reset is still worth trying once the devices are back.
bool EnableManagerController::rejectIfOffline(ResultSlot & slot)
{
  const int stale = offline_now_.load(std::memory_order_relaxed);
  offline_feedback_.store(stale, std::memory_order_relaxed);
  if (stale < 0) {
    return false;
  }
  publishResult(slot, false, Stage::kDeviceOffline);
  owner_.store(Owner::kNone, std::memory_order_release);
  return true;
}

void EnableManagerController::recordFailure(
  Stage stage, std::int8_t failed_batch, std::int8_t failed_joint, std::uint16_t status_word)
{
  last_failure_stage_.store(stage, std::memory_order_relaxed);
  last_failed_batch_.store(failed_batch, std::memory_order_relaxed);
  last_failed_joint_.store(failed_joint, std::memory_order_relaxed);
  last_failed_status_word_.store(status_word, std::memory_order_release);
  if (failed_joint >= 0 && static_cast<std::size_t>(failed_joint) < expected_modes_.size()) {
    last_failed_mode_.store(actual_modes_[failed_joint], std::memory_order_relaxed);
    last_expected_mode_.store(expected_modes_[failed_joint], std::memory_order_release);
  } else {
    last_failed_mode_.store(256, std::memory_order_relaxed);
    last_expected_mode_.store(0, std::memory_order_release);
  }
}

void EnableManagerController::clearFailure()
{
  last_failure_stage_.store(Stage::kSuccess, std::memory_order_relaxed);
  last_failed_batch_.store(-1, std::memory_order_relaxed);
  last_failed_joint_.store(-1, std::memory_order_relaxed);
  last_failed_status_word_.store(0U, std::memory_order_release);
  last_failed_mode_.store(256, std::memory_order_relaxed);
  last_expected_mode_.store(0, std::memory_order_release);
}

void EnableManagerController::startDownward(Phase phase, std::int64_t now_ns)
{
  downward_stage_ = 0U;
  stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  phase_.store(phase, std::memory_order_release);
}

void EnableManagerController::updateDownward(std::int64_t now_ns)
{
  const Phase active_phase = phase_.load(std::memory_order_acquire);
  bool any_operation_enabled = false;
  bool any_switched_on = false;
  bool all_nonfault_terminal = true;
  bool any_fault = false;
  std::int8_t first_nonterminal = -1;

  requestDevicesEnable(false);
  for (const auto & device : handshake_devices_) {
    any_fault = any_fault || device->faulted();
    all_nonfault_terminal = all_nonfault_terminal && (device->disabled() || device->faulted());
  }

  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    switch (state) {
      case DriveState::kOperationEnabled:
        command = 0x0007U;
        any_operation_enabled = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kSwitchedOn:
        command = 0x0006U;
        any_switched_on = true;
        all_nonfault_terminal = false;
        break;
      case DriveState::kReadyToSwitchOn:
        command = 0x0000U;
        all_nonfault_terminal = all_nonfault_terminal && isConfirmedDisableTerminal(axis, state);
        break;
      case DriveState::kQuickStopActive:
      case DriveState::kNotReadyToSwitchOn:
      case DriveState::kUnknown:
        command = 0x0000U;
        all_nonfault_terminal = false;
        break;
      case DriveState::kFaultReactionActive:
      case DriveState::kFault:
        any_fault = true;
        command = 0x0000U;
        break;
      case DriveState::kSwitchOnDisabled:
        command = 0x0000U;
        break;
    }
    if (!isConfirmedDisableTerminal(axis, state) && !isFaultState(state) && first_nonterminal < 0) {
      first_nonterminal = static_cast<std::int8_t>(axis);
    }
    setControlWord(axis, command);
  }

  bool advance_stage = false;
  if (downward_stage_ == 0U) {
    advance_stage = !any_operation_enabled;
  } else if (downward_stage_ == 1U) {
    advance_stage = !any_operation_enabled && !any_switched_on;
  } else if (all_nonfault_terminal) {
    finishDownward();
    return;
  }

  if (advance_stage) {
    ++downward_stage_;
    stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
    if (downward_stage_ > 2U && all_nonfault_terminal) {
      finishDownward();
    }
    return;
  }

  if (now_ns >= stage_deadline_ns_) {
    if (first_nonterminal < 0 && any_fault) {
      for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
        if (isFaultState(decodeState(status_words_[axis]))) {
          first_nonterminal = static_cast<std::int8_t>(axis);
          break;
        }
      }
    }
    if (active_phase == Phase::kResetDisabling) {
      const std::uint16_t status = first_nonterminal >= 0 ? status_words_[first_nonterminal] : 0U;
      recordFailure(Stage::kFaultResetTimeout, -1, first_nonterminal, status);
      publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, first_nonterminal, status);
      setAllControlWords(0x0000U);
      phase_.store(Phase::kFailed, std::memory_order_release);
      owner_.store(Owner::kNone, std::memory_order_release);
      return;
    }
    if (primary_failure_stage_ == Stage::kSuccess) {
      primary_failure_stage_ = any_fault ? Stage::kFaultRequiresReset : Stage::kDisableTimeout;
      primary_failed_joint_ = first_nonterminal;
      primary_failed_status_word_ = first_nonterminal >= 0 ? status_words_[first_nonterminal] : 0U;
      recordFailure(primary_failure_stage_, -1, primary_failed_joint_, primary_failed_status_word_);
    }
    if (phase_.load(std::memory_order_acquire) != Phase::kEmergencyDisable) {
      interrupted_owner_ = owner_.load(std::memory_order_acquire);
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
      phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
    } else {
      finishDownward();
    }
  }
}

void EnableManagerController::startEmergency(
  Stage stage, std::int8_t failed_joint, std::uint16_t status_word, std::int64_t now_ns)
{
  interrupted_owner_ = owner_.load(std::memory_order_acquire);
  primary_failure_stage_ = stage;
  primary_failed_batch_ =
    interrupted_owner_ == Owner::kEnable ? static_cast<std::int8_t>(current_batch_) : -1;
  primary_failed_joint_ = failed_joint;
  primary_failed_status_word_ = status_word;
  recordFailure(stage, primary_failed_batch_, failed_joint, status_word);
  owner_.store(Owner::kInternal, std::memory_order_release);
  stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(disable_stage_timeout_seconds_ * 1e9);
  emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
  phase_.store(Phase::kEmergencyQuickStop, std::memory_order_release);
}

void EnableManagerController::updateEmergencyQuickStop(std::int64_t now_ns)
{
  requestDevicesEnable(false);
  bool any_operation_enabled = false;
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (state == DriveState::kOperationEnabled || state == DriveState::kQuickStopActive) {
      command = 0x0002U;
    } else if (state == DriveState::kSwitchedOn) {
      command = 0x0006U;
    }
    any_operation_enabled = any_operation_enabled || state == DriveState::kOperationEnabled;
    setControlWord(axis, command);
  }
  if (!any_operation_enabled || now_ns >= stage_deadline_ns_) {
    startDownward(Phase::kEmergencyDisable, now_ns);
  }
}

void EnableManagerController::finishDownward()
{
  const Phase completed_phase = phase_.load(std::memory_order_acquire);
  bool any_fault = anyDeviceFaulted();
  bool all_terminal = allDevicesDisabledOrFaulted();
  std::int8_t first_fault = -1;
  std::int8_t first_nonterminal = -1;
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    if (isFaultState(state)) {
      any_fault = true;
      if (first_fault < 0) {
        first_fault = static_cast<std::int8_t>(axis);
      }
    } else if (!isConfirmedDisableTerminal(axis, state)) {
      all_terminal = false;
      if (first_nonterminal < 0) {
        first_nonterminal = static_cast<std::int8_t>(axis);
      }
    }
  }

  // A faulted handshake device has no CiA402 index or status word.
  const std::uint16_t first_fault_status = first_fault >= 0 ? status_words_[first_fault] : 0U;

  if (completed_phase == Phase::kStartupSanitizing) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (primary_failure_stage_ != Stage::kSuccess) {
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (any_fault) {
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, first_fault_status);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kDisabling) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (any_fault) {
      publishResult(
        disable_result_, false, Stage::kFaultRequiresReset, -1, first_fault, first_fault_status);
      recordFailure(Stage::kFaultRequiresReset, -1, first_fault, first_fault_status);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (jtc_deactivate_failed_.load(std::memory_order_acquire)) {
      publishResult(disable_result_, false, Stage::kJtcDeactivateFailed);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else if (primary_failure_stage_ != Stage::kSuccess) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      clearFailure();
      publishResult(disable_result_, true, Stage::kSuccess);
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kRollback) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    publishResult(
      enable_result_, false, primary_failure_stage_, primary_failed_batch_, primary_failed_joint_,
      primary_failed_status_word_);
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(any_fault || restart ? Phase::kFailed : Phase::kIdle, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kResetDisabling) {
    current_control_mode_.store(ControlMode::kDisabled, std::memory_order_release);
    if (any_fault || !all_terminal) {
      const std::int8_t failed_joint = first_fault >= 0 ? first_fault : first_nonterminal;
      const std::uint16_t status_word = failed_joint >= 0 ? status_words_[failed_joint] : 0U;
      publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, failed_joint, status_word);
      recordFailure(Stage::kFaultResetTimeout, -1, failed_joint, status_word);
      phase_.store(Phase::kFailed, std::memory_order_release);
    } else {
      clearFailure();
      publishResult(reset_result_, true, Stage::kSuccess);
      phase_.store(Phase::kIdle, std::memory_order_release);
    }
    owner_.store(Owner::kNone, std::memory_order_release);
    return;
  }

  if (completed_phase == Phase::kEmergencyDisable) {
    current_control_mode_.store(
      restart_required_.load(std::memory_order_acquire) ? ControlMode::kRestartRequired
                                                        : ControlMode::kDisabled,
      std::memory_order_release);
    if (interrupted_owner_ == Owner::kEnable) {
      publishResult(
        enable_result_, false, primary_failure_stage_, primary_failed_batch_, primary_failed_joint_,
        primary_failed_status_word_);
    } else if (interrupted_owner_ == Owner::kDisable) {
      publishResult(
        disable_result_, false, primary_failure_stage_, primary_failed_batch_,
        primary_failed_joint_, primary_failed_status_word_);
    }
    const bool restart = restart_required_.load(std::memory_order_acquire);
    phase_.store(
      any_fault || !all_terminal || restart ? Phase::kFailed : Phase::kIdle,
      std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

bool EnableManagerController::stateSourcesReady() const noexcept
{
  for (const auto index : ready_state_indices_) {
    if (state_interfaces_[index].get_value() != 1.0) {
      return false;
    }
  }
  return true;
}

void EnableManagerController::failEnable(Stage stage, std::ptrdiff_t axis, std::int64_t now_ns)
{
  primary_failure_stage_ = stage;
  primary_failed_batch_ = static_cast<std::int8_t>(current_batch_);
  primary_failed_joint_ = static_cast<std::int8_t>(axis);
  primary_failed_status_word_ = axis >= 0 ? status_words_[static_cast<std::size_t>(axis)] : 0U;
  recordFailure(stage, primary_failed_batch_, primary_failed_joint_, primary_failed_status_word_);
  if (enable_preempt_requested_) {
    publishResult(
      enable_result_, false, stage, primary_failed_batch_, primary_failed_joint_,
      primary_failed_status_word_);
    owner_.store(Owner::kDisable, std::memory_order_release);
    disable_request_.store(false, std::memory_order_release);
    startDownward(Phase::kDisabling, now_ns);
  } else {
    startDownward(Phase::kRollback, now_ns);
  }
  // Failure outputs begin in this cycle, not one update after the failure.
  updateDownward(now_ns);
}

void EnableManagerController::updateEnable(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.load(std::memory_order_acquire)) {
    enable_preempt_requested_ = true;
  }

  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    if (isFaultState(decodeState(status_words_[axis]))) {
      disable_request_.store(false, std::memory_order_release);
      startEmergency(
        Stage::kFaultDetected, static_cast<std::int8_t>(axis), status_words_[axis], now_ns);
      updateEmergencyQuickStop(now_ns);
      return;
    }
  }
  if (anyDeviceFaulted()) {
    disable_request_.store(false, std::memory_order_release);
    startEmergency(Stage::kFaultDetected, -1, 0U, now_ns);
    updateEmergencyQuickStop(now_ns);
    return;
  }
  // A confirmed individual axis cannot go back to pending while another axis
  // is still enabling, nor during the inter-batch delay.
  for (std::size_t axis = 0; axis < mode_confirmed_.size(); ++axis) {
    if (!mode_confirmed_[axis]) {
      continue;
    }
    if (decodeState(status_words_[axis]) != DriveState::kOperationEnabled) {
      failEnable(Stage::kUnexpectedDriveState, axis, now_ns);
      return;
    }
    if (!modeConfirmed(axis)) {
      failEnable(Stage::kModeMismatch, axis, now_ns);
      return;
    }
  }

  // Lifecycle "active" is not readiness: hold all drives disabled until every required
  // state source has been ready for ready_hold_time, then restart the batch deadline.
  if (!ready_state_indices_.empty()) {
    if (!stateSourcesReady()) {
      if (state_sources_qualified_) {
        failEnable(Stage::kStateSourcesNotReady, -1, now_ns);
        return;
      }
      ready_since_ns_ = -1;
    } else if (!state_sources_qualified_) {
      if (ready_since_ns_ < 0) {
        ready_since_ns_ = now_ns;
      }
      if (now_ns - ready_since_ns_ >= static_cast<std::int64_t>(ready_hold_seconds_ * 1e9)) {
        state_sources_qualified_ = true;
        stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
      }
    }
    if (!state_sources_qualified_) {
      setAllControlWords(0x0000U);
      if (enable_preempt_requested_ || now_ns >= stage_deadline_ns_) {
        failEnable(Stage::kStateSourcesNotReady, -1, now_ns);
      }
      return;
    }
  }

  if (phase == Phase::kInterBatchDelay) {
    for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
      setControlWord(
        axis,
        decodeState(status_words_[axis]) == DriveState::kOperationEnabled ? 0x000FU : 0x0000U);
    }
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (now_ns >= stage_deadline_ns_) {
      ++current_batch_;
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
      phase_.store(Phase::kEnabling, std::memory_order_release);
    }
    return;
  }

  if (device_stage_) {
    updateDeviceEnable(now_ns);
    return;
  }

  bool batch_complete = true;
  std::int8_t invalid_axis = -1;
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    bool is_current = false;
    bool is_previous = false;
    for (std::size_t batch = 0; batch <= current_batch_; ++batch) {
      for (const auto batch_axis : enable_batches_[batch]) {
        if (batch_axis == axis) {
          is_current = batch == current_batch_;
          is_previous = batch < current_batch_;
        }
      }
    }
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (is_previous) {
      command = 0x000FU;
      if (state != DriveState::kOperationEnabled && invalid_axis < 0) {
        invalid_axis = static_cast<std::int8_t>(axis);
      }
    } else if (is_current) {
      if (!expected_modes_.empty() && !modeConfirmed(axis) && !modePending(axis)) {
        failEnable(Stage::kModeMismatch, axis, now_ns);
        return;
      }
      switch (state) {
        case DriveState::kSwitchOnDisabled:
          command = 0x0006U;
          batch_complete = false;
          break;
        case DriveState::kReadyToSwitchOn:
          command = 0x0007U;
          batch_complete = false;
          break;
        case DriveState::kSwitchedOn:
          command = 0x000FU;
          batch_complete = false;
          break;
        case DriveState::kOperationEnabled:
          command = 0x000FU;
          if (!modeConfirmed(axis)) {
            batch_complete = false;
            if (!modePending(axis)) {
              failEnable(Stage::kModeMismatch, axis, now_ns);
              return;
            }
          } else if (!mode_confirmed_.empty()) {
            mode_confirmed_[axis] = 1U;
          }
          break;
        default:
          command = 0x0000U;
          batch_complete = false;
          if (invalid_axis < 0) {
            invalid_axis = static_cast<std::int8_t>(axis);
          }
          break;
      }
    }
    setControlWord(axis, command);
  }
  if (invalid_axis >= 0) {
    failEnable(Stage::kEnableInvalidState, invalid_axis, now_ns);
    return;
  }
  if (batch_complete) {
    if (enable_preempt_requested_) {
      disable_request_.store(false, std::memory_order_release);
      publishResult(enable_result_, false, Stage::kPreemptedByDisable);
      owner_.store(Owner::kDisable, std::memory_order_release);
      primary_failure_stage_ = Stage::kSuccess;
      startDownward(Phase::kDisabling, now_ns);
    } else if (current_batch_ + 1U == enable_batches_.size() && !handshake_devices_.empty()) {
      device_stage_ = true;
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(batch_timeout_seconds_ * 1e9);
    } else if (current_batch_ + 1U == enable_batches_.size()) {
      // Publish phase before signalling the service thread that hardware is ready.
      phase_.store(Phase::kJtcActivating, std::memory_order_release);
      enable_hardware_ready_.store(true, std::memory_order_release);
    } else {
      stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(inter_batch_delay_seconds_ * 1e9);
      phase_.store(Phase::kInterBatchDelay, std::memory_order_release);
    }
    return;
  }
  if (now_ns >= stage_deadline_ns_) {
    auto failed_axis = enable_batches_[current_batch_][0];
    for (const auto axis : enable_batches_[current_batch_]) {
      if (
        decodeState(status_words_[axis]) != DriveState::kOperationEnabled || !modeConfirmed(axis)) {
        failed_axis = axis;
        break;
      }
    }
    const auto stage = decodeState(status_words_[failed_axis]) == DriveState::kOperationEnabled &&
                           modePending(failed_axis)
                         ? Stage::kModeNotConfirmed
                         : Stage::kEnableBatchTimeout;
    failEnable(stage, failed_axis, now_ns);
  }
}

void EnableManagerController::updateReset(std::int64_t now_ns)
{
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (disable_request_.exchange(false, std::memory_order_acq_rel)) {
    setAllControlWords(0x0000U);
    publishResult(reset_result_, false, Stage::kPreemptedByDisable);
    owner_.store(Owner::kDisable, std::memory_order_release);
    primary_failure_stage_ = Stage::kSuccess;
    startDownward(Phase::kDisabling, now_ns);
    return;
  }

  if (phase == Phase::kResetLow) {
    setAllControlWords(0x0000U);
    stage_deadline_ns_ = now_ns + static_cast<std::int64_t>(fault_reset_timeout_seconds_ * 1e9);
    phase_.store(Phase::kResetHigh, std::memory_order_release);
    return;
  }

  bool all_reset_targets_left_fault = allDeviceResetsComplete();
  std::int8_t first_pending = -1;
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    const DriveState state = decodeState(status_words_[axis]);
    std::uint16_t command = 0x0000U;
    if (reset_targets_[axis] && state == DriveState::kFault) {
      command = 0x0080U;
    }
    setControlWord(axis, command);

    if (isFaultState(state)) {
      all_reset_targets_left_fault = false;
      if (first_pending < 0) {
        first_pending = static_cast<std::int8_t>(axis);
      }
      continue;
    }

    if (reset_targets_[axis]) {
      reset_targets_[axis] = false;
    }
  }

  if (all_reset_targets_left_fault) {
    startDownward(Phase::kResetDisabling, now_ns);
  } else if (now_ns >= stage_deadline_ns_) {
    const std::uint16_t status = first_pending >= 0 ? status_words_[first_pending] : 0U;
    recordFailure(Stage::kFaultResetTimeout, -1, first_pending, status);
    publishResult(reset_result_, false, Stage::kFaultResetTimeout, -1, first_pending, status);
    setAllControlWords(0x0000U);
    phase_.store(Phase::kFailed, std::memory_order_release);
    owner_.store(Owner::kNone, std::memory_order_release);
  }
}

void EnableManagerController::updateDeviceEnable(std::int64_t now_ns)
{
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    if (decodeState(status_words_[axis]) != DriveState::kOperationEnabled) {
      failEnable(Stage::kUnexpectedDriveState, static_cast<std::ptrdiff_t>(axis), now_ns);
      return;
    }
    setControlWord(axis, 0x000FU);
  }
  if (enable_preempt_requested_) {
    disable_request_.store(false, std::memory_order_release);
    publishResult(enable_result_, false, Stage::kPreemptedByDisable);
    owner_.store(Owner::kDisable, std::memory_order_release);
    primary_failure_stage_ = Stage::kSuccess;
    startDownward(Phase::kDisabling, now_ns);
    return;
  }
  requestDevicesEnable(true);
  if (allDevicesEnabled()) {
    device_stage_ = false;
    // Publish phase before signalling the service thread that hardware is ready.
    phase_.store(Phase::kJtcActivating, std::memory_order_release);
    enable_hardware_ready_.store(true, std::memory_order_release);
  } else if (now_ns >= stage_deadline_ns_) {
    failEnable(Stage::kEnableBatchTimeout, -1, now_ns);
  }
}

void EnableManagerController::requestDevicesEnable(bool enable) noexcept
{
  for (auto & device : handshake_devices_) {
    device->request_enable(enable);
  }
}

void EnableManagerController::startDeviceResets() noexcept
{
  for (auto & device : handshake_devices_) {
    if (device->faulted()) {
      device->start_reset();
    }
  }
}

bool EnableManagerController::anyDeviceFaulted() const noexcept
{
  return std::any_of(handshake_devices_.begin(), handshake_devices_.end(), [](const auto & device) {
    return device->faulted();
  });
}

bool EnableManagerController::allDevicesEnabled() const noexcept
{
  return std::all_of(handshake_devices_.begin(), handshake_devices_.end(), [](const auto & device) {
    return device->enabled();
  });
}

bool EnableManagerController::allDevicesDisabledOrFaulted() const noexcept
{
  return std::all_of(handshake_devices_.begin(), handshake_devices_.end(), [](const auto & device) {
    return device->disabled() || device->faulted();
  });
}

bool EnableManagerController::allDeviceResetsComplete() const noexcept
{
  return std::all_of(handshake_devices_.begin(), handshake_devices_.end(), [](const auto & device) {
    return device->reset_complete();
  });
}

void EnableManagerController::setControlWord(std::size_t axis, std::uint16_t control_word)
{
  (void)cia402_axes_[axis].set_control_word(control_word);
}

void EnableManagerController::setAllControlWords(std::uint16_t control_word)
{
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    setControlWord(axis, control_word);
  }
}

void EnableManagerController::releaseCia402Interfaces() noexcept
{
  for (auto & axis : cia402_axes_) {
    axis.release_interfaces();
  }
  for (auto & device : handshake_devices_) {
    device->release();
  }
}

bool EnableManagerController::allAxesInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return false;
    }
  }
  return true;
}

std::int8_t EnableManagerController::firstAxisNotInState(DriveState state) const
{
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    if (decodeState(status_words_[axis]) != state) {
      return static_cast<std::int8_t>(axis);
    }
  }
  return -1;
}

void EnableManagerController::refreshStatusWords()
{
  for (std::size_t axis = 0; axis < cia402_axes_.size(); ++axis) {
    status_words_[axis] = cia402_axes_[axis].get_status_word().value_or(0xFFFFU);
  }
}

std::uint64_t EnableManagerController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
}

EnableManagerController::DriveState EnableManagerController::decodeState(std::uint16_t status_word)
{
  return hw_semantic_components::decode_cia402_state(status_word);
}

bool EnableManagerController::isFaultState(DriveState state)
{
  return state == DriveState::kFault || state == DriveState::kFaultReactionActive;
}

bool EnableManagerController::isConfirmedDisableTerminal(std::size_t axis, DriveState state) const
{
  if (state == DriveState::kSwitchOnDisabled) {
    return true;
  }
  return axis < ready_to_switch_on_disable_terminal_.size() &&
         ready_to_switch_on_disable_terminal_[axis] != 0U && state == DriveState::kReadyToSwitchOn;
}

const char * EnableManagerController::stageName(Stage stage)
{
  switch (stage) {
    case Stage::kSuccess:
      return "success";
    case Stage::kAlreadyEnabled:
      return "already_enabled";
    case Stage::kAlreadyDisabled:
      return "already_disabled";
    case Stage::kAlreadyClear:
      return "already_clear";
    case Stage::kOperationInProgress:
      return "operation_in_progress";
    case Stage::kControllerInactive:
      return "controller_inactive";
    case Stage::kPreemptedByDisable:
      return "preempted_by_disable";
    case Stage::kEnableBatchTimeout:
      return "enable_batch_timeout";
    case Stage::kEnableInvalidState:
      return "enable_invalid_state";
    case Stage::kModeMismatch:
      return "mode_mismatch";
    case Stage::kModeNotConfirmed:
      return "mode_not_confirmed";
    case Stage::kStateSourcesNotReady:
      return "state_sources_not_ready";
    case Stage::kMotionFault:
      return "motion_fault";
    case Stage::kFaultDetected:
      return "fault_detected";
    case Stage::kUnexpectedDriveState:
      return "unexpected_drive_state";
    case Stage::kDisableTimeout:
      return "disable_timeout";
    case Stage::kFaultRequiresReset:
      return "fault_requires_reset";
    case Stage::kFaultResetTimeout:
      return "fault_reset_timeout";
    case Stage::kJtcActivateFailed:
      return "jtc_activate_failed";
    case Stage::kJtcDeactivateFailed:
      return "jtc_deactivate_failed";
    case Stage::kControllerUpdateTimeout:
      return "controller_update_timeout";
    case Stage::kRestartRequired:
      return "restart_required";
    case Stage::kDeviceOffline:
      return "device_offline";
  }
  return "unknown";
}

const char * EnableManagerController::phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kInactive:
      return "INACTIVE";
    case Phase::kStartupSanitizing:
      return "STARTUP_SANITIZING";
    case Phase::kIdle:
      return "IDLE";
    case Phase::kEnabling:
      return "ENABLING";
    case Phase::kInterBatchDelay:
      return "INTER_BATCH_DELAY";
    case Phase::kJtcActivating:
      return "JTC_ACTIVATING";
    case Phase::kEnabled:
      return "ENABLED";
    case Phase::kJtcDeactivating:
      return "JTC_DEACTIVATING";
    case Phase::kDisabling:
      return "DISABLING";
    case Phase::kRollback:
      return "ROLLBACK";
    case Phase::kResetLow:
      return "RESET_LOW";
    case Phase::kResetHigh:
      return "RESET_HIGH";
    case Phase::kResetDisabling:
      return "RESET_DISABLING";
    case Phase::kEmergencyQuickStop:
      return "EMERGENCY_QUICK_STOP";
    case Phase::kEmergencyDisable:
      return "EMERGENCY_DISABLE";
    case Phase::kFailed:
      return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace ctrl_enable_manager

PLUGINLIB_EXPORT_CLASS(
  ctrl_enable_manager::EnableManagerController, controller_interface::ControllerInterface)
