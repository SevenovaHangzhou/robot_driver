// Motion mode switching: arms between the trajectory and rolling controllers, the swerve chassis
// between the swerve controller (drive wheels CSV) and the wheel trajectory controller (CSP).
#include "ctrl_enable_manager/enable_manager_controller.hpp"
#include "hw_semantic_components/feedback_age.hpp"

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
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace ctrl_enable_manager
{

void EnableManagerController::handleJtcState(
  MotionGroup & group, const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message)
{
  if (
    message->joint_names.size() != group.joint_names.size() ||
    message->output.positions.size() != group.joint_names.size()) {
    return;
  }
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kFjtReady);
  snapshot.received_steady_ns = steadyNowNs();
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    if (
      message->joint_names[axis] != group.joint_names[axis] ||
      !std::isfinite(message->output.positions[axis])) {
      return;
    }
    snapshot.positions[axis] = message->output.positions[axis];
  }
  std::lock_guard<std::mutex> lock(group.snapshot_mutex);
  group.jtc_snapshot = snapshot;
}

void EnableManagerController::handleRollingState(
  MotionGroup & group,
  const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message)
{
  // Published only while the rolling controller is active.
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kRollingReady);
  snapshot.received_steady_ns = steadyNowNs();
  snapshot.rolling_state = message->state;
  std::lock_guard<std::mutex> lock(group.snapshot_mutex);
  group.rolling_snapshot = snapshot;
}

bool EnableManagerController::motionStatesBound(const MotionGroup & group) const
{
  // state_interface_configuration lists the group positions, then the EtherCAT data age, right
  // after the CiA402 axes.
  if (group.joint_names.empty()) {
    return true;
  }
  const std::size_t offset = cia402_axes_.size();
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    if (state_interfaces_[offset + axis].get_name() != group.joint_names[axis] + "/position") {
      return false;
    }
  }
  return state_interfaces_[offset + group.joint_names.size()].get_name() ==
         "ethercat_domain/process_data_age_ms";
}

void EnableManagerController::resetMotionGroup(MotionGroup & group)
{
  group.callback_active.store(false, std::memory_order_release);
  group.abort_requested.store(false, std::memory_order_release);
  group.current_mode.store(ControlMode::kDisabled, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(group.snapshot_mutex);
    group.jtc_snapshot = CommandSnapshot{};
    group.rolling_snapshot = CommandSnapshot{};
  }
  group.sample_version.store(0U, std::memory_order_relaxed);
  group.sample_sequence.store(0U, std::memory_order_relaxed);
  group.sample_time_ns.store(0U, std::memory_order_relaxed);
  if (group.joint_names.empty()) {
    return;
  }
  const std::size_t offset = cia402_axes_.size();
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    group.sample_positions[axis].store(
      state_interfaces_[offset + axis].get_value(), std::memory_order_relaxed);
  }
  group.sample_feedback_age_ms.store(
    state_interfaces_[offset + group.joint_names.size()].get_value(), std::memory_order_relaxed);
}

void EnableManagerController::sampleMotionGroup(MotionGroup & group) noexcept
{
  // Real-time: seqlock-style publication of this cycle's positions for the standstill check.
  if (group.joint_names.empty()) {
    return;
  }
  const std::size_t offset = cia402_axes_.size();
  group.sample_version.fetch_add(1U, std::memory_order_acq_rel);
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    group.sample_positions[axis].store(
      state_interfaces_[offset + axis].get_value(), std::memory_order_relaxed);
  }
  group.sample_feedback_age_ms.store(
    state_interfaces_[offset + group.joint_names.size()].get_value(), std::memory_order_relaxed);
  group.sample_time_ns.store(steadyNowNs(), std::memory_order_relaxed);
  group.sample_sequence.fetch_add(1U, std::memory_order_relaxed);
  group.sample_version.fetch_add(1U, std::memory_order_release);
}

bool EnableManagerController::readActualSample(
  const MotionGroup & group, ActualSample & sample) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 5U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before = group.sample_version.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }
    for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
      sample.positions[axis] = group.sample_positions[axis].load(std::memory_order_relaxed);
    }
    sample.feedback_age_ms = group.sample_feedback_age_ms.load(std::memory_order_relaxed);
    sample.steady_time_ns = group.sample_time_ns.load(std::memory_order_relaxed);
    sample.sequence = group.sample_sequence.load(std::memory_order_relaxed);
    const std::uint64_t version_after = group.sample_version.load(std::memory_order_acquire);
    if (version_before == version_after && (version_after & 1U) == 0U) {
      return sample.sequence != 0U && sample.steady_time_ns != 0U;
    }
  }
  return false;
}

bool EnableManagerController::readCommandSnapshot(
  const MotionGroup & group, ControlMode mode, CommandSnapshot & snapshot) const
{
  std::lock_guard<std::mutex> lock(group.snapshot_mutex);
  if (mode == ControlMode::kFjtReady) {
    snapshot = group.jtc_snapshot;
  } else if (mode == ControlMode::kRollingReady) {
    snapshot = group.rolling_snapshot;
  } else {
    return false;
  }
  return snapshot.valid && snapshot.control_mode == static_cast<std::uint8_t>(mode);
}

std::uint8_t EnableManagerController::validateModeSource(
  MotionGroup & group, ControlMode source, CommandSnapshot & snapshot)
{
  // The swerve controller has no source state: the measured standstill alone admits leaving it.
  const bool source_has_state = source != ControlMode::kVelocityReady;
  if (source_has_state && !readCommandSnapshot(group, source, snapshot)) {
    return kModeSourceStale;
  }
  std::uint64_t now_ns = steadyNowNs();
  if (
    source_has_state && (now_ns < snapshot.received_steady_ns ||
                         now_ns - snapshot.received_steady_ns > group.source_state_max_age_ns)) {
    return kModeSourceStale;
  }
  if (
    source == ControlMode::kRollingReady &&
    snapshot.rolling_state != robot_rt_control_interfaces::msg::RollingJointControlState::HOLDING) {
    return kModeSourceMoving;
  }

  ActualSample current;
  const auto result = waitForStandstill(group, Phase::kEnabled, Owner::kMode, current);
  if (result != kModeOk) {
    return result;
  }
  if (!source_has_state) {
    return kModeOk;
  }
  CommandSnapshot fresh_source;
  if (!readCommandSnapshot(group, source, fresh_source)) {
    return kModeSourceStale;
  }
  now_ns = steadyNowNs();
  if (
    now_ns < fresh_source.received_steady_ns ||
    now_ns - fresh_source.received_steady_ns > group.source_state_max_age_ns) {
    return kModeSourceStale;
  }
  if (
    source == ControlMode::kRollingReady &&
    fresh_source.rolling_state !=
      robot_rt_control_interfaces::msg::RollingJointControlState::HOLDING) {
    return kModeSourceMoving;
  }
  // The trajectory controller hands over its output; rolling checks its own takeover on
  // activation and the trajectory controller starts from the measured state.
  if (
    source == ControlMode::kFjtReady &&
    !sourceWithinTakeoverTolerance(group, fresh_source.positions, current.positions)) {
    return kModeTakeoverMismatch;
  }
  snapshot = fresh_source;
  return kModeOk;
}

std::uint8_t EnableManagerController::waitForStandstill(
  MotionGroup & group, Phase phase, Owner owner, ActualSample & sample)
{
  ActualSample previous;
  if (!readActualSample(group, previous)) {
    return kModeNotReady;
  }
  if (!hw_semantic_components::feedback_fresh(
        previous.feedback_age_ms, group.feedback_age_limit_ms)) {
    return kModeFeedbackStale;
  }

  // Standstill over a window of consecutive samples: its start-to-end displacement rate. Per
  // 1 ms interval one encoder count already exceeds the threshold on a still arm (bench
  // 2026-11-05); a 20-sample window keeps quantization 20 times smaller.
  const auto barrier = steadyNowNs();
  ActualSample window_start = previous;
  std::size_t window_intervals = 0U;
  const auto deadline = std::chrono::steady_clock::now() + group.switch_timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      !active_.load(std::memory_order_acquire) ||
      restart_required_.load(std::memory_order_acquire) ||
      (owner == Owner::kMode && group.abort_requested.load(std::memory_order_acquire)) ||
      owner_.load(std::memory_order_acquire) != owner ||
      phase_.load(std::memory_order_acquire) != phase) {
      return kModeNotReady;
    }
    ActualSample current;
    if (!readActualSample(group, current) || current.sequence == previous.sequence) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (!hw_semantic_components::feedback_fresh(
          current.feedback_age_ms, group.feedback_age_limit_ms)) {
      return kModeFeedbackStale;
    }
    const bool contiguous =
      current.steady_time_ns > previous.steady_time_ns &&
      current.steady_time_ns - previous.steady_time_ns <= group.maximum_sample_period_ns;
    previous = current;
    if (!contiguous || window_start.steady_time_ns < barrier) {
      window_start = current;
      window_intervals = 0U;
      continue;
    }
    if (++window_intervals < group.stable_interval_count) {
      continue;
    }
    if (!stableActualInterval(group, window_start, current)) {
      window_start = current;
      window_intervals = 0U;
      continue;
    }

    sample = current;
    return kModeOk;
  }
  return kModeSourceMoving;
}

void EnableManagerController::handleConfirmStandstill(
  MotionGroup & group, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Domain-local, read-only query. It neither claims command ownership nor switches modes.
  const auto phase = phase_.load(std::memory_order_acquire);
  ActualSample sample;
  const std::uint8_t result = (phase == Phase::kIdle || phase == Phase::kEnabled) &&
                                  owner_.load(std::memory_order_acquire) == Owner::kNone
                                ? waitForStandstill(group, phase, Owner::kNone, sample)
                                : kModeNotReady;
  robot_system_interfaces::msg::ErrorInfo error;
  populateModeError(error, result);
  response->success = result == kModeOk;
  response->message = error.detail;
}

bool EnableManagerController::queryControllerStates(
  controller_manager_msgs::srv::ListControllers::Response & states,
  std::chrono::milliseconds timeout)
{
  if (!list_client_->wait_for_service(std::chrono::milliseconds(0))) {
    return false;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return false;
  }
  const auto response = future.get();
  if (response == nullptr) {
    return false;
  }
  states = *response;
  return true;
}

void EnableManagerController::convergeAfterUnsafeModeSwitch(
  MotionGroup & group, bool require_restart)
{
  restart_required_.store(require_restart, std::memory_order_release);
  group.current_mode.store(
    require_restart ? ControlMode::kRestartRequired : ControlMode::kDisabled,
    std::memory_order_release);
  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivation_required_.store(true, std::memory_order_release);
  owner_.store(Owner::kDisable, std::memory_order_release);
  if (require_restart) {
    emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
  }
  disable_request_.store(true, std::memory_order_release);
}

bool EnableManagerController::releaseModeOwnership() noexcept
{
  Owner expected_owner = Owner::kMode;
  return owner_.compare_exchange_strong(expected_owner, Owner::kNone, std::memory_order_acq_rel);
}

std::uint8_t EnableManagerController::executeModeSwitch(
  MotionGroup & group, ControlMode source, ControlMode target)
{
  if (group.abort_requested.load(std::memory_order_acquire)) {
    return kModeNotReady;
  }
  bool switch_expected = false;
  if (!switch_in_progress_.compare_exchange_strong(
        switch_expected, true, std::memory_order_acq_rel)) {
    return kModeNotReady;
  }
  const auto release_switch = [this]() {
    switch_in_progress_.store(false, std::memory_order_release);
  };
  const auto safety_preempted = [this]() {
    return owner_.load(std::memory_order_acquire) != Owner::kMode ||
           phase_.load(std::memory_order_acquire) != Phase::kEnabled;
  };
  if (group.abort_requested.load(std::memory_order_acquire)) {
    release_switch();
    return kModeNotReady;
  }
  controller_manager_msgs::srv::ListControllers::Response before;
  if (!queryControllerStates(before, group.switch_timeout)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    group.current_mode.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return kModeRestartRequired;
  }
  if (detectControlMode(group, before) != source) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }
  if (group.abort_requested.load(std::memory_order_acquire)) {
    release_switch();
    return kModeNotReady;
  }
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }

  // Leaving CSP: the drives return to their startup mode under the trajectory controller's hold
  // before the velocity controller takes over.
  if (
    switchedDrivesUseCsp(group, source) && !switchedDrivesUseCsp(group, target) &&
    !switchDriveModes(group, target)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, false);
    return kModeDriveModeTimeout;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (buildControllerSwitchRequest(group, before, target, *request) != SwitchResult::kSuccess) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration(group.switch_timeout);
  if (emergencyStopBlocksEnable() || safety_preempted()) {
    release_switch();
    return kModeNotReady;
  }
  auto future = switch_client_->async_send_request(request);
  const bool response_ready = future.wait_for(group.switch_timeout) == std::future_status::ready;
  const auto switch_response = response_ready ? future.get() : nullptr;
  const bool switch_ok = switch_response != nullptr && switch_response->ok;

  controller_manager_msgs::srv::ListControllers::Response after;
  if (!queryControllerStates(after, group.switch_timeout)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }
  const ModeSwitchState observed = classifyModeSwitchState(group, after, source, target);
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    group.current_mode.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return kModeRestartRequired;
  }
  if (!response_ready) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, true);
    return kModeRestartRequired;
  }
  if (switch_ok && observed == ModeSwitchState::kTargetActive) {
    // Entering CSP: the trajectory controller holds the measured position, then the drives
    // switch; a drive that does not confirm CSP in time stops the group.
    if (switchedDrivesUseCsp(group, target) && !switchDriveModes(group, target)) {
      release_switch();
      convergeAfterUnsafeModeSwitch(group, false);
      return kModeDriveModeTimeout;
    }
    group.current_mode.store(target, std::memory_order_release);
    release_switch();
    return kModeOk;
  }
  if (!switch_ok && observed == ModeSwitchState::kSourcePreserved) {
    release_switch();
    return kModeSwitchRejected;
  }
  if (observed == ModeSwitchState::kAllInactive) {
    release_switch();
    convergeAfterUnsafeModeSwitch(group, false);
    return kModeSwitchRejected;
  }
  release_switch();
  convergeAfterUnsafeModeSwitch(group, true);
  return kModeRestartRequired;
}

void EnableManagerController::handleSetMode(
  MotionGroup & group, const std::shared_ptr<ModeService::Request> request,
  std::shared_ptr<ModeService::Response> response)
{
  using ModeMessage = robot_rt_control_interfaces::msg::JointControlMode;
  *response = ModeService::Response{};
  const auto respond = [&response](std::uint8_t result, ControlMode mode) {
    response->accepted = result == kModeOk;
    response->mode.value = static_cast<std::uint8_t>(mode);
    populateModeError(response->error, result);
  };

  bool expected_callback = false;
  if (!group.callback_active.compare_exchange_strong(expected_callback, true)) {
    respond(kModeNotReady, group.current_mode.load(std::memory_order_acquire));
    return;
  }
  const auto finish = [&group, &respond](std::uint8_t result, ControlMode mode) {
    respond(result, mode);
    group.callback_active.store(false, std::memory_order_release);
  };

  if (!active_.load(std::memory_order_acquire)) {
    finish(kModeNotEnabled, ControlMode::kDisabled);
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    finish(kModeRestartRequired, ControlMode::kRestartRequired);
    return;
  }
  if (emergencyStopBlocksEnable()) {
    finish(kModeNotReady, group.current_mode.load(std::memory_order_acquire));
    return;
  }
  const ControlMode current_mode = group.current_mode.load(std::memory_order_acquire);
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kEnabled) {
    finish(phase == Phase::kIdle ? kModeNotEnabled : kModeNotReady, current_mode);
    return;
  }
  const ControlMode target_mode = static_cast<ControlMode>(request->target_mode.value);
  const std::uint8_t alternate_mode =
    group.rolling_controller.empty() ? ModeMessage::VELOCITY_READY : ModeMessage::ROLLING_READY;
  if (
    request->target_mode.value != ModeMessage::FJT_READY &&
    request->target_mode.value != alternate_mode) {
    finish(kModeWrongMode, current_mode);
    return;
  }
  if (owner_.load(std::memory_order_acquire) != Owner::kNone) {
    finish(kModeNotReady, current_mode);
    return;
  }
  if (target_mode == current_mode) {
    finish(kModeOk, current_mode);
    return;
  }

  Owner expected_owner = Owner::kNone;
  group.abort_requested.store(false, std::memory_order_release);
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kMode)) {
    finish(kModeNotReady, current_mode);
    return;
  }
  CommandSnapshot source_snapshot;
  const std::uint8_t admission_result = validateModeSource(group, current_mode, source_snapshot);
  if (admission_result != kModeOk) {
    (void)releaseModeOwnership();
    finish(admission_result, current_mode);
    return;
  }
  const std::uint8_t switch_result = executeModeSwitch(group, current_mode, target_mode);
  (void)releaseModeOwnership();
  finish(switch_result, group.current_mode.load(std::memory_order_acquire));
}

bool EnableManagerController::controllerRunsInMode(
  const MotionGroup & group, const std::string & name, ControlMode target) const
{
  if (target == ControlMode::kDisabled) {
    return false;
  }
  if (name == group.default_controller) {
    return target == group.default_mode;
  }
  if (name == group.rolling_controller) {
    return target == ControlMode::kRollingReady;
  }
  if (name == group.trajectory_controller) {
    return target == ControlMode::kFjtReady;
  }
  return true;  // Companions stay active in either motion mode.
}

EnableManagerController::SwitchResult EnableManagerController::buildControllerSwitchRequest(
  const MotionGroup & group, const controller_manager_msgs::srv::ListControllers::Response & states,
  ControlMode target, controller_manager_msgs::srv::SwitchController::Request & request) const
{
  request.activate_controllers.clear();
  request.deactivate_controllers.clear();
  for (const auto & name : managed_controllers_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    if (controller == states.controller.end()) {
      return SwitchResult::kFailed;
    }
    const bool is_active = controller->state == "active";
    if (controllerRunsInMode(group, name, target)) {
      if (!is_active) {
        if (controller->state != "inactive") {
          return SwitchResult::kFailed;
        }
        request.activate_controllers.push_back(name);
      }
    } else if (is_active) {
      request.deactivate_controllers.push_back(name);
    }
  }
  return SwitchResult::kSuccess;
}

bool EnableManagerController::controllerStateMatches(
  const MotionGroup & group, const controller_manager_msgs::srv::ListControllers::Response & states,
  ControlMode target) const
{
  for (const auto & name : managed_controllers_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    const char * expected = controllerRunsInMode(group, name, target) ? "active" : "inactive";
    if (controller == states.controller.end() || controller->state != expected) {
      return false;
    }
  }
  return true;
}

EnableManagerController::ControlMode EnableManagerController::detectControlMode(
  const MotionGroup & group,
  const controller_manager_msgs::srv::ListControllers::Response & states) const
{
  if (controllerStateMatches(group, states, ControlMode::kDisabled)) {
    return ControlMode::kDisabled;
  }
  const ControlMode alternate = !group.rolling_controller.empty()      ? ControlMode::kRollingReady
                                : !group.trajectory_controller.empty() ? ControlMode::kFjtReady
                                                                       : ControlMode::kDisabled;
  for (const ControlMode mode : {group.default_mode, alternate}) {
    if (mode != ControlMode::kDisabled && controllerStateMatches(group, states, mode)) {
      return mode;
    }
  }
  return ControlMode::kRestartRequired;
}

EnableManagerController::ModeSwitchState EnableManagerController::classifyModeSwitchState(
  const MotionGroup & group, const controller_manager_msgs::srv::ListControllers::Response & states,
  ControlMode source, ControlMode target) const
{
  const ControlMode observed = detectControlMode(group, states);
  if (observed == target) {
    return ModeSwitchState::kTargetActive;
  }
  if (observed == source) {
    return ModeSwitchState::kSourcePreserved;
  }
  if (observed == ControlMode::kDisabled) {
    return ModeSwitchState::kAllInactive;
  }
  return ModeSwitchState::kAmbiguous;
}

bool EnableManagerController::stableActualInterval(
  const MotionGroup & group, const ActualSample & window_start,
  const ActualSample & current) const noexcept
{
  const double elapsed_seconds =
    static_cast<double>(current.steady_time_ns - window_start.steady_time_ns) / 1.0e9;
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    const double speed =
      std::abs(current.positions[axis] - window_start.positions[axis]) / elapsed_seconds;
    if (!std::isfinite(speed) || speed > group.stable_velocity_thresholds[axis]) {
      return false;
    }
  }
  return true;
}

bool EnableManagerController::sourceWithinTakeoverTolerance(
  const MotionGroup & group, const MotionPositions & source,
  const MotionPositions & actual) const noexcept
{
  for (std::size_t axis = 0U; axis < group.joint_names.size(); ++axis) {
    if (
      !std::isfinite(source[axis]) || !std::isfinite(actual[axis]) ||
      std::abs(source[axis] - actual[axis]) > group.takeover_tolerances[axis]) {
      return false;
    }
  }
  return true;
}

void EnableManagerController::populateModeError(
  robot_system_interfaces::msg::ErrorInfo & error, std::uint8_t result)
{
  using ErrorCode = robot_system_interfaces::msg::ErrorCode;
  using ErrorInfo = robot_system_interfaces::msg::ErrorInfo;
  std::uint32_t code = ErrorCode::INTERNAL_ERROR;
  const char * detail = "internal";
  switch (result) {
    case kModeOk:
      code = ErrorCode::SUCCESS;
      detail = "ok";
      break;
    case kModeNotEnabled:
      code = ErrorCode::NOT_READY;
      detail = "not_enabled";
      break;
    case kModeNotReady:
      code = ErrorCode::NOT_READY;
      detail = "busy";
      break;
    case kModeSourceStale:
      code = ErrorCode::NOT_READY;
      detail = "source_state_stale";
      break;
    case kModeSourceMoving:
      code = ErrorCode::NOT_READY;
      detail = "source_moving";
      break;
    case kModeFeedbackStale:
      code = ErrorCode::NOT_READY;
      detail = "feedback_stale";
      break;
    case kModeWrongMode:
      code = ErrorCode::GOAL_REJECTED;
      detail = "wrong_mode";
      break;
    case kModeSwitchRejected:
      code = ErrorCode::GOAL_REJECTED;
      detail = "switch_rejected";
      break;
    case kModeTakeoverMismatch:
      code = ErrorCode::RT_TOLERANCE_VIOLATED;
      detail = "takeover_mismatch";
      break;
    case kModeRestartRequired:
      code = ErrorCode::RT_RESTART_REQUIRED;
      detail = "restart_required";
      break;
    case kModeDriveModeTimeout:
      code = ErrorCode::TIMEOUT;
      detail = "drive_mode_not_confirmed";
      break;
    default:
      break;
  }
  error.code = code;
  error.retryable = code != ErrorCode::SUCCESS && ((code / 100U) % 10U) == 1U;
  error.severity = code == ErrorCode::SUCCESS
                     ? ErrorInfo::OK
                     : (error.retryable ? ErrorInfo::WARN : ErrorInfo::FAULT);
  error.source = "rt_control/enable_manager";
  error.message = code == ErrorCode::SUCCESS ? "mode request accepted" : "mode request rejected";
  error.detail = detail;
}

}  // namespace ctrl_enable_manager
