// Motion mode switching between the trajectory and rolling controllers.
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
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message)
{
  if (
    message->joint_names.size() != motion_joint_names_.size() ||
    message->output.positions.size() != motion_joint_names_.size()) {
    return;
  }
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kFjtReady);
  snapshot.received_steady_ns = steadyNowNs();
  for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
    if (
      message->joint_names[axis] != motion_joint_names_[axis] ||
      !std::isfinite(message->output.positions[axis])) {
      return;
    }
    snapshot.positions[axis] = message->output.positions[axis];
  }
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  jtc_command_snapshot_ = snapshot;
}

void EnableManagerController::handleRollingState(
  const robot_rt_control_interfaces::msg::RollingJointControlState::SharedPtr message)
{
  // Published only while the rolling controller is active.
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kRollingReady);
  snapshot.received_steady_ns = steadyNowNs();
  snapshot.rolling_state = message->state;
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  rolling_command_snapshot_ = snapshot;
}

bool EnableManagerController::readActualSample(ActualSample & sample) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 5U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before = actual_sample_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }
    for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
      sample.positions[axis] = actual_sample_positions_[axis].load(std::memory_order_relaxed);
    }
    sample.feedback_age_ms = actual_sample_feedback_age_ms_.load(std::memory_order_relaxed);
    sample.steady_time_ns = actual_sample_time_ns_.load(std::memory_order_relaxed);
    sample.sequence = actual_sample_sequence_.load(std::memory_order_relaxed);
    const std::uint64_t version_after = actual_sample_version_.load(std::memory_order_acquire);
    if (version_before == version_after && (version_after & 1U) == 0U) {
      return sample.sequence != 0U && sample.steady_time_ns != 0U;
    }
  }
  return false;
}

bool EnableManagerController::readCommandSnapshot(
  ControlMode mode, CommandSnapshot & snapshot) const
{
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  if (mode == ControlMode::kFjtReady) {
    snapshot = jtc_command_snapshot_;
  } else if (mode == ControlMode::kRollingReady) {
    snapshot = rolling_command_snapshot_;
  } else {
    return false;
  }
  return snapshot.valid && snapshot.control_mode == static_cast<std::uint8_t>(mode);
}

std::uint8_t EnableManagerController::validateModeSource(
  ControlMode source, CommandSnapshot & snapshot)
{
  if (!readCommandSnapshot(source, snapshot)) {
    return kModeSourceStale;
  }
  std::uint64_t now_ns = steadyNowNs();
  if (
    now_ns < snapshot.received_steady_ns ||
    now_ns - snapshot.received_steady_ns > mode_switch_source_state_max_age_ns_) {
    return kModeSourceStale;
  }
  if (
    source == ControlMode::kRollingReady &&
    snapshot.rolling_state != robot_rt_control_interfaces::msg::RollingJointControlState::HOLDING) {
    return kModeSourceMoving;
  }

  ActualSample previous;
  if (!readActualSample(previous)) {
    return kModeNotReady;
  }
  if (!hw_semantic_components::feedback_fresh(
        previous.feedback_age_ms, mode_switch_feedback_age_limit_ms_)) {
    return kModeFeedbackStale;
  }

  // Standstill over a window of consecutive samples: its start-to-end displacement rate. Per
  // 1 ms interval one encoder count already exceeds the threshold on a still arm (bench
  // 2026-11-05); a 20-sample window keeps quantization 20 times smaller.
  ActualSample window_start = previous;
  std::size_t window_intervals = 0U;
  const auto deadline = std::chrono::steady_clock::now() + mode_switch_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      mode_abort_requested_.load(std::memory_order_acquire) ||
      owner_.load(std::memory_order_acquire) != Owner::kMode ||
      phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
      return kModeNotReady;
    }
    ActualSample current;
    if (!readActualSample(current) || current.sequence == previous.sequence) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (!hw_semantic_components::feedback_fresh(
          current.feedback_age_ms, mode_switch_feedback_age_limit_ms_)) {
      return kModeFeedbackStale;
    }
    const bool contiguous =
      current.steady_time_ns > previous.steady_time_ns &&
      current.steady_time_ns - previous.steady_time_ns <= mode_switch_maximum_sample_period_ns_;
    previous = current;
    if (!contiguous) {
      window_start = current;
      window_intervals = 0U;
      continue;
    }
    if (++window_intervals < mode_switch_stable_interval_count_) {
      continue;
    }
    if (!stableActualInterval(window_start, current)) {
      window_start = current;
      window_intervals = 0U;
      continue;
    }

    CommandSnapshot fresh_source;
    if (!readCommandSnapshot(source, fresh_source)) {
      return kModeSourceStale;
    }
    now_ns = steadyNowNs();
    if (
      now_ns < fresh_source.received_steady_ns ||
      now_ns - fresh_source.received_steady_ns > mode_switch_source_state_max_age_ns_) {
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
      !sourceWithinTakeoverTolerance(fresh_source.positions, current.positions)) {
      return kModeTakeoverMismatch;
    }
    snapshot = fresh_source;
    return kModeOk;
  }
  return kModeSourceMoving;
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

void EnableManagerController::convergeAfterUnsafeModeSwitch(bool require_restart)
{
  restart_required_.store(require_restart, std::memory_order_release);
  current_control_mode_.store(
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

std::uint8_t EnableManagerController::executeModeSwitch(ControlMode source, ControlMode target)
{
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
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
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    release_switch();
    return kModeNotReady;
  }
  controller_manager_msgs::srv::ListControllers::Response before;
  if (!queryControllerStates(before, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return kModeRestartRequired;
  }
  if (detectControlMode(before) != source) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    release_switch();
    return kModeNotReady;
  }
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (buildControllerSwitchRequest(before, target, *request) != SwitchResult::kSuccess) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration(mode_switch_timeout_);
  auto future = switch_client_->async_send_request(request);
  const bool response_ready = future.wait_for(mode_switch_timeout_) == std::future_status::ready;
  const auto switch_response = response_ready ? future.get() : nullptr;
  const bool switch_ok = switch_response != nullptr && switch_response->ok;

  controller_manager_msgs::srv::ListControllers::Response after;
  if (!queryControllerStates(after, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }
  const ModeSwitchState observed = classifyModeSwitchState(after, source, target);
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return kModeRestartRequired;
  }
  if (!response_ready) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return kModeRestartRequired;
  }
  if (switch_ok && observed == ModeSwitchState::kTargetActive) {
    current_control_mode_.store(target, std::memory_order_release);
    release_switch();
    return kModeOk;
  }
  if (!switch_ok && observed == ModeSwitchState::kSourcePreserved) {
    release_switch();
    return kModeSwitchRejected;
  }
  if (observed == ModeSwitchState::kAllInactive) {
    release_switch();
    convergeAfterUnsafeModeSwitch(false);
    return kModeSwitchRejected;
  }
  release_switch();
  convergeAfterUnsafeModeSwitch(true);
  return kModeRestartRequired;
}

void EnableManagerController::handleSetMode(
  const std::shared_ptr<ModeService::Request> request,
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
  if (!mode_callback_active_.compare_exchange_strong(expected_callback, true)) {
    respond(kModeNotReady, current_control_mode_.load(std::memory_order_acquire));
    return;
  }
  const auto finish = [this, &respond](std::uint8_t result, ControlMode mode) {
    respond(result, mode);
    mode_callback_active_.store(false, std::memory_order_release);
  };

  if (!active_.load(std::memory_order_acquire)) {
    finish(kModeNotEnabled, ControlMode::kDisabled);
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    finish(kModeRestartRequired, ControlMode::kRestartRequired);
    return;
  }
  const ControlMode current_mode = current_control_mode_.load(std::memory_order_acquire);
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kEnabled) {
    finish(phase == Phase::kIdle ? kModeNotEnabled : kModeNotReady, current_mode);
    return;
  }
  const ControlMode target_mode = static_cast<ControlMode>(request->target_mode.value);
  if (
    request->target_mode.value != ModeMessage::FJT_READY &&
    request->target_mode.value != ModeMessage::ROLLING_READY) {
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
  mode_abort_requested_.store(false, std::memory_order_release);
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kMode)) {
    finish(kModeNotReady, current_mode);
    return;
  }
  CommandSnapshot source_snapshot;
  const std::uint8_t admission_result = validateModeSource(current_mode, source_snapshot);
  if (admission_result != kModeOk) {
    (void)releaseModeOwnership();
    finish(admission_result, current_mode);
    return;
  }
  const std::uint8_t switch_result = executeModeSwitch(current_mode, target_mode);
  (void)releaseModeOwnership();
  finish(switch_result, current_control_mode_.load(std::memory_order_acquire));
}

bool EnableManagerController::controllerRunsInMode(
  const std::string & name, ControlMode target) const
{
  if (target == ControlMode::kDisabled) {
    return false;
  }
  if (name == default_motion_controller_) {
    return target == ControlMode::kFjtReady;
  }
  if (name == rolling_motion_controller_) {
    return target == ControlMode::kRollingReady;
  }
  return true;  // Companions stay active in either motion mode.
}

EnableManagerController::SwitchResult EnableManagerController::buildControllerSwitchRequest(
  const controller_manager_msgs::srv::ListControllers::Response & states, ControlMode target,
  controller_manager_msgs::srv::SwitchController::Request & request) const
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
    if (controllerRunsInMode(name, target)) {
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
  const controller_manager_msgs::srv::ListControllers::Response & states, ControlMode target) const
{
  for (const auto & name : managed_controllers_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    const char * expected = controllerRunsInMode(name, target) ? "active" : "inactive";
    if (controller == states.controller.end() || controller->state != expected) {
      return false;
    }
  }
  return true;
}

EnableManagerController::ControlMode EnableManagerController::detectControlMode(
  const controller_manager_msgs::srv::ListControllers::Response & states) const
{
  if (controllerStateMatches(states, ControlMode::kDisabled)) {
    return ControlMode::kDisabled;
  }
  if (controllerStateMatches(states, ControlMode::kFjtReady)) {
    return ControlMode::kFjtReady;
  }
  if (motion_mode_switching_ && controllerStateMatches(states, ControlMode::kRollingReady)) {
    return ControlMode::kRollingReady;
  }
  return ControlMode::kRestartRequired;
}

EnableManagerController::ModeSwitchState EnableManagerController::classifyModeSwitchState(
  const controller_manager_msgs::srv::ListControllers::Response & states, ControlMode source,
  ControlMode target) const
{
  const ControlMode observed = detectControlMode(states);
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
  const ActualSample & window_start, const ActualSample & current) const noexcept
{
  const double elapsed_seconds =
    static_cast<double>(current.steady_time_ns - window_start.steady_time_ns) / 1.0e9;
  for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
    const double speed =
      std::abs(current.positions[axis] - window_start.positions[axis]) / elapsed_seconds;
    if (!std::isfinite(speed) || speed > mode_switch_stable_velocity_thresholds_[axis]) {
      return false;
    }
  }
  return true;
}

bool EnableManagerController::sourceWithinTakeoverTolerance(
  const MotionPositions & source, const MotionPositions & actual) const noexcept
{
  for (std::size_t axis = 0U; axis < motion_joint_names_.size(); ++axis) {
    if (
      !std::isfinite(source[axis]) || !std::isfinite(actual[axis]) ||
      std::abs(source[axis] - actual[axis]) > mode_switch_takeover_tolerances_[axis]) {
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
