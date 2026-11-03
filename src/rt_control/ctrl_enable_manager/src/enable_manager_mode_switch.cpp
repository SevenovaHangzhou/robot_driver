// Motion mode switching between the trajectory and rolling controllers.
#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <set>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_rt_control_interfaces/msg/rolling_service_result.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"

namespace ctrl_enable_manager
{

void EnableManagerController::handleJtcState(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr message)
{
  if (
    message->joint_names.size() != kMotionAxisCount ||
    message->output.positions.size() != kMotionAxisCount) {
    return;
  }
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = static_cast<std::uint8_t>(ControlMode::kFjtReady);
  snapshot.received_steady_ns = steadyNowNs();
  for (std::size_t axis = 0U; axis < kMotionAxisCount; ++axis) {
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
  CommandSnapshot snapshot;
  snapshot.valid = true;
  snapshot.control_mode = message->control_mode.value;
  snapshot.received_steady_ns = steadyNowNs();
  snapshot.controller_boot_id = message->controller_boot_id.uuid;
  snapshot.has_session = message->has_session;
  snapshot.session_state = message->session_state.value;
  for (std::size_t axis = 0U; axis < kMotionAxisCount; ++axis) {
    if (!std::isfinite(message->desired_positions[axis])) {
      return;
    }
    snapshot.positions[axis] = message->desired_positions[axis];
  }
  std::lock_guard<std::mutex> lock(command_snapshot_mutex_);
  rolling_command_snapshot_ = snapshot;
}

bool EnableManagerController::sameModeRequest(
  const ModeService::Request & lhs, const ModeService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.expected_mode.value == rhs.expected_mode.value &&
         lhs.target_mode.value == rhs.target_mode.value;
}

EnableManagerController::ModeCacheEntry * EnableManagerController::findModeCache(
  const std::array<std::uint8_t, 16U> & client_id,
  const std::array<std::uint8_t, 16U> & request_id) noexcept
{
  const auto entry = std::find_if(
    mode_cache_.begin(), mode_cache_.end(),
    [&client_id, &request_id](const ModeCacheEntry & candidate) {
      return candidate.valid && candidate.request.client_instance_id.uuid == client_id &&
             candidate.request.request_id.uuid == request_id;
    });
  return entry == mode_cache_.end() ? nullptr : &*entry;
}

EnableManagerController::ModeCacheEntry & EnableManagerController::allocateModeCache() noexcept
{
  ModeCacheEntry & entry = mode_cache_[next_mode_cache_slot_];
  next_mode_cache_slot_ = (next_mode_cache_slot_ + 1U) % kModeCacheCapacity;
  entry = ModeCacheEntry{};
  return entry;
}

bool EnableManagerController::readActualSample(ActualSample & sample) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 5U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before = actual_sample_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }
    for (std::size_t axis = 0U; axis < kMotionAxisCount; ++axis) {
      sample.positions[axis] =
        decodeDouble(actual_position_bits_[axis].load(std::memory_order_relaxed));
    }
    sample.feedback_age_ms =
      decodeDouble(actual_feedback_age_bits_.load(std::memory_order_relaxed));
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
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  if (!readCommandSnapshot(source, snapshot)) {
    return ServiceResult::SOURCE_STATE_STALE;
  }
  std::uint64_t now_ns = steadyNowNs();
  if (
    now_ns < snapshot.received_steady_ns ||
    now_ns - snapshot.received_steady_ns > mode_switch_source_state_max_age_ns_) {
    return ServiceResult::SOURCE_STATE_STALE;
  }
  if (source == ControlMode::kRollingReady && snapshot.has_session) {
    return ServiceResult::SESSION_EXISTS;
  }

  ActualSample previous;
  if (!readActualSample(previous)) {
    return ServiceResult::NOT_READY;
  }
  if (
    !std::isfinite(previous.feedback_age_ms) || previous.feedback_age_ms < 0.0 ||
    previous.feedback_age_ms > mode_switch_feedback_age_limit_ms_) {
    return ServiceResult::FEEDBACK_STALE;
  }

  std::size_t stable_intervals = 0U;
  const auto deadline = std::chrono::steady_clock::now() + mode_switch_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      mode_abort_requested_.load(std::memory_order_acquire) ||
      owner_.load(std::memory_order_acquire) != Owner::kMode ||
      phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
      return ServiceResult::NOT_READY;
    }
    ActualSample current;
    if (!readActualSample(current) || current.sequence == previous.sequence) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (
      !std::isfinite(current.feedback_age_ms) || current.feedback_age_ms < 0.0 ||
      current.feedback_age_ms > mode_switch_feedback_age_limit_ms_) {
      return ServiceResult::FEEDBACK_STALE;
    }
    stable_intervals = stableActualInterval(previous, current) ? stable_intervals + 1U : 0U;
    previous = current;
    if (stable_intervals < mode_switch_stable_interval_count_) {
      continue;
    }

    CommandSnapshot fresh_source;
    if (!readCommandSnapshot(source, fresh_source)) {
      return ServiceResult::SOURCE_STATE_STALE;
    }
    now_ns = steadyNowNs();
    if (
      now_ns < fresh_source.received_steady_ns ||
      now_ns - fresh_source.received_steady_ns > mode_switch_source_state_max_age_ns_) {
      return ServiceResult::SOURCE_STATE_STALE;
    }
    if (source == ControlMode::kRollingReady && fresh_source.has_session) {
      return ServiceResult::SESSION_EXISTS;
    }
    if (!sourceWithinTakeoverTolerance(fresh_source.positions, current.positions)) {
      return ServiceResult::TAKEOVER_MISMATCH;
    }
    snapshot = fresh_source;
    return ServiceResult::NONE;
  }
  return ServiceResult::SOURCE_MOVING;
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

const char * EnableManagerController::controllerNameForMode(
  ControlMode mode, const std::string & default_controller,
  const std::string & rolling_controller) noexcept
{
  if (mode == ControlMode::kFjtReady) {
    return default_controller.c_str();
  }
  if (mode == ControlMode::kRollingReady) {
    return rolling_controller.c_str();
  }
  return nullptr;
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

std::uint8_t EnableManagerController::executeModeSwitch(
  ControlMode source, ControlMode target, ModeService::Response & response,
  std::uint64_t switch_started_ns)
{
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    return ServiceResult::NOT_READY;
  }
  bool switch_expected = false;
  if (!switch_in_progress_.compare_exchange_strong(
        switch_expected, true, std::memory_order_acq_rel)) {
    return ServiceResult::NOT_READY;
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
    return ServiceResult::NOT_READY;
  }
  controller_manager_msgs::srv::ListControllers::Response before;
  if (!queryControllerStates(before, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return ServiceResult::RESTART_REQUIRED;
  }
  if (detectControlMode(before) != source) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (mode_abort_requested_.load(std::memory_order_acquire)) {
    release_switch();
    return ServiceResult::NOT_READY;
  }
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }

  const char * source_controller =
    controllerNameForMode(source, default_motion_controller_, rolling_motion_controller_);
  const char * target_controller =
    controllerNameForMode(target, default_motion_controller_, rolling_motion_controller_);
  if (source_controller == nullptr || target_controller == nullptr) {
    release_switch();
    return ServiceResult::WRONG_MODE;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers.emplace_back(source_controller);
  request->activate_controllers.emplace_back(target_controller);
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  const std::int64_t timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(mode_switch_timeout_).count();
  request->timeout.sec = static_cast<std::int32_t>(timeout_ns / 1000000000LL);
  request->timeout.nanosec = static_cast<std::uint32_t>(timeout_ns % 1000000000LL);
  auto future = switch_client_->async_send_request(request);
  const bool response_ready = future.wait_for(mode_switch_timeout_) == std::future_status::ready;
  const auto switch_response = response_ready ? future.get() : nullptr;
  const bool switch_ok = switch_response != nullptr && switch_response->ok;

  controller_manager_msgs::srv::ListControllers::Response after;
  if (!queryControllerStates(after, mode_switch_timeout_)) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  const auto controller_has_state = [&after](const char * name, const char * state) {
    const auto controller = std::find_if(
      after.controller.begin(), after.controller.end(),
      [name](const auto & candidate) { return candidate.name == name; });
    return controller != after.controller.end() && controller->state == state;
  };
  const ModeSwitchState observed = classifyModeSwitchState(after, source, target);
  response.source_controller_deactivated = controller_has_state(source_controller, "inactive");
  response.target_controller_activated = controller_has_state(target_controller, "active");
  if (safety_preempted()) {
    restart_required_.store(true, std::memory_order_release);
    current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
    release_switch();
    return ServiceResult::RESTART_REQUIRED;
  }
  if (!response_ready) {
    release_switch();
    convergeAfterUnsafeModeSwitch(true);
    return ServiceResult::RESTART_REQUIRED;
  }
  if (switch_ok && observed == ModeSwitchState::kTargetActive) {
    if (target == ControlMode::kRollingReady) {
      CommandSnapshot rolling_snapshot;
      if (
        readCommandSnapshot(ControlMode::kRollingReady, rolling_snapshot) &&
        rolling_snapshot.received_steady_ns >= switch_started_ns &&
        !isZeroIdentifier(rolling_snapshot.controller_boot_id)) {
        response.controller_boot_id.uuid = rolling_snapshot.controller_boot_id;
      }
    }
    current_control_mode_.store(target, std::memory_order_release);
    release_switch();
    return ServiceResult::NONE;
  }
  if (!switch_ok && observed == ModeSwitchState::kSourcePreserved) {
    release_switch();
    return ServiceResult::SWITCH_REJECTED;
  }
  if (observed == ModeSwitchState::kAllInactive) {
    release_switch();
    convergeAfterUnsafeModeSwitch(false);
    return ServiceResult::SWITCH_REJECTED;
  }
  release_switch();
  convergeAfterUnsafeModeSwitch(true);
  return ServiceResult::RESTART_REQUIRED;
}

void EnableManagerController::setModeResponse(
  ModeService::Response & response, const ModeService::Request & request, std::uint8_t result,
  ControlMode mode, bool source_deactivated, bool target_activated, bool require_restart)
{
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  response.accepted = result == ServiceResult::NONE;
  response.result.value = result;
  response.request_id = request.request_id;
  response.mode.value = static_cast<std::uint8_t>(mode);
  response.source_controller_deactivated = source_deactivated;
  response.target_controller_activated = target_activated;
  response.restart_required = require_restart;
  populateModeError(response.error, result);
}

void EnableManagerController::handleSetMode(
  const std::shared_ptr<ModeService::Request> request,
  std::shared_ptr<ModeService::Response> response)
{
  using ModeMessage = robot_rt_control_interfaces::msg::JointControlMode;
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  *response = ModeService::Response{};
  response->request_id = request->request_id;

  bool expected_callback = false;
  if (!mode_callback_active_.compare_exchange_strong(expected_callback, true)) {
    setModeResponse(
      *response, *request, ServiceResult::NOT_READY,
      current_control_mode_.load(std::memory_order_acquire), false, false,
      restart_required_.load(std::memory_order_acquire));
    publishModeResult(*response);
    return;
  }
  const auto release_callback = [this]() {
    mode_callback_active_.store(false, std::memory_order_release);
  };
  const auto finish = [this, &request, &response, &release_callback](
                        std::uint8_t result, ControlMode mode, bool source_deactivated,
                        bool target_activated, bool require_restart, bool cache) {
    setModeResponse(
      *response, *request, result, mode, source_deactivated, target_activated, require_restart);
    if (cache) {
      ModeCacheEntry & entry = allocateModeCache();
      entry.valid = true;
      entry.request = *request;
      entry.response = *response;
    }
    publishModeResult(*response);
    release_callback();
  };

  const bool identifiers_valid = !isZeroIdentifier(request->client_instance_id.uuid) &&
                                 !isZeroIdentifier(request->request_id.uuid);
  if (identifiers_valid) {
    ModeCacheEntry * cached =
      findModeCache(request->client_instance_id.uuid, request->request_id.uuid);
    if (cached != nullptr) {
      if (sameModeRequest(cached->request, *request)) {
        *response = cached->response;
        publishModeResult(*response);
      } else {
        setModeResponse(
          *response, *request, ServiceResult::WRONG_REQUEST,
          current_control_mode_.load(std::memory_order_acquire), false, false,
          restart_required_.load(std::memory_order_acquire));
        publishModeResult(*response);
      }
      release_callback();
      return;
    }
  }
  if (!identifiers_valid) {
    finish(
      ServiceResult::WRONG_REQUEST, current_control_mode_.load(std::memory_order_acquire), false,
      false, restart_required_.load(std::memory_order_acquire), false);
    return;
  }
  if (request->protocol_major != 1U || request->protocol_minor != 0U) {
    finish(
      ServiceResult::WRONG_PROTOCOL, current_control_mode_.load(std::memory_order_acquire), false,
      false, restart_required_.load(std::memory_order_acquire), true);
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    finish(ServiceResult::NOT_ENABLED, ControlMode::kDisabled, false, false, false, true);
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    finish(
      ServiceResult::RESTART_REQUIRED, ControlMode::kRestartRequired, false, false, true, true);
    return;
  }
  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kEnabled) {
    finish(
      phase == Phase::kIdle ? ServiceResult::NOT_ENABLED : ServiceResult::NOT_READY,
      current_control_mode_.load(std::memory_order_acquire), false, false, false, true);
    return;
  }
  const ControlMode expected_mode = static_cast<ControlMode>(request->expected_mode.value);
  const ControlMode target_mode = static_cast<ControlMode>(request->target_mode.value);
  const ControlMode current_mode = current_control_mode_.load(std::memory_order_acquire);
  const bool valid_expected = request->expected_mode.value == ModeMessage::FJT_READY ||
                              request->expected_mode.value == ModeMessage::ROLLING_READY;
  const bool valid_target = request->target_mode.value == ModeMessage::FJT_READY ||
                            request->target_mode.value == ModeMessage::ROLLING_READY;
  if (!valid_expected || !valid_target || expected_mode != current_mode) {
    finish(ServiceResult::WRONG_MODE, current_mode, false, false, false, true);
    return;
  }
  if (owner_.load(std::memory_order_acquire) != Owner::kNone) {
    finish(ServiceResult::NOT_READY, current_mode, false, false, false, true);
    return;
  }
  if (target_mode == current_mode) {
    CommandSnapshot current_snapshot;
    if (
      current_mode == ControlMode::kRollingReady &&
      readCommandSnapshot(current_mode, current_snapshot)) {
      response->controller_boot_id.uuid = current_snapshot.controller_boot_id;
    }
    finish(ServiceResult::NONE, current_mode, false, true, false, true);
    return;
  }

  Owner expected_owner = Owner::kNone;
  mode_abort_requested_.store(false, std::memory_order_release);
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kMode)) {
    finish(ServiceResult::NOT_READY, current_mode, false, false, false, true);
    return;
  }
  CommandSnapshot source_snapshot;
  const std::uint8_t admission_result = validateModeSource(current_mode, source_snapshot);
  if (admission_result != ServiceResult::NONE) {
    (void)releaseModeOwnership();
    finish(admission_result, current_mode, false, false, false, true);
    return;
  }
  if (current_mode == ControlMode::kRollingReady) {
    response->controller_boot_id.uuid = source_snapshot.controller_boot_id;
  }
  const std::uint64_t switch_started_ns = steadyNowNs();
  const std::uint8_t switch_result =
    executeModeSwitch(current_mode, target_mode, *response, switch_started_ns);
  (void)releaseModeOwnership();
  const ControlMode resulting_mode = current_control_mode_.load(std::memory_order_acquire);
  finish(
    switch_result, resulting_mode, response->source_controller_deactivated,
    response->target_controller_activated, resulting_mode == ControlMode::kRestartRequired, true);
}

EnableManagerController::SwitchResult EnableManagerController::buildMotionControllerSwitchRequest(
  const controller_manager_msgs::srv::ListControllers::Response & states, bool activate_default,
  controller_manager_msgs::srv::SwitchController::Request & request) const
{
  request.activate_controllers.clear();
  request.deactivate_controllers.clear();
  for (const std::string & registered_name : motion_controller_names_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&registered_name](const auto & candidate) { return candidate.name == registered_name; });
    if (controller == states.controller.end()) {
      return SwitchResult::kFailed;
    }
    const bool is_active = controller->state == "active";
    if (registered_name == default_motion_controller_ && activate_default) {
      if (!is_active) {
        if (controller->state != "inactive") {
          return SwitchResult::kFailed;
        }
        request.activate_controllers.push_back(registered_name);
      }
    } else if (is_active) {
      request.deactivate_controllers.push_back(registered_name);
    }
  }
  return SwitchResult::kSuccess;
}

bool EnableManagerController::registeredMotionControllerStateMatches(
  const controller_manager_msgs::srv::ListControllers::Response & states, bool default_active) const
{
  for (const std::string & registered_name : motion_controller_names_) {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&registered_name](const auto & candidate) { return candidate.name == registered_name; });
    if (controller == states.controller.end()) {
      return false;
    }
    const std::string expected_state =
      default_active && registered_name == default_motion_controller_ ? "active" : "inactive";
    if (controller->state != expected_state) {
      return false;
    }
  }
  return true;
}

EnableManagerController::ControlMode EnableManagerController::detectControlMode(
  const controller_manager_msgs::srv::ListControllers::Response & states) const
{
  const auto state_for = [&states](const std::string & name) -> const std::string * {
    const auto controller = std::find_if(
      states.controller.begin(), states.controller.end(),
      [&name](const auto & candidate) { return candidate.name == name; });
    return controller == states.controller.end() ? nullptr : &controller->state;
  };
  const std::string * fjt_state = state_for(default_motion_controller_);
  const std::string * rolling_state = state_for(rolling_motion_controller_);
  if (fjt_state == nullptr || rolling_state == nullptr) {
    return ControlMode::kRestartRequired;
  }
  const bool fjt_active = *fjt_state == "active";
  const bool rolling_active = *rolling_state == "active";
  const bool fjt_inactive = *fjt_state == "inactive";
  const bool rolling_inactive = *rolling_state == "inactive";
  if (fjt_active && rolling_inactive) {
    return ControlMode::kFjtReady;
  }
  if (fjt_inactive && rolling_active) {
    return ControlMode::kRollingReady;
  }
  if (fjt_inactive && rolling_inactive) {
    return ControlMode::kDisabled;
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
  const ActualSample & previous, const ActualSample & current) const noexcept
{
  if (
    current.steady_time_ns <= previous.steady_time_ns ||
    current.steady_time_ns - previous.steady_time_ns > mode_switch_maximum_sample_period_ns_) {
    return false;
  }
  const double elapsed_seconds =
    static_cast<double>(current.steady_time_ns - previous.steady_time_ns) / 1.0e9;
  for (std::size_t axis = 0U; axis < kMotionAxisCount; ++axis) {
    if (
      !std::isfinite(previous.positions[axis]) || !std::isfinite(current.positions[axis]) ||
      std::abs(current.positions[axis] - previous.positions[axis]) / elapsed_seconds >
        mode_switch_stable_velocity_thresholds_[axis]) {
      return false;
    }
  }
  return true;
}

bool EnableManagerController::sourceWithinTakeoverTolerance(
  const std::array<double, kMotionAxisCount> & source,
  const std::array<double, kMotionAxisCount> & actual) const noexcept
{
  for (std::size_t axis = 0U; axis < kMotionAxisCount; ++axis) {
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
  using ServiceResult = robot_rt_control_interfaces::msg::RollingServiceResult;
  std::uint32_t code = ErrorCode::INTERNAL_ERROR;
  switch (result) {
    case ServiceResult::NONE:
      code = ErrorCode::SUCCESS;
      break;
    case ServiceResult::WRONG_PROTOCOL:
      code = ErrorCode::VERSION_MISMATCH;
      break;
    case ServiceResult::WRONG_REQUEST:
    case ServiceResult::WRONG_CLIENT:
    case ServiceResult::AXIS_SET_MISMATCH:
      code = ErrorCode::INVALID_GOAL;
      break;
    case ServiceResult::WRONG_BOOT:
    case ServiceResult::WRONG_SESSION:
      code = ErrorCode::RETRYABLE_INVALID_GOAL;
      break;
    case ServiceResult::NOT_ENABLED:
    case ServiceResult::SESSION_BUSY:
    case ServiceResult::SESSION_EXISTS:
    case ServiceResult::SOURCE_STATE_STALE:
    case ServiceResult::SOURCE_MOVING:
    case ServiceResult::FEEDBACK_STALE:
    case ServiceResult::SWITCH_TIMEOUT:
    case ServiceResult::NOT_READY:
      code = ErrorCode::NOT_READY;
      break;
    case ServiceResult::WRONG_MODE:
    case ServiceResult::SWITCH_REJECTED:
      code = ErrorCode::GOAL_REJECTED;
      break;
    case ServiceResult::TAKEOVER_MISMATCH:
      code = ErrorCode::RT_TOLERANCE_VIOLATED;
      break;
    case ServiceResult::UNSAFE_HOLD:
      code = ErrorCode::SAFETY_DENIED;
      break;
    case ServiceResult::LIMITS_UNAVAILABLE:
    case ServiceResult::RESTART_REQUIRED:
      code = ErrorCode::VERSION_MISMATCH;
      break;
    default:
      break;
  }
  error.code = code;
  error.retryable = code != ErrorCode::SUCCESS && ((code / 100U) % 10U) == 1U;
  error.severity = code == ErrorCode::SUCCESS
                     ? ErrorInfo::OK
                     : (error.retryable ? ErrorInfo::WARN : ErrorInfo::FAULT);
  error.source = "rt_control";
  error.message = code == ErrorCode::SUCCESS ? "mode request accepted" : "mode request rejected";
  error.detail = "rolling_service_result=" + std::to_string(result);
}

void EnableManagerController::publishModeResult(const ModeService::Response & response)
{
  if (mode_result_publisher_ == nullptr || isZeroIdentifier(response.request_id.uuid)) {
    return;
  }
  ModeResultMessage event;
  event.protocol_major = ModeResultMessage::PROTOCOL_MAJOR;
  event.protocol_minor = ModeResultMessage::PROTOCOL_MINOR;
  event.sequence = mode_result_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  event.request_id = response.request_id;
  event.result = response.result;
  event.mode = response.mode;
  event.controller_boot_id = response.controller_boot_id;
  event.source_controller_deactivated = response.source_controller_deactivated;
  event.target_controller_activated = response.target_controller_activated;
  event.restart_required = response.restart_required;
  mode_result_publisher_->publish(event);
}

}  // namespace ctrl_enable_manager
