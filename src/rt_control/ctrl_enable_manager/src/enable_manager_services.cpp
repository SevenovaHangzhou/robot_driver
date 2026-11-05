// Enable/disable/reset services and the controller_manager switch client (service threads).
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
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/joint_control_mode.hpp"
#include "robot_system_interfaces/msg/error_code.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"

namespace ctrl_enable_manager
{

void EnableManagerController::handleEnable(
  const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!enable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
    enable_callback_active_.store(false, std::memory_order_release);
  };

  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }
  if (phase_.load(std::memory_order_acquire) == Phase::kEnabled) {
    fillImmediateResponse(*response, true, Stage::kAlreadyEnabled);
    release_callback();
    return;
  }

  Owner expected_owner = Owner::kNone;
  if (
    phase_.load(std::memory_order_acquire) != Phase::kIdle ||
    !owner_.compare_exchange_strong(expected_owner, Owner::kEnable)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  enable_result_.ready.store(false, std::memory_order_release);
  enable_hardware_ready_.store(false, std::memory_order_release);
  enable_request_.store(true, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (active_.load(std::memory_order_acquire) &&
         !enable_hardware_ready_.load(std::memory_order_acquire) &&
         !enable_result_.ready.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (enable_result_.ready.load(std::memory_order_acquire)) {
    fillResponse(enable_result_, *response);
    release_callback();
    return;
  }
  if (!enable_hardware_ready_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    release_callback();
    return;
  }

  jtc_deactivation_required_.store(true, std::memory_order_release);
  const SwitchResult activation_result = switchJtc(true);
  if (activation_result != SwitchResult::kSuccess) {
    if (activation_result == SwitchResult::kFailed) {
      jtc_deactivation_required_.store(false, std::memory_order_release);
    }
    if (activation_result == SwitchResult::kAmbiguous) {
      restart_required_.store(true, std::memory_order_release);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
    }
    if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
      jtc_activate_failed_request_.store(true, std::memory_order_release);
    }
    if (waitForResult(enable_result_)) {
      fillResponse(enable_result_, *response);
    } else {
      fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
    }
    release_callback();
    return;
  }

  if (phase_.load(std::memory_order_acquire) == Phase::kJtcActivating) {
    jtc_activated_request_.store(true, std::memory_order_release);
  }
  if (waitForResult(enable_result_)) {
    fillResponse(enable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleDisable(
  const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!disable_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
    disable_callback_active_.store(false, std::memory_order_release);
  };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }

  Phase phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::kIdle) {
    fillImmediateResponse(*response, true, Stage::kAlreadyDisabled);
    release_callback();
    return;
  }
  if (
    phase == Phase::kStartupSanitizing || phase == Phase::kEmergencyQuickStop ||
    phase == Phase::kEmergencyDisable || phase == Phase::kJtcActivating ||
    phase == Phase::kDisabling || phase == Phase::kRollback) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  Owner current_owner = owner_.load(std::memory_order_acquire);
  if (motion_mode_switching_ && current_owner == Owner::kMode) {
    mode_abort_requested_.store(true, std::memory_order_release);
    const auto mode_deadline = std::chrono::steady_clock::now() + mode_switch_timeout_ * 5;
    while (active_.load(std::memory_order_acquire) &&
           owner_.load(std::memory_order_acquire) == Owner::kMode &&
           std::chrono::steady_clock::now() < mode_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    current_owner = owner_.load(std::memory_order_acquire);
    phase = phase_.load(std::memory_order_acquire);
    if (current_owner == Owner::kMode) {
      restart_required_.store(true, std::memory_order_release);
      current_control_mode_.store(ControlMode::kRestartRequired, std::memory_order_release);
      jtc_deactivation_required_.store(true, std::memory_order_release);
      emergency_jtc_deactivate_request_.store(true, std::memory_order_release);
      disable_request_.store(true, std::memory_order_release);
      fillImmediateResponse(*response, false, Stage::kRestartRequired);
      release_callback();
      return;
    }
    if (phase != Phase::kEnabled) {
      fillImmediateResponse(*response, false, Stage::kOperationInProgress);
      release_callback();
      return;
    }
  }
  const bool preempt_enable = current_owner == Owner::kEnable &&
                              (phase == Phase::kEnabling || phase == Phase::kInterBatchDelay);
  const bool preempt_reset =
    current_owner == Owner::kReset && (phase == Phase::kResetLow || phase == Phase::kResetHigh);
  if (!preempt_enable && !preempt_reset) {
    Owner expected_owner = Owner::kNone;
    if (!owner_.compare_exchange_strong(expected_owner, Owner::kDisable)) {
      fillImmediateResponse(*response, false, Stage::kOperationInProgress);
      release_callback();
      return;
    }
  }

  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  if (phase == Phase::kEnabled) {
    phase_.store(Phase::kJtcDeactivating, std::memory_order_release);
    if (switchJtc(false) != SwitchResult::kSuccess) {
      jtc_deactivate_failed_.store(true, std::memory_order_release);
      restart_required_.store(true, std::memory_order_release);
      recordFailure(Stage::kJtcDeactivateFailed, -1, -1, 0U);
    } else {
      jtc_deactivation_required_.store(false, std::memory_order_release);
    }
  }
  disable_request_.store(true, std::memory_order_release);
  if (waitForResult(disable_result_)) {
    fillResponse(disable_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

void EnableManagerController::handleResetFault(
  const std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Request>,
  std::shared_ptr<robot_rt_control_interfaces::srv::RtEnable::Response> response)
{
  bool expected_callback = false;
  if (!reset_callback_active_.compare_exchange_strong(expected_callback, true)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    return;
  }
  const auto release_callback = [this]() {
    reset_callback_active_.store(false, std::memory_order_release);
  };
  if (!active_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kControllerInactive);
    release_callback();
    return;
  }
  if (restart_required_.load(std::memory_order_acquire)) {
    fillImmediateResponse(*response, false, Stage::kRestartRequired);
    release_callback();
    return;
  }

  const Phase phase = phase_.load(std::memory_order_acquire);
  if (phase != Phase::kIdle && phase != Phase::kFailed) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }
  Owner expected_owner = Owner::kNone;
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kReset)) {
    fillImmediateResponse(*response, false, Stage::kOperationInProgress);
    release_callback();
    return;
  }

  reset_result_.ready.store(false, std::memory_order_release);
  reset_request_.store(true, std::memory_order_release);
  if (waitForResult(reset_result_)) {
    fillResponse(reset_result_, *response);
  } else {
    fillImmediateResponse(*response, false, Stage::kControllerUpdateTimeout);
  }
  release_callback();
}

bool EnableManagerController::waitForResult(ResultSlot & slot)
{
  const auto deadline = std::chrono::steady_clock::now() + service_result_timeout_;
  while (active_.load(std::memory_order_acquire) && !slot.ready.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return slot.ready.load(std::memory_order_acquire);
}

void EnableManagerController::fillResponse(
  const ResultSlot & slot, robot_rt_control_interfaces::srv::RtEnable::Response & response) const
{
  response.ok = slot.ok.load(std::memory_order_acquire);
  response.failed_batch = slot.failed_batch.load(std::memory_order_acquire);
  const std::int8_t failed_joint = slot.failed_joint.load(std::memory_order_acquire);
  response.failed_joint = failed_joint >= 0 ? joint_names_[failed_joint] : "";
  response.status_word = slot.status_word.load(std::memory_order_acquire);
  response.stage = stageName(slot.stage.load(std::memory_order_acquire));
  if (!response.ok && !expected_modes_.empty() && failed_joint >= 0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Enable failed: stage=%s joint=%s status=0x%04x actual_mode=%d expected_mode=%d (256 means "
      "unavailable)",
      response.stage.c_str(), response.failed_joint.c_str(), response.status_word,
      static_cast<int>(last_failed_mode_.load(std::memory_order_acquire)),
      static_cast<int>(last_expected_mode_.load(std::memory_order_acquire)));
  }
}

void EnableManagerController::reportResult(
  const char * service, robot_rt_control_interfaces::srv::RtEnable::Response & response,
  std::uint32_t failure_code) const
{
  using robot_system_interfaces::msg::ErrorCode;
  using robot_system_interfaces::msg::ErrorInfo;
  static const std::unordered_map<std::string_view, std::uint32_t> kStageCodes{
    {"operation_in_progress", ErrorCode::RT_OPERATION_IN_PROGRESS},
    {"controller_inactive", ErrorCode::RT_ENABLE_MANAGER_NOT_READY},
    {"device_offline", ErrorCode::RT_DEVICE_OFFLINE},
    {"restart_required", ErrorCode::RT_RESTART_REQUIRED},
  };
  std::uint32_t code = ErrorCode::SUCCESS;
  if (!response.ok) {
    const auto found = kStageCodes.find(response.stage);
    code = found != kStageCodes.end() ? found->second : failure_code;
  }
  std::string summary = "stage=" + response.stage;
  if (!response.failed_joint.empty()) {
    summary += " failed_joint=" + response.failed_joint;
  }
  if (response.failed_batch >= 0) {
    summary += " failed_batch=" + std::to_string(response.failed_batch);
  }
  // DREE recovery digit 1 marks retryable codes.
  const bool retryable = code != ErrorCode::SUCCESS && (code / 100U) % 10U == 1U;
  response.error.code = code;
  response.error.message = summary;
  response.error.retryable = retryable;
  response.error.severity = code == ErrorCode::SUCCESS ? ErrorInfo::OK
                            : retryable                ? ErrorInfo::WARN
                                                       : ErrorInfo::FAULT;
  response.error.source = "rt_control/enable_manager";
  RCLCPP_INFO(
    get_node()->get_logger(), "%s %s: %s", service, response.ok ? "accepted" : "rejected",
    summary.c_str());
}

void EnableManagerController::fillImmediateResponse(
  robot_rt_control_interfaces::srv::RtEnable::Response & response, bool ok, Stage stage) const
{
  response.ok = ok;
  response.failed_batch = -1;
  response.failed_joint.clear();
  response.status_word = 0U;
  response.stage = stageName(stage);
}

EnableManagerController::SwitchResult EnableManagerController::switchJtc(bool activate)
{
  // Stationary commissioning exposes no motion command interfaces. Its explicit
  // policy has no trajectory controller to activate; drive-state, timeout,
  // fault, rollback and ordered-disable paths still run normally.
  if (enable_only_) {
    return SwitchResult::kSuccess;
  }
  if (motion_mode_switching_) {
    bool expected = false;
    if (!switch_in_progress_.compare_exchange_strong(expected, true)) {
      return SwitchResult::kAmbiguous;
    }
    const auto clear_in_progress = [this]() {
      switch_in_progress_.store(false, std::memory_order_release);
    };
    if (
      !switch_client_->wait_for_service(std::chrono::milliseconds(0)) ||
      !list_client_->wait_for_service(std::chrono::milliseconds(0))) {
      clear_in_progress();
      return SwitchResult::kAmbiguous;
    }

    auto list_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto list_future = list_client_->async_send_request(list_request);
    const auto list_wait_status =
      list_future.wait_for(std::chrono::duration<double>(controller_switch_timeout_seconds_));
    if (list_wait_status != std::future_status::ready) {
      clear_in_progress();
      return SwitchResult::kAmbiguous;
    }
    const auto list_response = list_future.get();
    if (list_response == nullptr) {
      clear_in_progress();
      return SwitchResult::kAmbiguous;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    const SwitchResult plan_result =
      buildMotionControllerSwitchRequest(*list_response, activate, *request);
    if (plan_result != SwitchResult::kSuccess) {
      clear_in_progress();
      return plan_result;
    }
    if (request->activate_controllers.empty() && request->deactivate_controllers.empty()) {
      const bool state_matches = registeredMotionControllerStateMatches(*list_response, activate);
      clear_in_progress();
      return state_matches ? SwitchResult::kSuccess : SwitchResult::kFailed;
    }
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration::from_seconds(controller_switch_timeout_seconds_);
    auto future = switch_client_->async_send_request(request);
    const auto wait_status =
      future.wait_for(std::chrono::duration<double>(controller_switch_timeout_seconds_));
    SwitchResult result = SwitchResult::kAmbiguous;
    if (wait_status == std::future_status::ready) {
      const auto switch_response = future.get();
      if (switch_response != nullptr && switch_response->ok) {
        auto verification_request =
          std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto verification_future = list_client_->async_send_request(verification_request);
        const auto verification_wait = verification_future.wait_for(
          std::chrono::duration<double>(controller_switch_timeout_seconds_));
        if (verification_wait == std::future_status::ready) {
          const auto verification_response = verification_future.get();
          result = verification_response != nullptr &&
                       registeredMotionControllerStateMatches(*verification_response, activate)
                     ? SwitchResult::kSuccess
                     : SwitchResult::kAmbiguous;
        }
      } else {
        result = SwitchResult::kFailed;
      }
    }
    clear_in_progress();
    return result;
  }
  bool expected = false;
  if (!switch_in_progress_.compare_exchange_strong(expected, true)) {
    return SwitchResult::kAmbiguous;
  }
  const auto clear_in_progress = [this]() {
    switch_in_progress_.store(false, std::memory_order_release);
  };
  if (!switch_client_->wait_for_service(std::chrono::milliseconds(0))) {
    clear_in_progress();
    return SwitchResult::kAmbiguous;
  }

  auto targets = switched_controllers_;
  if (!activate && list_client_->wait_for_service(std::chrono::milliseconds(0))) {
    auto list_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto list_future = list_client_->async_send_request(list_request);
    const auto list_wait_status =
      list_future.wait_for(std::chrono::duration<double>(controller_switch_timeout_seconds_));
    if (list_wait_status == std::future_status::ready) {
      const auto list_response = list_future.get();
      // STRICT rejects deactivating an inactive controller: stop only the active ones.
      targets.clear();
      if (list_response != nullptr) {
        for (const auto & controller : list_response->controller) {
          if (
            controller.state == "active" &&
            std::find(
              switched_controllers_.begin(), switched_controllers_.end(), controller.name) !=
              switched_controllers_.end()) {
            targets.push_back(controller.name);
          }
        }
      }
      if (targets.empty()) {
        clear_in_progress();
        return SwitchResult::kSuccess;
      }
    }
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (activate) {
    request->activate_controllers = targets;
  } else {
    request->deactivate_controllers = targets;
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(controller_switch_timeout_seconds_);
  auto future = switch_client_->async_send_request(request);
  const auto wait_status =
    future.wait_for(std::chrono::duration<double>(controller_switch_timeout_seconds_));
  SwitchResult result = SwitchResult::kAmbiguous;
  if (wait_status == std::future_status::ready) {
    const auto switch_response = future.get();
    result = switch_response != nullptr && switch_response->ok ? SwitchResult::kSuccess
                                                               : SwitchResult::kFailed;
  }
  clear_in_progress();
  return result;
}

void EnableManagerController::handleMotionFault()
{
  if (!motion_fault_request_.load(std::memory_order_acquire)) {
    return;
  }
  if (phase_.load(std::memory_order_acquire) != Phase::kEnabled) {
    motion_fault_request_.store(false, std::memory_order_release);
    return;
  }
  Owner expected_owner = Owner::kNone;
  if (!owner_.compare_exchange_strong(expected_owner, Owner::kDisable)) {
    return;  // An operator request owns the state machine; retry on the next tick.
  }
  motion_fault_request_.store(false, std::memory_order_release);
  // Same ordered sequence as /rt/disable, but the result is a recorded motion fault.
  motion_fault_pending_.store(true, std::memory_order_release);
  disable_result_.ready.store(false, std::memory_order_release);
  jtc_deactivate_failed_.store(false, std::memory_order_release);
  phase_.store(Phase::kJtcDeactivating, std::memory_order_release);
  if (switchJtc(false) != SwitchResult::kSuccess) {
    jtc_deactivate_failed_.store(true, std::memory_order_release);
    restart_required_.store(true, std::memory_order_release);
    recordFailure(Stage::kJtcDeactivateFailed, -1, -1, 0U);
  } else {
    jtc_deactivation_required_.store(false, std::memory_order_release);
  }
  disable_request_.store(true, std::memory_order_release);
}

void EnableManagerController::handleNonRtFaultStop()
{
  handleMotionFault();
  if (switch_in_progress_.load(std::memory_order_acquire)) {
    return;
  }
  if (!emergency_jtc_deactivate_request_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (!jtc_deactivation_required_.load(std::memory_order_acquire)) {
    return;
  }
  if (switchJtc(false) != SwitchResult::kSuccess) {
    restart_required_.store(true, std::memory_order_release);
  } else {
    jtc_deactivation_required_.store(false, std::memory_order_release);
  }
}

}  // namespace ctrl_enable_manager
