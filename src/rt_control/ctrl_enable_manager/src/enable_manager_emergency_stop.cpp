#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <algorithm>
#include <iterator>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/qos.hpp"

namespace ctrl_enable_manager
{
bool EnableManagerController::configureEmergencyStop()
{
  emergency_stop_response_ = get_node()->get_parameter("emergency_stop_response").as_bool();
  const auto timeout = get_node()->get_parameter("emergency_stop_timeout_ms").as_int();
  const auto topic = get_node()->get_parameter("emergency_stop_topic").as_string();
  if (
    topic.empty() || timeout < 0 || timeout > 2147483647LL ||
    (emergency_stop_response_ && timeout == 0)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "emergency_stop_topic is required; stop response requires explicit positive "
      "emergency_stop_timeout_ms (maximum 2147483647)");
    return false;
  }
  emergency_stop_subscription_.reset();
  emergency_stop_input_.configure(static_cast<std::uint64_t>(timeout) * 1000000ULL);
  emergency_stop_latched_.store(false, std::memory_order_release);
  emergency_stop_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = emergency_stop_group_;
  emergency_stop_subscription_ = get_node()->create_subscription<EmergencyStopInput::Message>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](EmergencyStopInput::Message::ConstSharedPtr message) {
      emergency_stop_input_.observe(*message, steadyNowNs(), get_node()->now().nanoseconds());
    },
    options);
  return true;
}

bool EnableManagerController::emergencyStopBlocksEnable() const noexcept
{
  return emergency_stop_response_ && (emergency_stop_latched_.load(std::memory_order_acquire) ||
                                      !emergency_stop_input_.released(steadyNowNs()));
}

bool EnableManagerController::emergencyStopBlocksReset() const noexcept
{
  return emergency_stop_response_ &&
         (!emergency_stop_input_.released(steadyNowNs()) ||
          emergency_jtc_deactivate_request_.load(std::memory_order_acquire) ||
          jtc_deactivation_required_.load(std::memory_order_acquire) ||
          switch_in_progress_.load(std::memory_order_acquire));
}

bool EnableManagerController::configureEmergencyDisableVoltage()
{
  const auto joints =
    get_node()->get_parameter("emergency_disable_voltage_joints").as_string_array();
  std::vector<std::uint8_t> candidate(joint_names_.size(), 0U);
  for (const auto & joint : joints) {
    const auto found = std::find(joint_names_.begin(), joint_names_.end(), joint);
    const auto axis = static_cast<std::size_t>(std::distance(joint_names_.begin(), found));
    if (found == joint_names_.end() || candidate[axis] != 0U) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "emergency_disable_voltage_joints must be a unique managed-joint subset");
      return false;
    }
    candidate[axis] = 1U;
  }
  emergency_disable_voltage_ = std::move(candidate);
  return true;
}

std::uint16_t EnableManagerController::emergencyStopCommand(
  std::size_t axis, DriveState state) const noexcept
{
  if (axis < emergency_disable_voltage_.size() && emergency_disable_voltage_[axis] != 0U) {
    // Disable Voltage with the quick-stop bit high: no Quick Stop for these drives.
    return state == DriveState::kSwitchOnDisabled ? 0x0000U : 0x0005U;
  }
  if (state == DriveState::kOperationEnabled || state == DriveState::kQuickStopActive) {
    return 0x0002U;
  }
  return state == DriveState::kSwitchedOn ? 0x0006U : 0x0000U;
}

}  // namespace ctrl_enable_manager
