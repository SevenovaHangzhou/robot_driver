#include "ctrl_enable_manager/enable_manager_controller.hpp"

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

}  // namespace ctrl_enable_manager
