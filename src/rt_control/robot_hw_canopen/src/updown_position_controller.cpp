#include "robot_hw_canopen/updown_position_controller.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hw_canopen
{
namespace
{
constexpr std::size_t kPositionCommand = 0U;
constexpr std::size_t kTpdoIndexCommand = 1U;
constexpr std::size_t kTpdoSubindexCommand = 2U;
constexpr std::size_t kTpdoDataCommand = 3U;
constexpr std::size_t kTpdoOneShotCommand = 4U;
}  // namespace

controller_interface::CallbackReturn UpdownPositionController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UpdownPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {
    "updown/position", "updown/tpdo/index", "updown/tpdo/subindex", "updown/tpdo/data",
    "updown/tpdo/ons"};
  return configuration;
}

controller_interface::InterfaceConfiguration
UpdownPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {"updown/position"};
  return configuration;
}

controller_interface::CallbackReturn UpdownPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  subscription_active_.store(false, std::memory_order_release);
  received_sequence_.store(0U, std::memory_order_release);
  handled_sequence_ = 0U;
  command_buffer_.writeFromNonRT(BufferedCommand{});

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  command_subscription_ = get_node()->create_subscription<robot_interfaces::msg::UpdownCommand>(
    "~/command", qos,
    [this](const robot_interfaces::msg::UpdownCommand::SharedPtr message)
    {
      if (!subscription_active_.load(std::memory_order_acquire))
      {
        return;
      }

      BufferedCommand command;
      command.expected_start_position = message->expected_start_position;
      command.position = message->position;
      command.profile_velocity_raw = message->profile_velocity_raw;
      command.sequence = received_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
      command_buffer_.writeFromNonRT(command);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UpdownPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 5U || state_interfaces_.size() != 1U)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected five command interfaces and one state interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double actual_position = state_interfaces_.front().get_value();
  if (!std::isfinite(actual_position))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot preload updown from a non-finite position");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces_[kPositionCommand].set_value(actual_position);
  command_interfaces_[kTpdoIndexCommand].set_value(0.0);
  command_interfaces_[kTpdoSubindexCommand].set_value(0.0);
  command_interfaces_[kTpdoDataCommand].set_value(0.0);
  command_interfaces_[kTpdoOneShotCommand].set_value(std::numeric_limits<double>::quiet_NaN());

  handled_sequence_ = received_sequence_.load(std::memory_order_acquire);
  command_buffer_.writeFromNonRT(BufferedCommand{});
  subscription_active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UpdownPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscription_active_.store(false, std::memory_order_release);
  const double actual_position = state_interfaces_.front().get_value();
  if (std::isfinite(actual_position))
  {
    command_interfaces_[kPositionCommand].set_value(actual_position);
  }
  command_interfaces_[kTpdoOneShotCommand].set_value(std::numeric_limits<double>::quiet_NaN());
  handled_sequence_ = received_sequence_.load(std::memory_order_acquire);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UpdownPositionController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const BufferedCommand * const command = command_buffer_.readFromRT();
  if (command == nullptr || command->sequence == 0U || command->sequence == handled_sequence_)
  {
    return controller_interface::return_type::OK;
  }

  const double actual_position = state_interfaces_.front().get_value();
  const bool values_are_finite = std::isfinite(command->expected_start_position) &&
                                 std::isfinite(command->position) &&
                                 std::isfinite(actual_position);
  const bool target_is_in_range =
    command->position >= kMinimumPosition && command->position <= kMaximumPosition;
  const bool expected_start_matches = values_are_finite &&
                                      std::abs(command->expected_start_position - actual_position) <=
                                        kExpectedStartTolerance;

  handled_sequence_ = command->sequence;
  if (!values_are_finite || !target_is_in_range || !expected_start_matches)
  {
    return controller_interface::return_type::OK;
  }

  command_interfaces_[kTpdoIndexCommand].set_value(kProfileVelocityIndex);
  command_interfaces_[kTpdoSubindexCommand].set_value(0.0);
  command_interfaces_[kTpdoDataCommand].set_value(
    static_cast<double>(command->profile_velocity_raw));
  command_interfaces_[kTpdoOneShotCommand].set_value(1.0);
  command_interfaces_[kPositionCommand].set_value(command->position);
  return controller_interface::return_type::OK;
}

}  // namespace robot_hw_canopen

PLUGINLIB_EXPORT_CLASS(
  robot_hw_canopen::UpdownPositionController, controller_interface::ControllerInterface)
