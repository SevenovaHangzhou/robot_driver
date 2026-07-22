#ifndef ROBOT_HW_CANOPEN__UPDOWN_POSITION_CONTROLLER_HPP_
#define ROBOT_HW_CANOPEN__UPDOWN_POSITION_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_interfaces/msg/updown_command.hpp"

namespace robot_hw_canopen
{

class UpdownPositionController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct BufferedCommand
  {
    double expected_start_position{0.0};
    double position{0.0};
    std::uint32_t profile_velocity_raw{0U};
    std::uint64_t sequence{0U};
  };

  static constexpr double kMinimumPosition = 0.0;
  static constexpr double kMaximumPosition = 0.8;
  static constexpr double kExpectedStartTolerance = 0.05;
  static constexpr double kProfileVelocityIndex = 0x6081;

  realtime_tools::RealtimeBuffer<BufferedCommand> command_buffer_;
  rclcpp::Subscription<robot_interfaces::msg::UpdownCommand>::SharedPtr command_subscription_;
  std::atomic<std::uint64_t> received_sequence_{0U};
  std::atomic<bool> subscription_active_{false};
  std::uint64_t handled_sequence_{0U};
};

}  // namespace robot_hw_canopen

#endif  // ROBOT_HW_CANOPEN__UPDOWN_POSITION_CONTROLLER_HPP_
