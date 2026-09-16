#ifndef ROBOT_HW_CANOPEN__SWERVE_ENCODER_SYSTEM_HPP_
#define ROBOT_HW_CANOPEN__SWERVE_ENCODER_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "canopen_ros2_control/canopen_system.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "robot_hw_canopen/swerve_encoder_feedback.hpp"

namespace robot_hw_canopen
{

class SwerveEncoderSystem : public canopen_ros2_control::CanopenSystem
{
public:
  SwerveEncoderSystem();
  ~SwerveEncoderSystem() noexcept override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void on_rpdo_received(
    ros2_canopen::COData data, std::uint8_t id,
    std::chrono::steady_clock::time_point received_at) override;

private:
  std::unique_ptr<SwerveEncoderFeedback> feedback_;
  std::array<double, kSwerveEncoderCount> position_rad_{};
  std::array<double, kSwerveEncoderCount> feedback_age_ms_{};
};

}  // namespace robot_hw_canopen

#endif  // ROBOT_HW_CANOPEN__SWERVE_ENCODER_SYSTEM_HPP_
