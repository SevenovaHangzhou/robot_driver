#ifndef ROBOT_HW_CAN__DAMIAO_SYSTEM_HPP_
#define ROBOT_HW_CAN__DAMIAO_SYSTEM_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <optional>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "robot_hw_can/head_state_machine.hpp"
#include "robot_hw_can/protocol.hpp"
#include "robot_hw_can/socketcan.hpp"

namespace robot_hw_can
{

class DamiaoSystem final : public hardware_interface::SystemInterface
{
public:
  ~DamiaoSystem() noexcept override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Motor
  {
    std::uint16_t can_id{0U};
    std::uint16_t master_id{0U};
    double velocity_limit{0.0};
    double command_min{0.0};
    double command_max{0.0};
    TrapezoidalProfile expected_profile{};
    MotorLimits limits{};
    double command{0.0};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double fault_code{0.0};
    double mos_temperature{0.0};
    double motor_temperature{0.0};
    double feedback_age_ms{0.0};
    double enabled{0.0};
    std::uint8_t status{0U};
    std::chrono::steady_clock::time_point received_at{};
  };

  [[nodiscard]] std::uint32_t read_register(std::size_t motor_index, Register register_id);
  [[nodiscard]] bool await_feedback(std::size_t motor_index);
  void observe_frame(const CanFrame & frame, std::chrono::steady_clock::time_point received_at);
  void send_special(SpecialCommand command);
  void send_disable() noexcept;
  void update_control_states() noexcept;
  [[nodiscard]] bool all_position_interfaces(
    const std::vector<std::string> & interfaces) const;
  [[nodiscard]] std::size_t relevant_interface_count(
    const std::vector<std::string> & interfaces) const;

  SocketCan socket_;
  std::string can_interface_;
  int configure_timeout_ms_{0};
  int feedback_timeout_ms_{0};
  int transition_timeout_ms_{0};
  int disabled_poll_interval_ms_{0};
  double command_rate_hz_{0.0};
  std::size_t max_rx_frames_per_cycle_{0U};
  std::vector<Motor> motors_;
  bool hardware_active_{false};
  bool command_interfaces_active_{false};
  double enable_request_{0.0};
  double reset_generation_{0.0};
  double phase_state_{0.0};
  double fault_latched_state_{0.0};
  double handled_reset_generation_state_{0.0};
  double motion_allowed_state_{0.0};
  std::optional<HeadStateMachine> state_machine_;
  std::optional<CommandRateLimiter> command_rate_limiter_;
  std::optional<CommandRateLimiter> special_rate_limiter_;
  std::chrono::steady_clock::time_point last_disabled_poll_at_{};
};

}  // namespace robot_hw_can

#endif  // ROBOT_HW_CAN__DAMIAO_SYSTEM_HPP_
