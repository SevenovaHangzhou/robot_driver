#ifndef CTRL_WHEEL_OBSERVATION_BROADCASTER__WHEEL_OBSERVATION_BROADCASTER_HPP_
#define CTRL_WHEEL_OBSERVATION_BROADCASTER__WHEEL_OBSERVATION_BROADCASTER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ctrl_wheel_observation_broadcaster/channel_tracker.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "robot_rt_control_interfaces/msg/wheel_observation_array.hpp"

namespace ctrl_wheel_observation_broadcaster
{

// Read-only: claims state interfaces only, so it runs whether or not the chassis is enabled
// and whichever motion controller owns the wheels.
class WheelObservationBroadcaster final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  using Message = robot_rt_control_interfaces::msg::WheelObservationArray;
  struct Module
  {
    // Message slot, FL/FR/RL/RR from the calibrated location, not the wiring index.
    std::size_t slot{0U};
    double yaw_sign{1.0};
    ChannelTracker drive, steering_motor, steering_encoder;
    double drive_sequence{0.0}, steering_sequence{0.0}, encoder_age_ms{0.0};
    double drive_position{0.0};
  };
  double state(std::size_t module, std::size_t offset) const;
  void start_session(std::int64_t time_ns);

  std::vector<std::string> state_names_;
  std::array<Module, 4> modules_{};
  std::array<double, 4> x_{}, y_{}, radius_{};
  double timeout_ms_{0.0}, publish_period_{0.0}, publish_elapsed_{0.0};
  std::string session_prefix_, calibration_id_;
  std::uint64_t session_count_{0U}, sequence_{0U};
  std::int64_t last_time_ns_{0};
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> realtime_publisher_;
};

}  // namespace ctrl_wheel_observation_broadcaster

#endif  // CTRL_WHEEL_OBSERVATION_BROADCASTER__WHEEL_OBSERVATION_BROADCASTER_HPP_
