#ifndef RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_BROADCASTER_HPP_
#define RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_BROADCASTER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/timer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rt_control_semantic_components/force_torque_sensor.hpp"
#include "rt_force_torque_broadcaster/force_torque_processor.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace rt_force_torque_broadcaster
{

class ForceTorqueBroadcaster final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration()
  const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
  const override;
  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void publish_calibration();
  void clear_runtime_resources();

  std::unique_ptr<rt_control_semantic_components::ForceTorqueSensor> sensor_;
  std::unique_ptr<ForceTorqueProcessor> processor_;
  std::vector<std::string> state_interface_names_;
  hardware_interface::LoanedStateInterface * link_interface_{nullptr};
  hardware_interface::LoanedStateInterface * al_state_interface_{nullptr};

  std::string sensor_name_;
  std::string frame_id_;
  std::string wrench_topic_;
  std::string raw_topic_;
  std::string calibration_topic_;
  std::string diagnostic_name_;
  std::string link_interface_name_;
  std::string al_state_interface_name_;
  std::string startup_id_;
  std::string snapshot_source_;
  std::string validity_policy_;
  std::vector<std::int64_t> decimals_;
  std::vector<std::int64_t> unit_codes_;
  std::int64_t minimum_auxiliary_value_{0};
  std::int64_t maximum_auxiliary_value_{0};
  bool calibration_valid_{false};

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
  realtime_wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>>
  realtime_raw_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    calibration_publisher_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  std::atomic_bool active_{false};
  std::atomic_bool startup_invalidated_{false};
  std::atomic<std::uint64_t> diagnostic_generation_{0U};
  std::uint64_t published_diagnostic_generation_{0U};
  std::mutex lifecycle_mutex_;
};

}  // namespace rt_force_torque_broadcaster

#endif  // RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_BROADCASTER_HPP_
