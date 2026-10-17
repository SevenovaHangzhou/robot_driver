#ifndef SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_
#define SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "swerve_driver/control_core.hpp"
#include "swerve_driver/chassis_runtime.hpp"
#include "swerve_driver/odometry_covariance.hpp"

namespace swerve_driver
{
class SwerveController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Command
  {
    ChassisSpeeds speeds{};
    int64_t received_ns{0};
    uint64_t generation{0};
    bool valid{false};
    uint64_t navigation_generation{0};
  };
  struct ImuSample
  {
    YawSample yaw{};
    int64_t received_ns{0};
    uint64_t generation{0};
    bool valid{false};
  };
  static int64_t steady_now() noexcept;
  ModuleFeedbackArray read_feedback() const;
  [[nodiscard]] bool write_output(const ControlOutput & output);
  void stop_outputs();
  void accept_command(const geometry_msgs::msg::Twist & message, uint64_t generation);
  void accept_imu(const sensor_msgs::msg::Imu & message, uint64_t generation);
  void publish_diagnostics();

  ChassisRuntimeFeedback read_runtime_feedback(double now);
  bool write_runtime_output(const ChassisRuntimeOutput & output);
  std::shared_ptr<ChassisRuntime> runtime_;
  bool was_navigation_{false};
  // Robot-model joint q -> chassis-plane steering heading (bound from the chassis renderer).
  std::array<double, 4> steering_to_yaw_sign_{{1.0, 1.0, 1.0, 1.0}};
  // Model-joint positions last successfully written to the steering command interfaces.
  std::array<double, 4> last_steering_commands_{};
  CoreConfig config_;
  std::unique_ptr<ControlCore> core_;
  std::vector<std::string> command_names_, state_names_;
  std::vector<hardware_interface::LoanedCommandInterface *> commands_;
  std::vector<hardware_interface::LoanedStateInterface *> states_;
  std::array<OdometryCovariances, 8> covariances_{};
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  realtime_tools::RealtimeBuffer<ImuSample> imu_buffer_;
  std::atomic_bool active_{false}, ready_{false};
  std::atomic<ControlStatus> status_{ControlStatus::inactive};
  // Motion fault: latched until deactivation; ready for ordered disable once wheels stopped.
  void track_fault(ControlStatus status, const ModuleFeedbackArray & feedback, int64_t now);
  std::atomic_bool fault_latched_{false}, fault_stop_ready_{false};
  int64_t fault_since_ns_{0}, wheels_stopped_since_ns_{0};
  double fault_stop_speed_{0.01}, fault_stop_hold_{0.05}, fault_stop_timeout_{2.0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_stop_publisher_;
  rclcpp::TimerBase::SharedPtr fault_stop_timer_;
  std::atomic<unsigned int> slipping_mask_{0U};
  std::mutex lifecycle_mutex_;
  uint64_t generation_{0};
  int64_t block_before_ns_{0};
  double feedback_timeout_{0.0}, publish_elapsed_{0.0};
  double imu_timeout_{0.0}, max_imu_yaw_step_{0.0}, quaternion_tolerance_{0.0};
  bool imu_enabled_{false};
  std::string imu_frame_;
  ControlOutput last_output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_
