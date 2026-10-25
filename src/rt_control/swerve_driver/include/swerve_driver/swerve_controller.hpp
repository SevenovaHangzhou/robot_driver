#ifndef SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_
#define SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
  using ModeService = std_srvs::srv::SetBool;
  using ModeServer = rclcpp::Service<ModeService>;
  enum class ModeResult {
    Applied,
    NotReady,
    FeedbackInvalid,
    NotStopped,
    Expired,
    NotNavigation,
    FreshZeroRequired,
    InvalidParameters
  };
  struct ModeRequest
  {
    uint64_t sequence{0}, generation{0};
    int64_t deadline_ns{0};
    double threshold{0.0}, release{0.0};
    bool align_first{true};
  };
  struct ModeResponse
  {
    ModeServer::SharedPtr service;
    rmw_request_id_t header;
    ModeRequest request;
  };
  void request_drive_mode(
    ModeServer::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
    ModeService::Request::SharedPtr request);
  void mode_timer();
  void process_drive_mode(
    const ModuleFeedbackArray & feedback, const Command & command, bool command_fresh,
    bool navigation, int64_t now);
  static const char * mode_result_text(ModeResult result) noexcept;
  realtime_tools::RealtimeBuffer<ModeRequest> mode_request_buffer_;
  std::atomic<uint64_t> mode_pending_{0}, mode_completed_{0};
  std::atomic<ModeResult> mode_result_{ModeResult::NotReady};
  uint64_t mode_sequence_{0};                  // non-RT, protected by lifecycle_mutex_
  int64_t mode_stationary_since_ns_{0};        // RT only
  std::optional<ModeResponse> mode_response_;  // non-RT, lifecycle_mutex_
  ModeServer::SharedPtr mode_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_publisher_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
  std::optional<bool> published_mode_;
  int64_t mode_published_at_ns_{0};
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
  // First latched fault, for diagnostics: status and per-module feedback reasons
  // (4 bits per module: 1 stale, 2 not enabled/mode, 4 encoder vs motor > limit, 8 non-finite).
  std::atomic<int> fault_status_code_{-1};
  std::atomic<uint32_t> fault_module_bits_{0U};
  std::atomic<int32_t> fault_max_difference_mdeg_{0};
  // Largest |measurement - steering target| at the latch (motor and external encoder), mdeg.
  std::atomic<int32_t> fault_target_deviation_mdeg_{0};
  std::atomic<int> fault_target_module_{0};
  double fault_stop_speed_{0.01}, fault_stop_hold_{0.05}, fault_stop_timeout_{2.0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_stop_publisher_;
  rclcpp::TimerBase::SharedPtr fault_stop_timer_;
  std::atomic<unsigned int> slipping_mask_{0U};
  std::mutex lifecycle_mutex_;
  // Operator-tunable steering motion (user 2026-10-18): checked against hard limits on set,
  // applied at the next activation (the console only changes them while disabled).
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr steering_parameter_callback_;
  bool apply_steering_motion_parameters();
  uint64_t generation_{0};
  int64_t block_before_ns_{0};
  double feedback_timeout_{0.0}, publish_elapsed_{0.0};
  double imu_timeout_{0.0}, max_imu_yaw_step_{0.0}, quaternion_tolerance_{0.0};
  bool imu_enabled_{false};
  // Startup default is configured by drive_mode; runtime changes occur only at an RT boundary.
  std::atomic_bool align_first_{true};
  std::string imu_frame_;
  ControlOutput last_output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  // Control-axis feedback, independent of URDF joint numbering and heading signs.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr chassis_feedback_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_feedback_;
  std::array<double, 4> published_steering_{};
  int64_t feedback_published_ns_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__SWERVE_CONTROLLER_HPP_
