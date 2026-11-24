#ifndef CTRL_SWERVE__CONTROL_CORE_HPP_
#define CTRL_SWERVE__CONTROL_CORE_HPP_

#include <array>
#include <limits>
#include <optional>
#include "ctrl_swerve/body_velocity_shaper.hpp"
#include "ctrl_swerve/swerve_odometry.hpp"

namespace ctrl_swerve
{
struct CoreConfig
{
  std::array<Translation2d, 4> locations{};
  std::array<double, 4> wheel_radius{};
  std::array<double, 4> steering_min{};
  std::array<double, 4> steering_max{};
  std::array<double, 4> steering_limit_margin{};
  std::array<double, 4> steering_limit_tolerance{};
  double max_linear_speed{0.0}, max_angular_speed{0.0}, max_wheel_speed{0.0};
  double max_wheel_acceleration{0.0}, velocity_deadband{0.0};
  double max_encoder_difference{0.0}, max_update_period{0.0};
  double slip_residual_threshold{0.0};
  // External encoder monitor: invalid, out of range or > max_encoder_difference from the
  // motor-derived angle for longer than this window raises feedback_fault (seconds).
  double encoder_fault_window{0.05};
  // XY norm and angular acceleration are independent; jerk uses equivalent wheel radius.
  // All three +infinity disables body shaping for non-navigation configurations.
  double max_linear_acceleration{std::numeric_limits<double>::infinity()};
  double max_angular_acceleration{std::numeric_limits<double>::infinity()};
  double max_body_jerk{std::numeric_limits<double>::infinity()};
  double max_rotation_speed{1.5}, max_rotation_acceleration{1.5};
  double rotation_stop_speed{0.01}, rotation_stop_hold{0.05};
  SwerveSetpointParameters setpoint{};
};

struct ModuleFeedback
{
  // Motor-derived output-axis angle (control, hold, odometry) and the external output-axis
  // angle (monitor only: startup reference and plausibility) stay separate.
  double steering_position{0.0}, steering_angle{0.0};
  double wheel_position{0.0}, wheel_velocity{0.0};
  // valid: drive feedback fresh. encoder_valid: external encoder sample fresh.
  bool valid{false}, enabled{false}, encoder_valid{false};
};
using ModuleFeedbackArray = std::array<ModuleFeedback, 4>;

struct YawSample
{
  double yaw_rad;
  double rate_radps;
};
enum class ControlStatus {
  inactive,
  running,
  alignment_gated,
  command_timeout,
  feedback_fault,
  invalid_command,
  steering_limit,
  steering_hold_fault
};

struct ControlOutput
{
  std::array<double, 4> steering_position{}, drive_velocity{};
  Pose2d pose{};
  ChassisSpeeds measured_twist{};
  ControlStatus status{ControlStatus::inactive};
  size_t valid_modules{0};
  bool imu_fallback{true};
  std::array<bool, 4> slipping_modules{};
  bool slip_detected{false};
};

struct ModuleExecutionSnapshot
{
  bool ready{false}, reversed{false};
  double steering_actual{0.0}, steering_encoder{0.0};
  double steering_target{0.0}, steering_reference{0.0};
  double steering_rate{0.0}, steering_acceleration{0.0}, steering_error{0.0};
  double wheel_actual{0.0}, wheel_target{0.0}, wheel_reference{0.0};
  double wheel_position{0.0};
  double wheel_max_speed{0.0}, wheel_max_acceleration{0.0};
  double steering_min{0.0}, steering_max{0.0};
  double steering_max_rate{0.0}, steering_max_acceleration{0.0}, steering_max_jerk{0.0};
  double wheel_radius{0.0};
  Translation2d location{};
};

// Fixed-size SI state. Steering goals precede profiling; wheel references are the final
// applied output in m/s. Measured fields are NaN when their feedback is unavailable.
struct ExecutionSnapshot
{
  ControlStatus status{ControlStatus::inactive};
  bool ready{false}, feedback_valid{false}, rotation_active{false}, rotation_requested{false};
  bool gated{false}, waiting{false};
  ChassisSpeeds body_reference{}, body_acceleration{}, body_measured{};
  double max_linear_speed{0.0}, max_angular_speed{0.0};
  double max_linear_acceleration{0.0}, max_angular_acceleration{0.0}, max_body_jerk{0.0};
  std::array<ModuleExecutionSnapshot, 4> modules{};
};

class ControlCore
{
public:
  explicit ControlCore(const CoreConfig & config);
  bool activate(const ModuleFeedbackArray & feedback);
  void deactivate() noexcept;
  bool feedback_ready(const ModuleFeedbackArray & feedback) const noexcept;
  const std::array<double, 4> & hold_positions() const noexcept { return last_steering_targets_; }
  bool hold_fault_latched() const noexcept { return hold_fault_latched_; }
  ControlOutput update(
    const ModuleFeedbackArray & feedback, ChassisSpeeds command, bool command_fresh, double dt,
    std::optional<YawSample> imu = std::nullopt, bool rotation_requested = false);
  const ExecutionSnapshot & execution_snapshot() const noexcept { return snapshot_; }

private:
  ControlOutput update_command(
    const ModuleFeedbackArray & feedback, ChassisSpeeds command, bool command_fresh, double dt);
  void refresh_snapshot(const ModuleFeedbackArray & feedback) noexcept;
  void update_rotation_capability(
    const ModuleFeedbackArray & feedback, const ChassisSpeeds & command, bool command_fresh,
    double dt, bool requested) noexcept;
  bool measurement_valid(const ModuleFeedback & feedback) const noexcept;
  bool module_ready(const ModuleFeedback & feedback, std::size_t index) const noexcept;
  bool encoder_consistent(const ModuleFeedback & feedback, std::size_t index) const noexcept;
  // Returns true when an external encoder stayed inconsistent longer than the window.
  bool encoder_monitor_tripped(const ModuleFeedbackArray & feedback, double dt) noexcept;
  bool hold_deviation(
    const ModuleFeedbackArray & feedback, const std::array<bool, 4> & holding) const noexcept;
  std::array<SwerveModulePosition, 4> positions(const ModuleFeedbackArray & feedback) const;
  void observe(const ModuleFeedbackArray & feedback, std::optional<YawSample> imu);
  ControlOutput stop(const ModuleFeedbackArray & feedback, ControlStatus status);
  // Ordinary stop: ramp steering rate and wheel speed down, then latch the reached targets.
  ControlOutput ramp_stop(const ModuleFeedbackArray & feedback, ControlStatus status, double dt);

  CoreConfig config_;
  SwerveSetpointGenerator setpoints_;
  BodyVelocityShaper body_shaper_;
  bool body_alignment_waiting_{true};
  std::optional<SwerveOdometry> odometry_;
  std::array<SwerveModulePosition, 4> previous_positions_{};
  std::array<double, 4> previous_speeds_{};
  std::array<double, 4> last_steering_targets_{};
  std::array<bool, 4> holding_modules_{};
  std::array<double, 4> encoder_bad_time_{};
  ControlOutput output_;
  ExecutionSnapshot snapshot_{};
  bool rotation_active_{false}, rotation_requested_{false};
  double rotation_stopped_time_{0.0};
  double yaw_{0.0}, last_imu_yaw_{0.0};
  bool active_{false}, imu_fallback_{true}, hold_fault_latched_{false};
};
}  // namespace ctrl_swerve
#endif  // CTRL_SWERVE__CONTROL_CORE_HPP_
