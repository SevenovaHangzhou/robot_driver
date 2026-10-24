#ifndef SWERVE_DRIVER__CONTROL_CORE_HPP_
#define SWERVE_DRIVER__CONTROL_CORE_HPP_

#include <array>
#include <limits>
#include <optional>
#include "swerve_driver/body_velocity_shaper.hpp"
#include "swerve_driver/swerve_odometry.hpp"

namespace swerve_driver
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
  // Chassis twist S-curve before inverse kinematics (wheel-rim units, see BodyShaperLimits).
  // Infinite acceleration disables it; max_wheel_acceleration stays the per-wheel backstop.
  double max_body_acceleration{std::numeric_limits<double>::infinity()};
  double max_body_jerk{std::numeric_limits<double>::infinity()};
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

struct YawSample {double yaw_rad; double rate_radps;};
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

class ControlCore
{
public:
  explicit ControlCore(const CoreConfig & config);
  bool activate(const ModuleFeedbackArray & feedback);
  void deactivate() noexcept;
  bool feedback_ready(const ModuleFeedbackArray & feedback) const noexcept;
  const std::array<double, 4> & hold_positions() const noexcept { return last_steering_targets_; }
  bool hold_fault_latched() const noexcept { return hold_fault_latched_; }
  bool mode_switch_feedback_ready(const ModuleFeedbackArray & feedback) const noexcept;
  bool switch_alignment_policy(
    const ModuleFeedbackArray & feedback, double stationary_speed, double threshold,
    double release) noexcept;
  ControlOutput update(const ModuleFeedbackArray & feedback, ChassisSpeeds command,
    bool command_fresh, double dt, std::optional<YawSample> imu = std::nullopt);

private:
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
  double yaw_{0.0}, last_imu_yaw_{0.0};
  bool active_{false}, imu_fallback_{true}, hold_fault_latched_{false};
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__CONTROL_CORE_HPP_
