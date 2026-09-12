#ifndef SWERVE_DRIVER__CONTROL_CORE_HPP_
#define SWERVE_DRIVER__CONTROL_CORE_HPP_

#include <array>
#include <optional>
#include "swerve_driver/swerve_odometry.hpp"

namespace swerve_driver
{
struct CoreConfig
{
  std::array<Translation2d, 4> locations{};
  std::array<double, 4> wheel_radius{};
  std::array<double, 4> steering_min{};
  std::array<double, 4> steering_max{};
  double max_linear_speed{0.0}, max_angular_speed{0.0}, max_wheel_speed{0.0};
  double max_wheel_acceleration{0.0}, velocity_deadband{0.0};
  double max_encoder_difference{0.0}, max_update_period{0.0};
  SwerveSetpointParameters setpoint{};
};

struct ModuleFeedback
{
  // Motor-derived output-axis angle and external output-axis angle stay separate.
  double steering_position{0.0}, steering_angle{0.0};
  double wheel_position{0.0}, wheel_velocity{0.0};
  bool valid{false}, enabled{false};
};
using ModuleFeedbackArray = std::array<ModuleFeedback, 4>;

struct YawSample {double yaw_rad; double rate_radps;};
enum class ControlStatus
{
  inactive, running, alignment_gated, command_timeout,
  feedback_fault, invalid_command, steering_limit
};

struct ControlOutput
{
  std::array<double, 4> steering_position{}, drive_velocity{};
  Pose2d pose{};
  ChassisSpeeds measured_twist{};
  ControlStatus status{ControlStatus::inactive};
  size_t valid_modules{0};
  bool imu_fallback{true};
};

class ControlCore
{
public:
  explicit ControlCore(const CoreConfig & config);
  bool activate(const ModuleFeedbackArray & feedback);
  void deactivate() noexcept;
  bool feedback_ready(const ModuleFeedbackArray & feedback) const noexcept;
  const std::array<double, 4> & hold_positions() const noexcept {return last_motor_positions_;}
  ControlOutput update(const ModuleFeedbackArray & feedback, ChassisSpeeds command,
    bool command_fresh, double dt, std::optional<YawSample> imu = std::nullopt);

private:
  bool measurement_valid(const ModuleFeedback & feedback) const noexcept;
  void remember_positions(const ModuleFeedbackArray & feedback) noexcept;
  std::array<SwerveModulePosition, 4> positions(const ModuleFeedbackArray & feedback) const;
  void observe(const ModuleFeedbackArray & feedback, std::optional<YawSample> imu);
  ControlOutput stop(const ModuleFeedbackArray & feedback, ControlStatus status);

  CoreConfig config_;
  SwerveSetpointGenerator setpoints_;
  std::optional<SwerveOdometry> odometry_;
  std::array<SwerveModulePosition, 4> previous_positions_{};
  std::array<double, 4> previous_speeds_{};
  std::array<double, 4> last_motor_positions_{};
  ControlOutput output_;
  double yaw_{0.0}, last_imu_yaw_{0.0};
  bool active_{false}, imu_fallback_{true};
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__CONTROL_CORE_HPP_
