#ifndef DM_SWERVE_DRIVER__SWERVE_MODULE_HPP_
#define DM_SWERVE_DRIVER__SWERVE_MODULE_HPP_

#include <array>
#include <optional>

#include "dm_swerve_driver/dm_motor.hpp"
#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct SteeringModuleConfig {
  double gear_ratio{1.0};
  bool inverted{false};
  double zero_offset_rad{0.0};
  double kp{30.0};
  double kd{1.0};
  double kff_omega{0.9};
  double recenter_trigger_fraction{0.9};
  double command_limit_fraction{0.95};
  double max_ff_speed_radps{3.0};
};

struct DriveModuleConfig {
  double gear_ratio{1.0};
  bool inverted{false};
  double wheel_radius_m{0.0};
  double kd{2.0};
  double ks{0.0};
  double kv{0.0};
  double ka{0.0};
  double maximum_wheel_acceleration_mps2{0.0};
};

struct SwerveModuleCommand {
  MitCommand steering;
  MitCommand drive;
  double wheel_speed_mps{0.0};
  double continuous_angle_rad{0.0};
  bool recentered{false};
};

class SwerveModule final {
public:
  SwerveModule(
    DmMotorConfig steering_motor,
    DmMotorConfig drive_motor,
    SteeringModuleConfig steering_config,
    DriveModuleConfig drive_config);

  [[nodiscard]] DmMotor & steering_motor() noexcept;
  [[nodiscard]] const DmMotor & steering_motor() const noexcept;
  [[nodiscard]] DmMotor & drive_motor() noexcept;
  [[nodiscard]] const DmMotor & drive_motor() const noexcept;

  [[nodiscard]] double steering_angle_rad() const;
  [[nodiscard]] double wheel_distance_m() const;
  [[nodiscard]] double wheel_velocity_mps() const;

  [[nodiscard]] SwerveModuleCommand make_command(
    const OptimizedModuleState & target,
    double dt_seconds,
    bool force_drive_zero = false);
  [[nodiscard]] std::array<CanFrame, 2U> encode_command_frames(
    const SwerveModuleCommand & command) const;
  void reset_command_history() noexcept;

private:
  [[nodiscard]] double steering_sign() const noexcept;
  [[nodiscard]] double drive_sign() const noexcept;

  DmMotor steering_motor_;
  DmMotor drive_motor_;
  SteeringModuleConfig steering_config_;
  DriveModuleConfig drive_config_;
  std::optional<double> previous_steering_target_rad_;
  double previous_wheel_speed_mps_{0.0};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SWERVE_MODULE_HPP_
