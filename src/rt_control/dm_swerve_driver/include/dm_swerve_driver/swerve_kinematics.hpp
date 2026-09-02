#ifndef DM_SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_
#define DM_SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_

#include <array>
#include <cstddef>
#include <optional>

namespace dm_swerve_driver {

inline constexpr std::size_t kSwerveModuleCount{4U};
inline constexpr double kPi{3.141592653589793238462643383279502884};

struct ChassisSpeeds {
  double vx_mps{0.0};
  double vy_mps{0.0};
  double omega_radps{0.0};
};

struct Translation2d {
  double x{0.0};
  double y{0.0};
};

struct ChassisDelta {
  double dx_m{0.0};
  double dy_m{0.0};
  double dtheta_rad{0.0};
};

struct SwerveModuleState {
  double speed_mps{0.0};
  double angle_rad{0.0};
};

struct SwerveModulePosition {
  double distance_m{0.0};
  double angle_rad{0.0};
  bool valid{true};
};

struct OptimizedModuleState {
  double speed_mps{0.0};
  double continuous_angle_rad{0.0};
  double error_rad{0.0};
};

struct AlignmentResult {
  std::array<OptimizedModuleState, kSwerveModuleCount> modules{};
  double maximum_error_rad{0.0};
  bool gated{false};
};

[[nodiscard]] double wrap_pi(double angle_rad) noexcept;
[[nodiscard]] ChassisSpeeds discretize(const ChassisSpeeds & speeds, double dt_seconds);

[[nodiscard]] std::array<SwerveModuleState, kSwerveModuleCount> inverse_kinematics(
  const ChassisSpeeds & speeds,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  const std::array<double, kSwerveModuleCount> & previous_angles_rad,
  double velocity_deadband_mps);

void desaturate_wheel_speeds(
  std::array<SwerveModuleState, kSwerveModuleCount> & states,
  double maximum_speed_mps);

[[nodiscard]] OptimizedModuleState optimize_module(
  const SwerveModuleState & desired, double current_unwrapped_angle_rad);

[[nodiscard]] AlignmentResult optimize_and_apply_alignment(
  const std::array<SwerveModuleState, kSwerveModuleCount> & desired,
  const std::array<double, kSwerveModuleCount> & current_unwrapped_angles_rad,
  double alignment_threshold_rad);

[[nodiscard]] std::optional<Translation2d> wheel_translation_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  double delta_yaw_rad);

[[nodiscard]] std::optional<ChassisDelta> wheel_chassis_delta_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_
