#ifndef DM_SWERVE_DRIVER__SWERVE_ODOMETRY_HPP_
#define DM_SWERVE_DRIVER__SWERVE_ODOMETRY_HPP_

#include <array>

#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct Pose2d {
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};
};

struct BodyDelta {
  double dx_m{0.0};
  double dy_m{0.0};
  double dtheta_rad{0.0};
};

[[nodiscard]] Pose2d pose_exp(const BodyDelta & delta) noexcept;
[[nodiscard]] BodyDelta pose_log(const Pose2d & transform) noexcept;
[[nodiscard]] Pose2d integrate_pose(
  const Pose2d & current, const BodyDelta & delta, double absolute_heading_rad) noexcept;

class SwerveOdometry final {
public:
  SwerveOdometry(
    std::array<Translation2d, kSwerveModuleCount> module_locations,
    double initial_yaw_rad,
    std::array<SwerveModulePosition, kSwerveModuleCount> initial_positions,
    const Pose2d & initial_pose = {});

  void reset(
    const Pose2d & pose, double yaw_rad,
    const std::array<SwerveModulePosition, kSwerveModuleCount> & positions);

  [[nodiscard]] Pose2d update(
    double yaw_rad,
    const std::array<SwerveModulePosition, kSwerveModuleCount> & positions);

  [[nodiscard]] const Pose2d & pose() const noexcept;

private:
  std::array<Translation2d, kSwerveModuleCount> module_locations_{};
  std::array<SwerveModulePosition, kSwerveModuleCount> previous_positions_{};
  Pose2d pose_{};
  double previous_yaw_rad_{0.0};
  double yaw_offset_rad_{0.0};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SWERVE_ODOMETRY_HPP_
