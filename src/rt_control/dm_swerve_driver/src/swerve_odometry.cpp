#include "dm_swerve_driver/swerve_odometry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
namespace {

constexpr double kSmallAngleThreshold{1e-9};

void validate_pose(const Pose2d & pose)
{
  if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
    !std::isfinite(pose.heading_rad))
  {
    throw std::invalid_argument{"odometry pose must be finite"};
  }
}

void validate_yaw(double yaw_rad)
{
  if (!std::isfinite(yaw_rad)) {
    throw std::invalid_argument{"odometry yaw must be finite"};
  }
}

}  // namespace

Pose2d pose_exp(const BodyDelta & delta) noexcept
{
  const double theta{delta.dtheta_rad};
  double sine_scale{0.0};
  double cosine_scale{0.0};
  if (std::abs(theta) < kSmallAngleThreshold) {
    sine_scale = 1.0 - theta * theta / 6.0;
    cosine_scale = theta / 2.0;
  } else {
    sine_scale = std::sin(theta) / theta;
    cosine_scale = (1.0 - std::cos(theta)) / theta;
  }
  return Pose2d{
    delta.dx_m * sine_scale - delta.dy_m * cosine_scale,
    delta.dx_m * cosine_scale + delta.dy_m * sine_scale,
    theta};
}

BodyDelta pose_log(const Pose2d & transform) noexcept
{
  const double theta{wrap_pi(transform.heading_rad)};
  const double half_theta{theta / 2.0};
  const double coefficient = std::abs(theta) < kSmallAngleThreshold ?
    1.0 - theta * theta / 12.0 : half_theta / std::tan(half_theta);
  return BodyDelta{
    coefficient * transform.x_m + half_theta * transform.y_m,
    -half_theta * transform.x_m + coefficient * transform.y_m,
    theta};
}

Pose2d integrate_pose(
  const Pose2d & current, const BodyDelta & delta, double absolute_heading_rad) noexcept
{
  const Pose2d transform{pose_exp(delta)};
  const double cosine{std::cos(current.heading_rad)};
  const double sine{std::sin(current.heading_rad)};
  return Pose2d{
    current.x_m + transform.x_m * cosine - transform.y_m * sine,
    current.y_m + transform.x_m * sine + transform.y_m * cosine,
    absolute_heading_rad};
}

SwerveOdometry::SwerveOdometry(
  std::array<Translation2d, kSwerveModuleCount> module_locations,
  double initial_yaw_rad,
  std::array<SwerveModulePosition, kSwerveModuleCount> initial_positions,
  const Pose2d & initial_pose)
: module_locations_{std::move(module_locations)}
{
  reset(initial_pose, initial_yaw_rad, initial_positions);
}

void SwerveOdometry::reset(
  const Pose2d & pose, double yaw_rad,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & positions)
{
  validate_pose(pose);
  validate_yaw(yaw_rad);
  for (const auto & location : module_locations_) {
    if (!std::isfinite(location.x) || !std::isfinite(location.y)) {
      throw std::invalid_argument{"odometry module locations must be finite"};
    }
  }
  pose_ = pose;
  previous_yaw_rad_ = yaw_rad;
  yaw_offset_rad_ = pose.heading_rad - yaw_rad;
  previous_positions_ = positions;
}

Pose2d SwerveOdometry::update(
  double yaw_rad,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & positions)
{
  validate_yaw(yaw_rad);
  const double delta_yaw{wrap_pi(yaw_rad - previous_yaw_rad_)};
  const auto translation = wheel_translation_from_position_deltas(
    previous_positions_, positions, module_locations_, delta_yaw);
  const BodyDelta delta{
    translation.has_value() ? translation->x : 0.0,
    translation.has_value() ? translation->y : 0.0,
    delta_yaw};
  pose_ = integrate_pose(pose_, delta, yaw_offset_rad_ + yaw_rad);
  previous_positions_ = positions;
  previous_yaw_rad_ = yaw_rad;
  return pose_;
}

const Pose2d & SwerveOdometry::pose() const noexcept
{
  return pose_;
}

}  // namespace dm_swerve_driver
