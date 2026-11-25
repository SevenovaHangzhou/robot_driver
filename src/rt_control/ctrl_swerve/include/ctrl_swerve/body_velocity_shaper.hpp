#ifndef CTRL_SWERVE__BODY_VELOCITY_SHAPER_HPP_
#define CTRL_SWERVE__BODY_VELOCITY_SHAPER_HPP_

#include <array>
#include <limits>
#include <ruckig/ruckig.hpp>

#include "ctrl_swerve/swerve_kinematics.hpp"

namespace ctrl_swerve
{
// Separate XY norm and angular acceleration limits; jerk bounds (vx, vy, omega * R).
// All three limits infinite disables shaping for existing configurations.
struct BodyShaperLimits
{
  double linear_acceleration{std::numeric_limits<double>::infinity()};
  double jerk{std::numeric_limits<double>::infinity()};
  double rotation_radius_m{1.0};
  double angular_acceleration{std::numeric_limits<double>::infinity()};
};

class BodyVelocityShaper final
{
public:
  explicit BodyVelocityShaper(const BodyShaperLimits & limits);
  [[nodiscard]] bool enabled() const noexcept;
  // Fixed-size velocity-mode trajectory; no waypoints, network calls or dynamic DOFs.
  [[nodiscard]] ChassisSpeeds step(const ChassisSpeeds & target, double dt_seconds) noexcept;
  void reset() noexcept;
  [[nodiscard]] bool valid() const noexcept { return valid_; }
  void set_angular_acceleration(double limit) noexcept;
  [[nodiscard]] ChassisSpeeds velocity() const noexcept
  {
    return {
      input_.current_velocity[0], input_.current_velocity[1],
      input_.current_velocity[2] / limits_.rotation_radius_m};
  }
  [[nodiscard]] ChassisSpeeds body_acceleration() const noexcept
  {
    return {
      input_.current_acceleration[0], input_.current_acceleration[1],
      input_.current_acceleration[2] / limits_.rotation_radius_m};
  }
  [[nodiscard]] const std::array<double, 3> & acceleration() const noexcept
  {
    return input_.current_acceleration;
  }

private:
  BodyShaperLimits limits_;
  ruckig::Ruckig<3> generator_;
  ruckig::InputParameter<3> input_;
  ruckig::OutputParameter<3> output_;
  bool valid_{true};
};
}  // namespace ctrl_swerve
#endif  // CTRL_SWERVE__BODY_VELOCITY_SHAPER_HPP_
