#ifndef CTRL_SWERVE__BODY_VELOCITY_SHAPER_HPP_
#define CTRL_SWERVE__BODY_VELOCITY_SHAPER_HPP_

#include <array>
#include <limits>
#include <ruckig/ruckig.hpp>

#include "ctrl_swerve/swerve_kinematics.hpp"

namespace ctrl_swerve
{
// Norm limits on (vx, vy, omega * R). Component limits are divided by sqrt(3),
// conservatively bounding the norm even when the three components change together.
// Infinite acceleration and jerk disable shaping for existing configurations.
struct BodyShaperLimits
{
  double acceleration{std::numeric_limits<double>::infinity()};
  double jerk{std::numeric_limits<double>::infinity()};
  double rotation_radius_m{1.0};
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
