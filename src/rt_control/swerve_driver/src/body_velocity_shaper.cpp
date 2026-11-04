#include "swerve_driver/body_velocity_shaper.hpp"

#include <cmath>
#include <stdexcept>

namespace swerve_driver
{
BodyVelocityShaper::BodyVelocityShaper(const BodyShaperLimits & limits) : limits_{limits}
{
  const bool disabled = limits_.acceleration == std::numeric_limits<double>::infinity() &&
                        limits_.jerk == std::numeric_limits<double>::infinity();
  if (
    !disabled && (!std::isfinite(limits_.acceleration) || limits_.acceleration <= 0.0 ||
                  !std::isfinite(limits_.jerk) || limits_.jerk <= 0.0)) {
    throw std::invalid_argument{"body limits must both be finite positive or both +infinity"};
  }
  if (!std::isfinite(limits_.rotation_radius_m) || limits_.rotation_radius_m <= 0.0) {
    throw std::invalid_argument{"body shaper rotation radius must be finite positive"};
  }
  input_.control_interface = ruckig::ControlInterface::Velocity;
  // Phase synchronization preserves component ratios when states are collinear (including
  // rest-to-target). Ruckig falls back to time synchronization for arbitrary moving retargets.
  input_.synchronization = ruckig::Synchronization::Phase;
  input_.max_acceleration.fill(limits_.acceleration / std::sqrt(3.0));
  input_.max_jerk.fill(limits_.jerk / std::sqrt(3.0));
  reset();
}

bool BodyVelocityShaper::enabled() const noexcept
{
  return std::isfinite(limits_.acceleration);
}

void BodyVelocityShaper::reset() noexcept
{
  input_.current_position.fill(0.0);
  input_.current_velocity.fill(0.0);
  input_.current_acceleration.fill(0.0);
  input_.target_velocity.fill(0.0);
  input_.target_acceleration.fill(0.0);
  generator_.reset();
  valid_ = true;
}

ChassisSpeeds BodyVelocityShaper::step(const ChassisSpeeds & target, double dt) noexcept
{
  input_.target_velocity = {
    target.vx_mps, target.vy_mps, target.omega_radps * limits_.rotation_radius_m};
  if (!std::isfinite(dt) || dt <= 0.0) {
    valid_ = false;
    return {};
  }
  for (const auto value : input_.target_velocity) {
    if (!std::isfinite(value)) {
      valid_ = false;
      return {};
    }
  }
  if (!valid_) {
    return {};
  }
  if (!enabled()) {
    input_.current_velocity = input_.target_velocity;
    input_.current_acceleration.fill(0.0);
    return target;
  }
  generator_.delta_time = dt;
  const auto result = generator_.update(input_, output_);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    valid_ = false;
    return {};
  }
  output_.pass_to_input(input_);
  return {
    output_.new_velocity[0], output_.new_velocity[1],
    output_.new_velocity[2] / limits_.rotation_radius_m};
}
}  // namespace swerve_driver
