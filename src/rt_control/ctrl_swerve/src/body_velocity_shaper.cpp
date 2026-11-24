#include "ctrl_swerve/body_velocity_shaper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ctrl_swerve
{
BodyVelocityShaper::BodyVelocityShaper(const BodyShaperLimits & limits) : limits_{limits}
{
  const bool disabled = limits_.linear_acceleration == std::numeric_limits<double>::infinity() &&
                        limits_.angular_acceleration == std::numeric_limits<double>::infinity() &&
                        limits_.jerk == std::numeric_limits<double>::infinity();
  if (
    !disabled &&
    (!std::isfinite(limits_.linear_acceleration) || limits_.linear_acceleration <= 0.0 ||
     !std::isfinite(limits_.angular_acceleration) || limits_.angular_acceleration <= 0.0 ||
     !std::isfinite(limits_.jerk) || limits_.jerk <= 0.0)) {
    throw std::invalid_argument{"body limits must all be finite positive or all +infinity"};
  }
  if (!std::isfinite(limits_.rotation_radius_m) || limits_.rotation_radius_m <= 0.0) {
    throw std::invalid_argument{"body shaper rotation radius must be finite positive"};
  }
  input_.control_interface = ruckig::ControlInterface::Velocity;
  // Phase synchronization preserves component ratios when states are collinear (including
  // rest-to-target). Ruckig falls back to time synchronization for arbitrary moving retargets.
  input_.synchronization = ruckig::Synchronization::Phase;
  input_.max_acceleration = {
    limits_.linear_acceleration / std::sqrt(2.0), limits_.linear_acceleration / std::sqrt(2.0),
    limits_.angular_acceleration * limits_.rotation_radius_m};
  input_.max_jerk.fill(limits_.jerk / std::sqrt(3.0));
  reset();
}

bool BodyVelocityShaper::enabled() const noexcept
{
  return std::isfinite(limits_.linear_acceleration);
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

void BodyVelocityShaper::set_angular_acceleration(double limit) noexcept
{
  if (!enabled()) {
    return;
  }
  const double bound = limit * limits_.rotation_radius_m;
  if (
    bound < input_.max_acceleration[2] &&
    input_.current_acceleration[2] * input_.current_velocity[2] > 0.0) {
    input_.current_acceleration[2] = 0.0;
  }
  input_.max_acceleration[2] = bound;
  // Revocation may lower the bound mid-ramp. Keep velocity continuous; a safety-limit
  // transition may step acceleration rather than retaining the revoked acceleration.
  input_.current_acceleration[2] = std::clamp(input_.current_acceleration[2], -bound, bound);
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
}  // namespace ctrl_swerve
