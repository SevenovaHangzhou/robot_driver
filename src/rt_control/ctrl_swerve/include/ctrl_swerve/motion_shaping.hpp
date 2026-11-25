#ifndef CTRL_SWERVE__MOTION_SHAPING_HPP_
#define CTRL_SWERVE__MOTION_SHAPING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ctrl_swerve::motion_shaping
{
// The published prediction core's planar/angular envelope. General retargets use
// a conservative box; collinear moves receive the full planar norm budget.
template <class Input>
void prepare(Input & input, double planar, double angular, double radius, double jerk) noexcept
{
  input.max_acceleration = {planar / std::sqrt(2.0), planar / std::sqrt(2.0), angular * radius};
  input.max_jerk.fill(jerk / std::sqrt(3.0));
  std::size_t axis = 0;
  for (std::size_t i = 1; i < 3; ++i) {
    if (std::abs(input.target_velocity[i]) > std::abs(input.target_velocity[axis])) {
      axis = i;
    }
  }
  const double magnitude = std::abs(input.target_velocity[axis]);
  if (magnitude < 1e-12) {
    return;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    const double ratio = input.target_velocity[i] / input.target_velocity[axis];
    if (
      std::abs(input.current_velocity[i] - ratio * input.current_velocity[axis]) > 1e-10 ||
      std::abs(input.current_acceleration[i] - ratio * input.current_acceleration[axis]) > 1e-10) {
      return;
    }
  }
  const double xy = std::hypot(input.target_velocity[0], input.target_velocity[1]) / magnitude;
  double acceleration = xy > 1e-12 ? planar / xy : std::numeric_limits<double>::infinity();
  const double angular_weight = std::abs(input.target_velocity[2]) / magnitude;
  if (angular_weight > 1e-12) {
    acceleration = std::min(acceleration, angular * radius / angular_weight);
  }
  double limited_jerk = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 3; ++i) {
    const double weight = std::abs(input.target_velocity[i]) / magnitude;
    if (weight > 1e-12) {
      limited_jerk = std::min(limited_jerk, input.max_jerk[i] / weight);
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    const double weight = std::abs(input.target_velocity[i]) / magnitude;
    if (weight > 1e-12) {
      input.max_acceleration[i] = acceleration * weight;
      input.max_jerk[i] = limited_jerk * weight;
    }
  }
}

// At a reduced envelope boundary Ruckig may still be removing old acceleration.
// Preserve the hard norm and angular bounds, then restart its cached trajectory.
template <class Input, class Output>
bool finish(
  const Input & input, Output & output, double dt, double planar, double angular,
  double radius) noexcept
{
  std::array<double, 3> delta;
  for (std::size_t i = 0; i < 3; ++i) {
    delta[i] = output.new_velocity[i] - input.current_velocity[i];
  }
  const double scale = std::max(
    {1.0, std::hypot(delta[0], delta[1]) / (planar * dt),
     std::abs(delta[2]) / (angular * radius * dt),
     std::hypot(output.new_acceleration[0], output.new_acceleration[1]) / planar,
     std::abs(output.new_acceleration[2]) / (angular * radius)});
  if (scale <= 1.0 + 1e-10) {
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    output.new_velocity[i] = input.current_velocity[i] + delta[i] / scale;
    output.new_acceleration[i] /= scale;
  }
  return true;
}
}  // namespace ctrl_swerve::motion_shaping
#endif  // CTRL_SWERVE__MOTION_SHAPING_HPP_
