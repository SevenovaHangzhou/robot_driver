#include "dm_swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] bool finite(double value) noexcept
{
  return std::isfinite(value);
}

void validate_state(const SwerveModuleState & state)
{
  if (!finite(state.speed_mps) || !finite(state.angle_rad)) {
    throw std::invalid_argument{"swerve module state must be finite"};
  }
}

[[nodiscard]] std::optional<double> nearest_equivalent_in_limits(
  double angle_rad,
  double current_angle_rad,
  const SteeringAngleLimits & limits) noexcept
{
  constexpr double full_turn{2.0 * kPi};
  const double safe_minimum{limits.minimum_rad + limits.margin_rad};
  const double safe_maximum{limits.maximum_rad - limits.margin_rad};
  const double first_turn{std::ceil((safe_minimum - angle_rad) / full_turn)};
  const double last_turn{std::floor((safe_maximum - angle_rad) / full_turn)};
  if (first_turn > last_turn) {
    return std::nullopt;
  }
  const double nearest_turn{std::clamp(
      std::round((current_angle_rad - angle_rad) / full_turn), first_turn, last_turn)};
  return angle_rad + nearest_turn * full_turn;
}

}  // namespace

bool valid_steering_angle_limits(const SteeringAngleLimits & limits) noexcept
{
  const double physical_range{limits.maximum_rad - limits.minimum_rad};
  const double safe_range{physical_range - 2.0 * limits.margin_rad};
  return finite(limits.minimum_rad) && finite(limits.maximum_rad) &&
         finite(limits.margin_rad) && limits.margin_rad >= 0.0 &&
         finite(limits.measurement_tolerance_rad) &&
         limits.measurement_tolerance_rad >= 0.0 &&
         physical_range <= 2.0 * kPi && safe_range >= kPi;
}

bool steering_angle_within_limits(
  double angle_rad, const SteeringAngleLimits & limits) noexcept
{
  const double safe_minimum{limits.minimum_rad + limits.margin_rad};
  const double safe_maximum{limits.maximum_rad - limits.margin_rad};
  return finite(angle_rad) && valid_steering_angle_limits(limits) &&
         angle_rad >= safe_minimum && angle_rad <= safe_maximum;
}

bool steering_measurement_within_tolerance(
  double angle_rad, const SteeringAngleLimits & limits) noexcept
{
  return finite(angle_rad) && valid_steering_angle_limits(limits) &&
         angle_rad >= limits.minimum_rad - limits.measurement_tolerance_rad &&
         angle_rad <= limits.maximum_rad + limits.measurement_tolerance_rad;
}

double clamp_steering_measurement_to_safe_range(
  double angle_rad, const SteeringAngleLimits & limits)
{
  if (!finite(angle_rad) || !valid_steering_angle_limits(limits)) {
    throw std::invalid_argument{"steering measurement and limits must be valid"};
  }
  return std::clamp(
    angle_rad,
    limits.minimum_rad + limits.margin_rad,
    limits.maximum_rad - limits.margin_rad);
}

OptimizedModuleState optimize_module(
  const SwerveModuleState & desired, double current_angle_rad)
{
  return optimize_module(desired, current_angle_rad, false, 0.0);
}

OptimizedModuleState optimize_module(
  const SwerveModuleState & desired,
  double current_angle_rad,
  bool previous_reversed,
  double hysteresis_rad,
  const SteeringAngleLimits & limits)
{
  validate_state(desired);
  if (!finite(hysteresis_rad) || hysteresis_rad < 0.0 || hysteresis_rad >= kPi / 2.0) {
    throw std::invalid_argument{"flip hysteresis must be finite and in [0, pi/2)"};
  }
  if (!valid_steering_angle_limits(limits)) {
    throw std::invalid_argument{"steering angle limits must span [pi, 2*pi]"};
  }
  if (!steering_measurement_within_tolerance(current_angle_rad, limits)) {
    throw std::out_of_range{"current steering angle exceeds physical limit tolerance"};
  }
  const double planning_angle{
    clamp_steering_measurement_to_safe_range(current_angle_rad, limits)};
  const double forward_base{wrap_pi(desired.angle_rad)};
  const auto forward = nearest_equivalent_in_limits(
    forward_base, planning_angle, limits);
  const auto reverse = nearest_equivalent_in_limits(
    forward_base + kPi, planning_angle, limits);
  if (!forward.has_value() && !reverse.has_value()) {
    throw std::out_of_range{"no equivalent steering target is inside mechanical limits"};
  }

  bool reversed{previous_reversed};
  const auto & current = reversed ? reverse : forward;
  const auto & alternative = reversed ? forward : reverse;
  if (!current.has_value()) {
    reversed = !reversed;
  } else if (alternative.has_value() &&
    std::abs(*alternative - planning_angle) + hysteresis_rad <
    std::abs(*current - planning_angle))
  {
    reversed = !reversed;
  }
  const double target_angle{*(reversed ? reverse : forward)};
  const double error{target_angle - planning_angle};
  return OptimizedModuleState{
    reversed ? -desired.speed_mps : desired.speed_mps,
    target_angle, error, reversed};
}

AlignmentResult optimize_and_apply_alignment(
  const std::array<SwerveModuleState, kSwerveModuleCount> & desired,
  const std::array<double, kSwerveModuleCount> & current_angles_rad,
  double alignment_threshold_rad)
{
  if (!finite(alignment_threshold_rad) || alignment_threshold_rad < 0.0) {
    throw std::invalid_argument{"alignment threshold must be finite and nonnegative"};
  }
  AlignmentResult result{};
  for (std::size_t index{0U}; index < result.modules.size(); ++index) {
    result.modules[index] = optimize_module(desired[index], current_angles_rad[index]);
    result.maximum_error_rad = std::max(
      result.maximum_error_rad, std::abs(result.modules[index].error_rad));
    result.modules[index].speed_mps *= std::max(0.0, std::cos(result.modules[index].error_rad));
  }
  result.gated = result.maximum_error_rad > alignment_threshold_rad;
  if (result.gated) {
    for (auto & module : result.modules) {
      module.speed_mps = 0.0;
    }
  }
  return result;
}

}  // namespace dm_swerve_driver
