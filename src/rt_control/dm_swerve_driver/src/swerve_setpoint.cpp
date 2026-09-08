#include "dm_swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] bool finite(double value) noexcept
{
  return std::isfinite(value);
}

}  // namespace

SwerveSetpointGenerator::SwerveSetpointGenerator(
  const SwerveSetpointParameters & parameters)
: parameters_{parameters}
{
  if (!finite(parameters_.alignment_threshold_rad) ||
    parameters_.alignment_threshold_rad < 0.0)
  {
    throw std::invalid_argument{"alignment threshold must be finite and nonnegative"};
  }
  if (!finite(parameters_.flip_hysteresis_rad) ||
    parameters_.flip_hysteresis_rad < 0.0 ||
    parameters_.flip_hysteresis_rad >= kPi / 2.0)
  {
    throw std::invalid_argument{"flip hysteresis must be finite and in [0, pi/2)"};
  }
  if (!finite(parameters_.maximum_steering_slew_radps) ||
    parameters_.maximum_steering_slew_radps <= 0.0)
  {
    throw std::invalid_argument{"maximum steering slew must be finite and positive"};
  }
}

AlignmentResult SwerveSetpointGenerator::generate(
  const std::array<SwerveModuleState, kSwerveModuleCount> & desired,
  const std::array<double, kSwerveModuleCount> & measured_angles_rad,
  double dt_seconds)
{
  if (!finite(dt_seconds) || dt_seconds <= 0.0) {
    throw std::invalid_argument{"setpoint period must be finite and positive"};
  }

  AlignmentResult result{};
  const double maximum_step{parameters_.maximum_steering_slew_radps * dt_seconds};
  for (std::size_t index{0U}; index < result.modules.size(); ++index) {
    const auto optimized = optimize_module(
      desired[index], measured_angles_rad[index], reversed_[index],
      parameters_.flip_hysteresis_rad);
    reversed_[index] = optimized.reversed;

    if (!previous_targets_[index].has_value()) {
      previous_targets_[index] = measured_angles_rad[index];
    }
    const double target_delta{wrap_pi(
        optimized.continuous_angle_rad - *previous_targets_[index])};
    const double limited_target = *previous_targets_[index] +
      std::clamp(target_delta, -maximum_step, maximum_step);
    previous_targets_[index] = limited_target;

    result.modules[index] = optimized;
    result.modules[index].continuous_angle_rad = limited_target;
    result.modules[index].speed_mps *= std::max(0.0, std::cos(optimized.error_rad));
    result.maximum_error_rad = std::max(
      result.maximum_error_rad, std::abs(optimized.error_rad));
  }

  result.gated = result.maximum_error_rad > parameters_.alignment_threshold_rad;
  if (result.gated) {
    for (auto & module : result.modules) {
      module.speed_mps = 0.0;
    }
  }
  return result;
}

void SwerveSetpointGenerator::reset() noexcept
{
  reversed_.fill(false);
  previous_targets_.fill(std::nullopt);
}

}  // namespace dm_swerve_driver
