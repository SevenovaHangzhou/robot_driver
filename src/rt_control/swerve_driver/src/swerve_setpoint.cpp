#include "swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve_driver {
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
  if (!finite(parameters_.translation_heading_epsilon_rad) ||
    parameters_.translation_heading_epsilon_rad < 0.0 ||
    parameters_.translation_heading_epsilon_rad >= kPi / 2.0)
  {
    throw std::invalid_argument{
            "translation heading epsilon must be finite and in [0, pi/2)"};
  }
  if (!finite(parameters_.steering_angle_deadband_rad) ||
    parameters_.steering_angle_deadband_rad < 0.0 ||
    parameters_.steering_angle_deadband_rad >= kPi / 2.0)
  {
    throw std::invalid_argument{
            "steering angle deadband must be finite and in [0, pi/2)"};
  }
  for (const auto & limits : parameters_.angle_limits) {
    if (!valid_steering_angle_limits(limits)) {
      throw std::invalid_argument{"steering angle limits must span [pi, 2*pi]"};
    }
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
  std::array<double, kSwerveModuleCount> planning_angles{};
  double common_heading{0.0};
  double maximum_speed{-1.0};
  for (std::size_t index{0U}; index < desired.size(); ++index) {
    if (!finite(desired[index].speed_mps) || !finite(desired[index].angle_rad)) {
      throw std::invalid_argument{"swerve module state must be finite"};
    }
    if (!steering_measurement_within_tolerance(
        measured_angles_rad[index], parameters_.angle_limits[index]))
    {
      throw std::out_of_range{"steering measurement exceeds physical limit tolerance"};
    }
    planning_angles[index] = clamp_steering_measurement_to_safe_range(
      measured_angles_rad[index], parameters_.angle_limits[index]);
    if (std::abs(desired[index].speed_mps) > maximum_speed) {
      maximum_speed = std::abs(desired[index].speed_mps);
      common_heading = desired[index].angle_rad;
    }
  }

  double maximum_heading_delta{0.0};
  for (const auto & state : desired) {
    maximum_heading_delta = std::max(
      maximum_heading_delta, std::abs(wrap_pi(state.angle_rad - common_heading)));
  }
  const bool translation_requested = maximum_speed > 1e-6 &&
    maximum_heading_delta <= parameters_.translation_heading_epsilon_rad;
  std::array<std::optional<double>, kSwerveModuleCount> translation_forward{};
  std::array<std::optional<double>, kSwerveModuleCount> translation_reverse{};
  bool all_forward_feasible{translation_requested};
  bool all_reverse_feasible{translation_requested};
  double forward_cost{0.0};
  double reverse_cost{0.0};
  if (translation_requested) {
    for (std::size_t index{0U}; index < desired.size(); ++index) {
      translation_forward[index] = nearest_equivalent_steering_target(
        wrap_pi(common_heading), planning_angles[index], parameters_.angle_limits[index]);
      translation_reverse[index] = nearest_equivalent_steering_target(
        wrap_pi(common_heading) + kPi, planning_angles[index], parameters_.angle_limits[index]);
      all_forward_feasible = all_forward_feasible && translation_forward[index].has_value();
      all_reverse_feasible = all_reverse_feasible && translation_reverse[index].has_value();
      if (translation_forward[index].has_value()) {
        forward_cost += std::abs(*translation_forward[index] - planning_angles[index]);
      }
      if (translation_reverse[index].has_value()) {
        reverse_cost += std::abs(*translation_reverse[index] - planning_angles[index]);
      }
    }
  }
  const bool global_translation_branch{all_forward_feasible || all_reverse_feasible};
  bool translation_reversed{false};
  if (global_translation_branch) {
    if (!all_forward_feasible) {
      translation_reversed = true;
    } else if (!all_reverse_feasible) {
      translation_reversed = false;
    } else if (!translation_reversed_.has_value()) {
      translation_reversed = reverse_cost < forward_cost;
    } else {
      translation_reversed = *translation_reversed_;
      const double current_cost{translation_reversed ? reverse_cost : forward_cost};
      const double alternative_cost{translation_reversed ? forward_cost : reverse_cost};
      const double hysteresis{
        parameters_.flip_hysteresis_rad * static_cast<double>(desired.size())};
      if (alternative_cost + hysteresis < current_cost) {
        translation_reversed = !translation_reversed;
      }
    }
    translation_reversed_ = translation_reversed;
  } else {
    translation_reversed_.reset();
  }

  for (std::size_t index{0U}; index < result.modules.size(); ++index) {
    OptimizedModuleState optimized;
    if (global_translation_branch) {
      const double target_angle{*(translation_reversed ?
          translation_reverse[index] : translation_forward[index])};
      optimized = OptimizedModuleState{
        translation_reversed ? -desired[index].speed_mps : desired[index].speed_mps,
        target_angle, target_angle - planning_angles[index], translation_reversed};
    } else {
      optimized = optimize_module(
        desired[index], planning_angles[index], reversed_[index],
        parameters_.flip_hysteresis_rad, parameters_.angle_limits[index]);
    }
    reversed_[index] = optimized.reversed;

    if (!previous_targets_[index].has_value()) {
      previous_targets_[index] = planning_angles[index];
    }
    double target_delta{optimized.target_angle_rad - *previous_targets_[index]};
    if (std::abs(target_delta) < parameters_.steering_angle_deadband_rad) {
      target_delta = 0.0;
    }
    const double limited_target = *previous_targets_[index] +
      std::clamp(target_delta, -maximum_step, maximum_step);
    previous_targets_[index] = limited_target;

    result.modules[index] = optimized;
    result.modules[index].target_angle_rad = limited_target;
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
  translation_reversed_.reset();
  previous_targets_.fill(std::nullopt);
}

}  // namespace swerve_driver
