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
  if (
    !std::isnan(parameters_.alignment_release_rad) &&
    (!finite(parameters_.alignment_release_rad) || parameters_.alignment_release_rad < 0.0 ||
     parameters_.alignment_release_rad > parameters_.alignment_threshold_rad)) {
    throw std::invalid_argument{"alignment release must be in [0, alignment threshold]"};
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
  if (
    std::isnan(parameters_.maximum_steering_acceleration_radps2) ||
    parameters_.maximum_steering_acceleration_radps2 <= 0.0) {
    throw std::invalid_argument{"maximum steering acceleration must be positive"};
  }
  if (
    std::isnan(parameters_.maximum_steering_jerk_radps3) ||
    parameters_.maximum_steering_jerk_radps3 <= 0.0 ||
    (finite(parameters_.maximum_steering_jerk_radps3) &&
     !finite(parameters_.maximum_steering_acceleration_radps2))) {
    throw std::invalid_argument{
      "maximum steering jerk must be positive and needs a finite acceleration limit"};
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
  const std::array<double, kSwerveModuleCount> & measured_angles_rad, double dt_seconds,
  const std::array<bool, kSwerveModuleCount> & hold_modules)
{
  if (!finite(dt_seconds) || dt_seconds <= 0.0) {
    throw std::invalid_argument{"setpoint period must be finite and positive"};
  }

  AlignmentResult result{};
  const double maximum_rate{parameters_.maximum_steering_slew_radps};
  const double rate_step{parameters_.maximum_steering_acceleration_radps2 * dt_seconds};
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
    if (hold_modules[index]) {
      if (!previous_targets_[index].has_value()) {
        previous_targets_[index] = desired[index].angle_rad;
        steering_rates_[index] = 0.0;
        steering_accelerations_[index] = 0.0;
      }
      // Ramp an in-flight steering rate to zero, then latch the reached target.
      if (finite(parameters_.maximum_steering_jerk_radps3)) {
        *previous_targets_[index] += jerk_limited_stop(index, dt_seconds);
      } else {
        const double rate = steering_rates_[index];
        const double slower = std::max(0.0, std::abs(rate) - rate_step);
        steering_rates_[index] = std::copysign(slower, rate);
        *previous_targets_[index] += steering_rates_[index] * dt_seconds;
      }
      const double hold = *previous_targets_[index];
      result.modules[index] = {0.0, hold, hold - planning_angles[index], reversed_[index]};
      result.maximum_error_rad =
        std::max(result.maximum_error_rad, std::abs(result.modules[index].error_rad));
      continue;
    }
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
      steering_rates_[index] = 0.0;
      steering_accelerations_[index] = 0.0;
    }
    double target_delta{optimized.target_angle_rad - *previous_targets_[index]};
    if (std::abs(target_delta) < parameters_.steering_angle_deadband_rad) {
      target_delta = 0.0;
    }
    if (finite(parameters_.maximum_steering_jerk_radps3)) {
      const double limited_target =
        *previous_targets_[index] + jerk_limited_step(index, target_delta, dt_seconds);
      previous_targets_[index] = limited_target;
      result.modules[index] = optimized;
      result.modules[index].target_angle_rad = limited_target;
      result.modules[index].speed_mps *= std::max(0.0, std::cos(optimized.error_rad));
      result.maximum_error_rad = std::max(result.maximum_error_rad, std::abs(optimized.error_rad));
      continue;
    }
    // Trapezoidal rate: never faster than the slew limit or the braking distance allows.
    const double braking_rate{
      target_delta == 0.0
        ? 0.0
        : std::sqrt(
            2.0 * parameters_.maximum_steering_acceleration_radps2 * std::abs(target_delta))};
    const double wanted_rate{std::copysign(std::min(maximum_rate, braking_rate), target_delta)};
    const double previous_rate{steering_rates_[index]};
    double rate{std::clamp(wanted_rate, previous_rate - rate_step, previous_rate + rate_step)};
    double step{rate * dt_seconds};
    if (target_delta != 0.0 && step / target_delta > 1.0) {
      step = target_delta;
      rate = step / dt_seconds;
    }
    steering_rates_[index] = rate;
    const double limited_target = *previous_targets_[index] + step;
    previous_targets_[index] = limited_target;

    result.modules[index] = optimized;
    result.modules[index].target_angle_rad = limited_target;
    result.modules[index].speed_mps *= std::max(0.0, std::cos(optimized.error_rad));
    result.maximum_error_rad = std::max(
      result.maximum_error_rad, std::abs(optimized.error_rad));
  }

  // All four modules gate together. Enter above the threshold; once gated, release only
  // when every module is within the release angle (hysteresis avoids stop/start chatter).
  const double release{
    std::isnan(parameters_.alignment_release_rad) ? parameters_.alignment_threshold_rad
                                                  : parameters_.alignment_release_rad};
  gated_ = result.maximum_error_rad > (gated_ ? release : parameters_.alignment_threshold_rad);
  result.gated = gated_;
  if (result.gated) {
    for (auto & module : result.modules) {
      module.speed_mps = 0.0;
    }
  }
  return result;
}

void SwerveSetpointGenerator::reset(const std::array<double, kSwerveModuleCount> & targets) noexcept
{
  reset();
  for (std::size_t i{0U}; i < targets.size(); ++i) {
    previous_targets_[i] = targets[i];
  }
  steering_rates_.fill(0.0);
}

void SwerveSetpointGenerator::reset() noexcept
{
  reversed_.fill(false);
  translation_reversed_.reset();
  previous_targets_.fill(std::nullopt);
  steering_rates_.fill(0.0);
  steering_accelerations_.fill(0.0);
  gated_ = false;
}

namespace
{
// Distance travelled until rest when braking now from rate v >= 0 with acceleration a,
// using jerk J and deceleration at most A: ramp the acceleration down to -a_p, hold it,
// then ramp back to zero so the module arrives with zero rate and zero acceleration.
double stopping_distance(double v, double a, double A, double J) noexcept
{
  if (v <= 0.0 && a <= 0.0) {
    return 0.0;
  }
  if (a < 0.0 && a * a / (2.0 * J) >= v) {
    // Already decelerating hard enough: only the remaining ramp of a up to zero.
    const double t{-a / J};
    return std::max(0.0, v * t + a * t * t / 2.0 + J * t * t * t / 6.0);
  }
  const double peak{std::min(A, std::sqrt(J * v + a * a / 2.0))};
  const double t1{(a + peak) / J};
  const double v1{v + a * t1 - J * t1 * t1 / 2.0};
  const double d1{v * t1 + a * t1 * t1 / 2.0 - J * t1 * t1 * t1 / 6.0};
  const double t2{std::max(0.0, (v1 - peak * peak / (2.0 * J)) / peak)};
  const double d2{v1 * t2 - peak * t2 * t2 / 2.0};
  const double d3{peak * peak * peak / (6.0 * J * J)};
  return d1 + d2 + d3;
}
}  // namespace

double SwerveSetpointGenerator::jerk_limited_step(
  std::size_t index, double delta, double dt_seconds) noexcept
{
  const double A{parameters_.maximum_steering_acceleration_radps2};
  const double J{parameters_.maximum_steering_jerk_radps3};
  const double maximum_rate{parameters_.maximum_steering_slew_radps};
  // Work in the goal direction: v and a are positive when moving towards the goal.
  const double direction{delta >= 0.0 ? 1.0 : -1.0};
  const double remaining{std::abs(delta)};
  double v{steering_rates_[index] * direction};
  double a{steering_accelerations_[index] * direction};
  if (remaining == 0.0 && v == 0.0 && a == 0.0) {
    return 0.0;
  }
  const double jerk_step{J * dt_seconds};
  // Candidate: accelerate towards the goal, easing into the rate limit without overshoot.
  const double rate_room{std::max(maximum_rate - v, 0.0)};
  const double cruise_acceleration{std::min(A, std::sqrt(2.0 * J * rate_room))};
  double next_a{std::clamp(cruise_acceleration, a - jerk_step, a + jerk_step)};
  double next_v{v + next_a * dt_seconds};
  double step{(v + next_v) / 2.0 * dt_seconds};
  if (stopping_distance(next_v, next_a, A, J) > remaining - step) {
    // Brake: the stop from the candidate state would not fit any more.
    next_a = std::max(a - jerk_step, -A);
    next_v = v + next_a * dt_seconds;
    step = (v + std::max(next_v, 0.0)) / 2.0 * dt_seconds;
  }
  if (next_v <= 0.0 && v >= 0.0 && remaining <= step + 1e-12) {
    // Arrived at rest.
    next_v = 0.0;
    next_a = 0.0;
    step = remaining;
  } else if (step >= remaining) {
    // Never pass the goal; the residual rate here is below one jerk step of braking.
    step = remaining;
    next_v = 0.0;
    next_a = 0.0;
  }
  next_v = std::clamp(next_v, -maximum_rate, maximum_rate);
  steering_rates_[index] = next_v * direction;
  steering_accelerations_[index] = next_a * direction;
  return step * direction;
}

double SwerveSetpointGenerator::jerk_limited_stop(std::size_t index, double dt_seconds) noexcept
{
  const double maximum_acceleration{parameters_.maximum_steering_acceleration_radps2};
  const double jerk{parameters_.maximum_steering_jerk_radps3};
  double rate{steering_rates_[index]};
  double acceleration{steering_accelerations_[index]};
  if (rate == 0.0) {
    steering_accelerations_[index] = 0.0;
    return 0.0;
  }
  const double wanted_acceleration{
    std::copysign(std::min(maximum_acceleration, std::sqrt(2.0 * jerk * std::abs(rate))), -rate)};
  acceleration = std::clamp(
    wanted_acceleration, acceleration - jerk * dt_seconds, acceleration + jerk * dt_seconds);
  const double next{rate + acceleration * dt_seconds};
  if (next * rate <= 0.0) {
    rate = 0.0;
    acceleration = 0.0;
  } else {
    rate = next;
  }
  steering_rates_[index] = rate;
  steering_accelerations_[index] = acceleration;
  return rate * dt_seconds;
}

}  // namespace swerve_driver
