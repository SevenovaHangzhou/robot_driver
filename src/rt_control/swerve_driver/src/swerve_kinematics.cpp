#include "swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_driver {
namespace {

constexpr double kSmallAngleThreshold{1e-9};

[[nodiscard]] bool finite(double value) noexcept
{
  return std::isfinite(value);
}

void validate_speeds(const ChassisSpeeds & speeds)
{
  if (!finite(speeds.vx_mps) || !finite(speeds.vy_mps) || !finite(speeds.omega_radps)) {
    throw std::invalid_argument{"chassis speeds must be finite"};
  }
}

void validate_state(const SwerveModuleState & state)
{
  if (!finite(state.speed_mps) || !finite(state.angle_rad)) {
    throw std::invalid_argument{"swerve module state must be finite"};
  }
}

[[nodiscard]] double discretization_coefficient(double theta) noexcept
{
  if (std::abs(theta) < kSmallAngleThreshold) {
    return 1.0 - theta * theta / 12.0;
  }
  const double half_theta{theta / 2.0};
  return half_theta / std::tan(half_theta);
}

[[nodiscard]] double midpoint_angle(double previous, double current) noexcept
{
  return previous + wrap_pi(current - previous) / 2.0;
}

using AugmentedMatrix = std::array<std::array<double, 4U>, 3U>;

[[nodiscard]] std::optional<std::array<double, 3U>> solve_three_by_three(
  AugmentedMatrix matrix) noexcept
{
  constexpr double singular_tolerance{1e-12};
  for (std::size_t column{0U}; column < 3U; ++column) {
    std::size_t pivot{column};
    for (std::size_t row{column + 1U}; row < 3U; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < singular_tolerance) {
      return std::nullopt;
    }
    std::swap(matrix[column], matrix[pivot]);
    const double divisor{matrix[column][column]};
    for (std::size_t entry{column}; entry < 4U; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (std::size_t row{0U}; row < 3U; ++row) {
      if (row == column) {
        continue;
      }
      const double scale{matrix[row][column]};
      for (std::size_t entry{column}; entry < 4U; ++entry) {
        matrix[row][entry] -= scale * matrix[column][entry];
      }
    }
  }
  return std::array<double, 3U>{matrix[0][3], matrix[1][3], matrix[2][3]};
}

void accumulate_observation(
  AugmentedMatrix & normal,
  const std::array<double, 3U> & row,
  double measurement) noexcept
{
  for (std::size_t outer{0U}; outer < 3U; ++outer) {
    for (std::size_t inner{0U}; inner < 3U; ++inner) {
      normal[outer][inner] += row[outer] * row[inner];
    }
    normal[outer][3] += row[outer] * measurement;
  }
}

struct ModuleVectorObservation {
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

struct ModuleLeastSquaresFit {
  std::array<double, 3U> solution{};
  std::array<double, kSwerveModuleCount> residuals{};
  std::array<bool, kSwerveModuleCount> used{};
  std::array<bool, kSwerveModuleCount> rejected{};
  std::size_t used_count{0U};
};

[[nodiscard]] std::size_t count_used(
  const std::array<bool, kSwerveModuleCount> & used) noexcept
{
  return static_cast<std::size_t>(std::count(used.begin(), used.end(), true));
}

[[nodiscard]] std::optional<std::array<double, 3U>> solve_with_mask(
  const std::array<ModuleVectorObservation, kSwerveModuleCount> & observations,
  const std::array<Translation2d, kSwerveModuleCount> & locations,
  const std::array<bool, kSwerveModuleCount> & used) noexcept
{
  AugmentedMatrix normal{};
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    if (!used[index]) {
      continue;
    }
    accumulate_observation(
      normal, {1.0, 0.0, -locations[index].y}, observations[index].x);
    accumulate_observation(
      normal, {0.0, 1.0, locations[index].x}, observations[index].y);
  }
  return solve_three_by_three(normal);
}

[[nodiscard]] double module_residual(
  const ModuleVectorObservation & observation,
  const Translation2d & location,
  const std::array<double, 3U> & solution) noexcept
{
  const double predicted_x{solution[0] - solution[2] * location.y};
  const double predicted_y{solution[1] + solution[2] * location.x};
  return std::hypot(observation.x - predicted_x, observation.y - predicted_y);
}

[[nodiscard]] std::optional<ModuleLeastSquaresFit> fit_module_observations(
  const std::array<ModuleVectorObservation, kSwerveModuleCount> & observations,
  const std::array<Translation2d, kSwerveModuleCount> & locations,
  std::optional<double> rejection_threshold) noexcept
{
  ModuleLeastSquaresFit result;
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    result.used[index] = observations[index].valid;
  }

  while (count_used(result.used) >= 2U) {
    const auto solution = solve_with_mask(observations, locations, result.used);
    if (!solution.has_value()) {
      return std::nullopt;
    }
    result.solution = *solution;
    if (!rejection_threshold.has_value()) {
      break;
    }

    double largest_residual{-std::numeric_limits<double>::infinity()};
    std::size_t largest_index{0U};
    for (std::size_t index{0U}; index < observations.size(); ++index) {
      if (!result.used[index]) {
        continue;
      }
      const double residual{
        module_residual(observations[index], locations[index], *solution)};
      if (residual > largest_residual) {
        largest_residual = residual;
        largest_index = index;
      }
    }
    if (largest_residual <= *rejection_threshold) {
      break;
    }
    if (count_used(result.used) == 2U) {
      return std::nullopt;
    }
    result.used[largest_index] = false;
    result.rejected[largest_index] = true;
  }

  result.used_count = count_used(result.used);
  if (result.used_count < 2U) {
    return std::nullopt;
  }
  for (std::size_t index{0U}; index < observations.size(); ++index) {
    if (observations[index].valid) {
      result.residuals[index] = module_residual(
        observations[index], locations[index], result.solution);
    }
  }
  return result;
}

[[nodiscard]] std::array<ModuleVectorObservation, kSwerveModuleCount>
velocity_observations(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & locations)
{
  std::array<ModuleVectorObservation, kSwerveModuleCount> observations{};
  for (std::size_t index{0U}; index < modules.size(); ++index) {
    if (!modules[index].valid) {
      continue;
    }
    const auto & module = modules[index];
    const auto & location = locations[index];
    if (!finite(module.speed_mps) || !finite(module.angle_rad) ||
      !finite(location.x) || !finite(location.y))
    {
      throw std::invalid_argument{"valid module velocity samples and geometry must be finite"};
    }
    observations[index] = ModuleVectorObservation{
      module.speed_mps * std::cos(module.angle_rad),
      module.speed_mps * std::sin(module.angle_rad), true};
  }
  return observations;
}

}  // namespace

bool ChassisSpeedFit::slip_detected() const noexcept
{
  return std::any_of(
    slipping_modules.begin(), slipping_modules.end(), [](bool slipping) {return slipping;});
}

double wrap_pi(double angle_rad) noexcept
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

bool valid_steering_angle_limits(const SteeringAngleLimits & limits) noexcept
{
  constexpr double range_tolerance{1e-12};
  const double physical_range{limits.maximum_rad - limits.minimum_rad};
  const double safe_range{physical_range - 2.0 * limits.margin_rad};
  return finite(limits.minimum_rad) && finite(limits.maximum_rad) &&
         finite(limits.margin_rad) && limits.margin_rad >= 0.0 &&
         finite(limits.measurement_tolerance_rad) &&
         limits.measurement_tolerance_rad >= 0.0 &&
         physical_range <= 2.0 * kPi + range_tolerance && safe_range >= kPi;
}

bool steering_angle_within_limits(
  double angle_rad, const SteeringAngleLimits & limits) noexcept
{
  return finite(angle_rad) && valid_steering_angle_limits(limits) &&
         angle_rad >= limits.minimum_rad + limits.margin_rad &&
         angle_rad <= limits.maximum_rad - limits.margin_rad;
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

std::optional<double> nearest_equivalent_steering_target(
  double angle_rad,
  double current_angle_rad,
  const SteeringAngleLimits & limits) noexcept
{
  if (!finite(angle_rad) || !steering_angle_within_limits(current_angle_rad, limits)) {
    return std::nullopt;
  }
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

ChassisSpeeds discretize(const ChassisSpeeds & speeds, double dt_seconds)
{
  validate_speeds(speeds);
  if (!finite(dt_seconds) || dt_seconds <= 0.0) {
    throw std::invalid_argument{"discretization period must be finite and positive"};
  }

  const double theta{speeds.omega_radps * dt_seconds};
  const double half_theta{theta / 2.0};
  const double coefficient{discretization_coefficient(theta)};
  return ChassisSpeeds{
    coefficient * speeds.vx_mps + half_theta * speeds.vy_mps,
    -half_theta * speeds.vx_mps + coefficient * speeds.vy_mps,
    speeds.omega_radps};
}

std::array<SwerveModuleState, kSwerveModuleCount> inverse_kinematics(
  const ChassisSpeeds & speeds,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  const std::array<double, kSwerveModuleCount> & previous_angles_rad,
  double velocity_deadband_mps)
{
  validate_speeds(speeds);
  if (!finite(velocity_deadband_mps) || velocity_deadband_mps < 0.0) {
    throw std::invalid_argument{"velocity deadband must be finite and nonnegative"};
  }

  std::array<SwerveModuleState, kSwerveModuleCount> states{};
  for (std::size_t index{0U}; index < states.size(); ++index) {
    const auto & location = module_locations[index];
    if (!finite(location.x) || !finite(location.y) || !finite(previous_angles_rad[index])) {
      throw std::invalid_argument{"module geometry and previous angles must be finite"};
    }
    const double wheel_x{speeds.vx_mps - speeds.omega_radps * location.y};
    const double wheel_y{speeds.vy_mps + speeds.omega_radps * location.x};
    const double speed{std::hypot(wheel_x, wheel_y)};
    states[index] = SwerveModuleState{
      speed,
      speed < velocity_deadband_mps ? previous_angles_rad[index] : std::atan2(wheel_y, wheel_x)};
  }
  return states;
}

void desaturate_wheel_speeds(
  std::array<SwerveModuleState, kSwerveModuleCount> & states,
  double maximum_speed_mps)
{
  if (!finite(maximum_speed_mps) || maximum_speed_mps <= 0.0) {
    throw std::invalid_argument{"maximum wheel speed must be finite and positive"};
  }
  double largest_speed{0.0};
  for (const auto & state : states) {
    validate_state(state);
    largest_speed = std::max(largest_speed, std::abs(state.speed_mps));
  }
  if (largest_speed <= maximum_speed_mps) {
    return;
  }
  const double scale{maximum_speed_mps / largest_speed};
  for (auto & state : states) {
    state.speed_mps *= scale;
  }
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
  const auto forward = nearest_equivalent_steering_target(
    forward_base, planning_angle, limits);
  const auto reverse = nearest_equivalent_steering_target(
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
  const std::array<double, kSwerveModuleCount> & current_unwrapped_angles_rad,
  double alignment_threshold_rad)
{
  if (!finite(alignment_threshold_rad) || alignment_threshold_rad < 0.0) {
    throw std::invalid_argument{"alignment threshold must be finite and nonnegative"};
  }

  AlignmentResult result{};
  for (std::size_t index{0U}; index < result.modules.size(); ++index) {
    result.modules[index] = optimize_module(desired[index], current_unwrapped_angles_rad[index]);
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

std::optional<Translation2d> wheel_translation_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  double delta_yaw_rad)
{
  if (!finite(delta_yaw_rad)) {
    throw std::invalid_argument{"gyro yaw delta must be finite"};
  }

  Translation2d sum{};
  std::size_t valid_count{0U};
  for (std::size_t index{0U}; index < current.size(); ++index) {
    if (!previous[index].valid || !current[index].valid) {
      continue;
    }
    const auto & old_position = previous[index];
    const auto & new_position = current[index];
    const auto & location = module_locations[index];
    if (!finite(old_position.distance_m) || !finite(old_position.angle_rad) ||
      !finite(new_position.distance_m) || !finite(new_position.angle_rad) ||
      !finite(location.x) || !finite(location.y))
    {
      throw std::invalid_argument{"valid module samples and geometry must be finite"};
    }

    const double distance_delta{new_position.distance_m - old_position.distance_m};
    const double steering_angle{midpoint_angle(old_position.angle_rad, new_position.angle_rad)};
    sum.x += distance_delta * std::cos(steering_angle) + delta_yaw_rad * location.y;
    sum.y += distance_delta * std::sin(steering_angle) - delta_yaw_rad * location.x;
    ++valid_count;
  }

  if (valid_count == 0U) {
    return std::nullopt;
  }
  const double divisor{static_cast<double>(valid_count)};
  return Translation2d{sum.x / divisor, sum.y / divisor};
}

std::optional<ChassisDelta> wheel_chassis_delta_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations)
{
  AugmentedMatrix normal{};
  std::size_t valid_count{0U};
  for (std::size_t index{0U}; index < current.size(); ++index) {
    if (!previous[index].valid || !current[index].valid) {
      continue;
    }
    const auto & old_position = previous[index];
    const auto & new_position = current[index];
    const auto & location = module_locations[index];
    if (!finite(old_position.distance_m) || !finite(old_position.angle_rad) ||
      !finite(new_position.distance_m) || !finite(new_position.angle_rad) ||
      !finite(location.x) || !finite(location.y))
    {
      throw std::invalid_argument{"valid module samples and geometry must be finite"};
    }
    const double distance{new_position.distance_m - old_position.distance_m};
    const double angle{midpoint_angle(old_position.angle_rad, new_position.angle_rad)};
    accumulate_observation(normal, {1.0, 0.0, -location.y}, distance * std::cos(angle));
    accumulate_observation(normal, {0.0, 1.0, location.x}, distance * std::sin(angle));
    ++valid_count;
  }
  if (valid_count < 2U) {
    return std::nullopt;
  }
  const auto solution = solve_three_by_three(normal);
  if (!solution.has_value()) {
    return std::nullopt;
  }
  return ChassisDelta{(*solution)[0], (*solution)[1], (*solution)[2]};
}

std::optional<ChassisSpeeds> chassis_speeds_from_module_states(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations)
{
  const auto fit = fit_module_observations(
    velocity_observations(modules, module_locations), module_locations, std::nullopt);
  if (!fit.has_value()) {
    return std::nullopt;
  }
  return ChassisSpeeds{fit->solution[0], fit->solution[1], fit->solution[2]};
}

std::optional<ChassisSpeedFit> chassis_speeds_with_slip_rejection(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  double slip_residual_threshold)
{
  if (!finite(slip_residual_threshold) || slip_residual_threshold <= 0.0) {
    throw std::invalid_argument{"slip residual threshold must be finite and positive"};
  }
  const auto fit = fit_module_observations(
    velocity_observations(modules, module_locations), module_locations,
    slip_residual_threshold);
  if (!fit.has_value()) {
    return std::nullopt;
  }
  ChassisSpeedFit result;
  result.speeds = ChassisSpeeds{fit->solution[0], fit->solution[1], fit->solution[2]};
  result.residual_mps = fit->residuals;
  result.used_modules = fit->used;
  result.slipping_modules = fit->rejected;
  result.used_module_count = fit->used_count;
  return result;
}

}  // namespace swerve_driver
