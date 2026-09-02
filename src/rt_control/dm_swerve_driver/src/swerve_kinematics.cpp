#include "dm_swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_swerve_driver {
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

}  // namespace

double wrap_pi(double angle_rad) noexcept
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
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
  const SwerveModuleState & desired, double current_unwrapped_angle_rad)
{
  validate_state(desired);
  if (!finite(current_unwrapped_angle_rad)) {
    throw std::invalid_argument{"current module angle must be finite"};
  }

  double error{wrap_pi(desired.angle_rad - wrap_pi(current_unwrapped_angle_rad))};
  double speed{desired.speed_mps};
  if (std::abs(error) > kPi / 2.0) {
    error -= std::copysign(kPi, error);
    speed = -speed;
  }
  return OptimizedModuleState{speed, current_unwrapped_angle_rad + error, error};
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

}  // namespace dm_swerve_driver
