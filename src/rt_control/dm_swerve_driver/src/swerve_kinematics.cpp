#include "dm_swerve_driver/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swerve_least_squares.hpp"

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
  std::array<ModuleVectorObservation, kSwerveModuleCount> observations{};
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
    observations[index] = ModuleVectorObservation{
      distance * std::cos(angle), distance * std::sin(angle), true};
    ++valid_count;
  }
  if (valid_count < 2U) {
    return std::nullopt;
  }
  const auto fit = fit_module_observations(observations, module_locations, std::nullopt);
  if (!fit.has_value()) {
    return std::nullopt;
  }
  return ChassisDelta{fit->solution[0], fit->solution[1], fit->solution[2]};
}

namespace {

[[nodiscard]] std::array<ModuleVectorObservation, kSwerveModuleCount> velocity_observations(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations)
{
  std::array<ModuleVectorObservation, kSwerveModuleCount> observations{};
  for (std::size_t index{0U}; index < modules.size(); ++index) {
    if (!modules[index].valid) {
      continue;
    }
    const auto & module = modules[index];
    const auto & location = module_locations[index];
    if (!finite(module.speed_mps) || !finite(module.angle_rad) ||
      !finite(location.x) || !finite(location.y))
    {
      throw std::invalid_argument{"valid module velocity samples and geometry must be finite"};
    }
    observations[index] = ModuleVectorObservation{
      module.speed_mps * std::cos(module.angle_rad),
      module.speed_mps * std::sin(module.angle_rad),
      true};
  }
  return observations;
}

}  // namespace

bool ChassisSpeedFit::slip_detected() const noexcept
{
  return std::any_of(
    slipping_modules.begin(), slipping_modules.end(), [](bool slipping) {return slipping;});
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
  return ChassisSpeedFit{
    ChassisSpeeds{fit->solution[0], fit->solution[1], fit->solution[2]},
    fit->residuals,
    fit->used,
    fit->rejected,
    fit->used_count};
}

}  // namespace dm_swerve_driver
