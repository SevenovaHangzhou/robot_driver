#include "dm_swerve_driver/swerve_module.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
namespace {

struct SteeringTarget {
  double wheel_speed_mps{0.0};
  double angle_rad{0.0};
  double motor_position_rad{0.0};
  bool recentered{false};
};

struct DriveTarget {
  MitCommand command;
  double wheel_speed_mps{0.0};
};

[[nodiscard]] bool finite(double value) noexcept
{
  return std::isfinite(value);
}

void validate_steering_config(const SteeringModuleConfig & config)
{
  const bool gains_valid = finite(config.kp) && config.kp >= 0.0 && config.kp <= kMitKpMax &&
    finite(config.kd) && config.kd >= 0.0 && config.kd <= kMitKdMax &&
    finite(config.kff_omega) && config.kff_omega >= 0.0 &&
    finite(config.max_ff_speed_radps) && config.max_ff_speed_radps >= 0.0;
  const bool geometry_valid = finite(config.gear_ratio) && config.gear_ratio > 0.0 &&
    finite(config.zero_offset_rad);
  const bool fractions_valid = finite(config.recenter_trigger_fraction) &&
    finite(config.command_limit_fraction) && config.recenter_trigger_fraction > 0.0 &&
    config.recenter_trigger_fraction < config.command_limit_fraction &&
    config.command_limit_fraction <= 1.0;
  if (!gains_valid || !geometry_valid || !fractions_valid) {
    throw std::invalid_argument{"invalid steering module configuration"};
  }
}

void validate_drive_config(const DriveModuleConfig & config)
{
  const bool geometry_valid = finite(config.gear_ratio) && config.gear_ratio > 0.0 &&
    finite(config.wheel_radius_m) && config.wheel_radius_m > 0.0;
  const bool gains_valid = finite(config.kd) && config.kd >= 0.0 && config.kd <= kMitKdMax &&
    finite(config.ks) && config.ks >= 0.0 && finite(config.kv) && config.kv >= 0.0 &&
    finite(config.ka) && config.ka >= 0.0;
  const bool acceleration_valid = finite(config.maximum_wheel_acceleration_mps2) &&
    config.maximum_wheel_acceleration_mps2 > 0.0;
  if (!geometry_valid || !gains_valid || !acceleration_valid) {
    throw std::invalid_argument{"invalid drive module configuration"};
  }
}

[[nodiscard]] double axis_sign(bool inverted) noexcept
{
  return inverted ? -1.0 : 1.0;
}

[[nodiscard]] double signum(double value) noexcept
{
  if (value > 0.0) {
    return 1.0;
  }
  if (value < 0.0) {
    return -1.0;
  }
  return 0.0;
}

[[nodiscard]] double motor_position_for_angle(
  double angle_rad, const SteeringModuleConfig & config) noexcept
{
  return (angle_rad + config.zero_offset_rad) * config.gear_ratio *
         axis_sign(config.inverted);
}

[[nodiscard]] SteeringTarget select_steering_target(
  const OptimizedModuleState & target,
  const SteeringModuleConfig & config,
  double position_max)
{
  SteeringTarget selected{
    target.speed_mps,
    target.continuous_angle_rad,
    motor_position_for_angle(target.continuous_angle_rad, config),
    false};
  if (std::abs(selected.motor_position_rad) <= config.recenter_trigger_fraction * position_max) {
    return selected;
  }

  const double plus_angle{target.continuous_angle_rad + kPi};
  const double minus_angle{target.continuous_angle_rad - kPi};
  const double plus_position{motor_position_for_angle(plus_angle, config)};
  const double minus_position{motor_position_for_angle(minus_angle, config)};
  const bool use_plus{std::abs(plus_position) < std::abs(minus_position)};
  const double candidate_position{use_plus ? plus_position : minus_position};
  if (std::abs(candidate_position) < std::abs(selected.motor_position_rad)) {
    selected.angle_rad = use_plus ? plus_angle : minus_angle;
    selected.motor_position_rad = candidate_position;
    selected.wheel_speed_mps = -selected.wheel_speed_mps;
    selected.recentered = true;
  }
  return selected;
}

[[nodiscard]] MitCommand make_steering_command(
  SteeringTarget & target,
  const SteeringModuleConfig & config,
  const MotorLimits & limits,
  double direction,
  double dt_seconds,
  std::optional<double> & previous_target_rad)
{
  const double position_limit{config.command_limit_fraction * limits.position_max};
  target.motor_position_rad = std::clamp(
    target.motor_position_rad, -position_limit, position_limit);
  const double raw_target_velocity = previous_target_rad.has_value() ?
    (target.angle_rad - *previous_target_rad) / dt_seconds : 0.0;
  previous_target_rad = target.angle_rad;
  const double target_velocity = target.recentered ? 0.0 : std::clamp(
    raw_target_velocity, -config.max_ff_speed_radps, config.max_ff_speed_radps);
  const double motor_velocity{std::clamp(
      config.kff_omega * target_velocity * config.gear_ratio * direction,
      -limits.velocity_max, limits.velocity_max)};
  return MitCommand{target.motor_position_rad, motor_velocity, config.kp, config.kd, 0.0};
}

[[nodiscard]] DriveTarget make_drive_command(
  double requested_wheel_speed,
  bool force_zero,
  const DriveModuleConfig & config,
  const MotorLimits & limits,
  double direction,
  double dt_seconds,
  double & previous_wheel_speed)
{
  double wheel_speed{0.0};
  double acceleration{0.0};
  if (!force_zero) {
    const double maximum_step{config.maximum_wheel_acceleration_mps2 * dt_seconds};
    wheel_speed = std::clamp(
      requested_wheel_speed,
      previous_wheel_speed - maximum_step,
      previous_wheel_speed + maximum_step);
    acceleration = (wheel_speed - previous_wheel_speed) / dt_seconds;
  }
  previous_wheel_speed = wheel_speed;

  const double motor_velocity{std::clamp(
      wheel_speed / config.wheel_radius_m * config.gear_ratio * direction,
      -limits.velocity_max, limits.velocity_max)};
  const double wheel_torque = force_zero ? 0.0 :
    config.ks * signum(wheel_speed) + config.kv * wheel_speed + config.ka * acceleration;
  const double motor_torque{std::clamp(
      wheel_torque / config.gear_ratio * direction,
      -limits.torque_max, limits.torque_max)};
  return DriveTarget{
    MitCommand{0.0, motor_velocity, 0.0, config.kd, motor_torque}, wheel_speed};
}

}  // namespace

SwerveModule::SwerveModule(
  DmMotorConfig steering_motor,
  DmMotorConfig drive_motor,
  SteeringModuleConfig steering_config,
  DriveModuleConfig drive_config)
: steering_motor_{std::move(steering_motor)},
  drive_motor_{std::move(drive_motor)},
  steering_config_{std::move(steering_config)},
  drive_config_{std::move(drive_config)}
{
  validate_steering_config(steering_config_);
  validate_drive_config(drive_config_);
}

DmMotor & SwerveModule::steering_motor() noexcept
{
  return steering_motor_;
}

const DmMotor & SwerveModule::steering_motor() const noexcept
{
  return steering_motor_;
}

DmMotor & SwerveModule::drive_motor() noexcept
{
  return drive_motor_;
}

const DmMotor & SwerveModule::drive_motor() const noexcept
{
  return drive_motor_;
}

double SwerveModule::steering_angle_rad() const
{
  if (!steering_motor_.position_initialized()) {
    throw std::logic_error{"steering position has not been initialized"};
  }
  return steering_motor_.unwrapped_position() /
         (steering_config_.gear_ratio * steering_sign()) -
         steering_config_.zero_offset_rad;
}

double SwerveModule::wheel_distance_m() const
{
  if (!drive_motor_.position_initialized()) {
    throw std::logic_error{"drive position has not been initialized"};
  }
  return drive_motor_.unwrapped_position() * drive_sign() /
         drive_config_.gear_ratio * drive_config_.wheel_radius_m;
}

double SwerveModule::wheel_velocity_mps() const
{
  if (!drive_motor_.position_initialized()) {
    throw std::logic_error{"drive feedback has not been initialized"};
  }
  return drive_motor_.velocity() * drive_sign() /
         drive_config_.gear_ratio * drive_config_.wheel_radius_m;
}

SwerveModuleCommand SwerveModule::make_command(
  const OptimizedModuleState & target, double dt_seconds, bool force_drive_zero)
{
  if (!finite(target.speed_mps) || !finite(target.continuous_angle_rad) ||
    !finite(target.error_rad) || !finite(dt_seconds) || dt_seconds <= 0.0)
  {
    throw std::invalid_argument{"module target and period must be finite and valid"};
  }

  SteeringTarget steering_target{select_steering_target(
      target, steering_config_, steering_motor_.limits().position_max)};
  const MitCommand steering_command{make_steering_command(
      steering_target, steering_config_, steering_motor_.limits(), steering_sign(),
      dt_seconds, previous_steering_target_rad_)};
  const bool suppress_drive{force_drive_zero || steering_target.recentered};
  const DriveTarget drive_target{make_drive_command(
      steering_target.wheel_speed_mps, suppress_drive, drive_config_, drive_motor_.limits(),
      drive_sign(), dt_seconds, previous_wheel_speed_mps_)};

  return SwerveModuleCommand{
    steering_command,
    drive_target.command,
    drive_target.wheel_speed_mps,
    steering_target.angle_rad,
    steering_target.recentered};
}

std::array<CanFrame, 2U> SwerveModule::encode_command_frames(
  const SwerveModuleCommand & command) const
{
  return {
    steering_motor_.encode_command(command.steering),
    drive_motor_.encode_command(command.drive)};
}

void SwerveModule::reset_command_history() noexcept
{
  previous_steering_target_rad_.reset();
  previous_wheel_speed_mps_ = 0.0;
}

double SwerveModule::steering_sign() const noexcept
{
  return axis_sign(steering_config_.inverted);
}

double SwerveModule::drive_sign() const noexcept
{
  return axis_sign(drive_config_.inverted);
}

}  // namespace dm_swerve_driver
