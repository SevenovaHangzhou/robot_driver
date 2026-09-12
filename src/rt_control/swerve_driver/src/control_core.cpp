#include "swerve_driver/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_driver
{
namespace
{
bool positive(double x) noexcept {return std::isfinite(x) && x > 0.0;}
void validate(const CoreConfig & c)
{
  if (!positive(c.max_linear_speed) || !positive(c.max_angular_speed) ||
    !positive(c.max_wheel_speed) || !positive(c.max_wheel_acceleration) ||
    !positive(c.max_encoder_difference) || !positive(c.max_update_period) ||
    !positive(c.velocity_deadband) ||
    !(c.max_angular_speed * c.max_update_period < kPi))
  {
    throw std::invalid_argument("Swerve limits and timing must be explicitly configured");
  }
  std::array<SwerveModuleMeasurement, 4> sample{};
  for (size_t i = 0; i < 4; ++i) {
    if (!positive(c.wheel_radius[i]) || !std::isfinite(c.max_wheel_speed / c.wheel_radius[i]) ||
      !std::isfinite(c.locations[i].x) ||
      !std::isfinite(c.locations[i].y) || !std::isfinite(c.steering_min[i]) ||
      !std::isfinite(c.steering_max[i]) || c.steering_min[i] >= c.steering_max[i])
    {
      throw std::invalid_argument("Invalid swerve geometry or steering travel");
    }
    for (size_t j = 0; j < i; ++j) {
      if (c.locations[i].x == c.locations[j].x && c.locations[i].y == c.locations[j].y) {
        throw std::invalid_argument("Swerve modules must have distinct locations");
      }
    }
  }
  if (!chassis_speeds_from_module_states(sample, c.locations)) {
    throw std::invalid_argument("Singular swerve geometry");
  }
}
}  // namespace

ControlCore::ControlCore(const CoreConfig & config)
: config_(config), setpoints_(config.setpoint)
{
  validate(config_);
  last_motor_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  output_.steering_position = last_motor_positions_;
}

void ControlCore::remember_positions(const ModuleFeedbackArray & feedback) noexcept
{
  for (size_t i = 0; i < 4; ++i) {
    const auto position = feedback[i].steering_position;
    if (feedback[i].valid && std::isfinite(position) &&
      position >= config_.steering_min[i] && position <= config_.steering_max[i])
    {
      last_motor_positions_[i] = position;
    }
  }
}

bool ControlCore::measurement_valid(const ModuleFeedback & f) const noexcept
{
  return f.valid && std::isfinite(f.steering_position) && std::isfinite(f.steering_angle) &&
         std::isfinite(f.wheel_position) && std::isfinite(f.wheel_velocity);
}

bool ControlCore::feedback_ready(const ModuleFeedbackArray & feedback) const noexcept
{
  for (size_t i = 0; i < 4; ++i) {
    const auto & f = feedback[i];
    if (!measurement_valid(f) || !f.enabled ||
      !std::isfinite(f.wheel_position * config_.wheel_radius[i]) ||
      !std::isfinite(f.wheel_velocity * config_.wheel_radius[i]) ||
      std::abs(f.steering_position - f.steering_angle) > config_.max_encoder_difference ||
      f.steering_position < config_.steering_min[i] || f.steering_position > config_.steering_max[i])
    {
      return false;
    }
  }
  return true;
}

std::array<SwerveModulePosition, 4> ControlCore::positions(const ModuleFeedbackArray & feedback) const
{
  std::array<SwerveModulePosition, 4> result{};
  for (size_t i = 0; i < 4; ++i) {
    result[i] = {feedback[i].wheel_position * config_.wheel_radius[i],
      feedback[i].steering_angle, measurement_valid(feedback[i])};
    result[i].valid = result[i].valid && std::isfinite(result[i].distance_m);
  }
  return result;
}

bool ControlCore::activate(const ModuleFeedbackArray & feedback)
{
  active_ = false;
  remember_positions(feedback);
  if (!feedback_ready(feedback)) {return false;}
  previous_positions_ = positions(feedback);
  odometry_.emplace(config_.locations, yaw_, previous_positions_, output_.pose);
  setpoints_.reset();
  previous_speeds_.fill(0.0);
  imu_fallback_ = true;
  output_ = stop(feedback, ControlStatus::command_timeout);
  active_ = true;
  return true;
}

void ControlCore::deactivate() noexcept
{
  active_ = false;
  previous_speeds_.fill(0.0);
  setpoints_.reset();
}

void ControlCore::observe(const ModuleFeedbackArray & feedback, std::optional<YawSample> imu)
{
  if (!odometry_) {return;}
  const auto current = positions(feedback);
  const auto delta = wheel_chassis_delta_from_position_deltas(previous_positions_, current, config_.locations);
  for (size_t i = 0; i < 4; ++i) {
    if (current[i].valid) {previous_positions_[i] = current[i];}
  }
  const bool fresh_imu = imu && std::isfinite(imu->yaw_rad) && std::isfinite(imu->rate_radps);
  if (fresh_imu) {
    if (!imu_fallback_) {yaw_ += wrap_pi(imu->yaw_rad - last_imu_yaw_);}
    last_imu_yaw_ = imu->yaw_rad;
  } else if (delta) {
    yaw_ += delta->dtheta_rad;
  }
  imu_fallback_ = !fresh_imu;
  output_.imu_fallback = imu_fallback_;
  output_.pose = odometry_->update(yaw_, current);
  std::array<SwerveModuleMeasurement, 4> measured{};
  output_.valid_modules = 0;
  for (size_t i = 0; i < 4; ++i) {
    measured[i] = {feedback[i].wheel_velocity * config_.wheel_radius[i],
      feedback[i].steering_angle, current[i].valid};
    measured[i].valid = measured[i].valid && std::isfinite(measured[i].speed_mps);
    if (measured[i].valid) {++output_.valid_modules;}
  }
  output_.measured_twist = chassis_speeds_from_module_states(measured, config_.locations).value_or(ChassisSpeeds{});
  if (fresh_imu) {output_.measured_twist.omega_radps = imu->rate_radps;}
  if (!std::isfinite(output_.pose.x_m) || !std::isfinite(output_.pose.y_m) ||
    !std::isfinite(output_.pose.heading_rad) || !std::isfinite(output_.measured_twist.vx_mps) ||
    !std::isfinite(output_.measured_twist.vy_mps) || !std::isfinite(output_.measured_twist.omega_radps))
  {
    throw std::runtime_error("Non-finite swerve odometry");
  }
}

ControlOutput ControlCore::stop(const ModuleFeedbackArray & feedback, ControlStatus status)
{
  remember_positions(feedback);
  output_.steering_position = last_motor_positions_;
  output_.drive_velocity.fill(0.0);
  previous_speeds_.fill(0.0);
  setpoints_.reset();
  output_.status = status;
  return output_;
}

ControlOutput ControlCore::update(const ModuleFeedbackArray & feedback, ChassisSpeeds command,
  bool command_fresh, double dt, std::optional<YawSample> imu)
{
  if (!active_) {return stop(feedback, ControlStatus::inactive);}
  remember_positions(feedback);
  observe(feedback, imu);
  if (!feedback_ready(feedback)) {return stop(feedback, ControlStatus::feedback_fault);}
  if (!command_fresh) {return stop(feedback, ControlStatus::command_timeout);}
  if (!positive(dt) || dt > config_.max_update_period || !std::isfinite(command.vx_mps) ||
    !std::isfinite(command.vy_mps) || !std::isfinite(command.omega_radps))
  {
    return stop(feedback, ControlStatus::invalid_command);
  }
  const auto linear = std::hypot(command.vx_mps, command.vy_mps);
  if (!std::isfinite(linear)) {return stop(feedback, ControlStatus::invalid_command);}
  if (linear > config_.max_linear_speed) {
    command.vx_mps *= config_.max_linear_speed / linear;
    command.vy_mps *= config_.max_linear_speed / linear;
  }
  command.omega_radps = std::clamp(command.omega_radps, -config_.max_angular_speed, config_.max_angular_speed);
  std::array<double, 4> angles{};
  for (size_t i = 0; i < 4; ++i) {angles[i] = feedback[i].steering_angle;}
  auto desired = inverse_kinematics(discretize(command, dt), config_.locations, angles, config_.velocity_deadband);
  if (std::all_of(desired.begin(), desired.end(), [this](const auto & state) {
      return std::abs(state.speed_mps) < config_.velocity_deadband;
    }))
  {
    return stop(feedback, ControlStatus::running);
  }
  desaturate_wheel_speeds(desired, config_.max_wheel_speed);
  const auto targets = setpoints_.generate(desired, angles, dt);
  const auto max_step = config_.max_wheel_acceleration * dt;
  for (size_t i = 0; i < 4; ++i) {
    const auto target = std::abs(desired[i].speed_mps) < config_.velocity_deadband ?
      feedback[i].steering_position : targets.modules[i].continuous_angle_rad;
    if (!std::isfinite(target) || target < config_.steering_min[i] || target > config_.steering_max[i]) {
      return stop(feedback, ControlStatus::steering_limit);
    }
    output_.steering_position[i] = target;
    previous_speeds_[i] = targets.gated ? 0.0 : std::clamp(targets.modules[i].speed_mps,
      previous_speeds_[i] - max_step, previous_speeds_[i] + max_step);
    output_.drive_velocity[i] = previous_speeds_[i] / config_.wheel_radius[i];
  }
  output_.status = targets.gated ? ControlStatus::alignment_gated : ControlStatus::running;
  return output_;
}
}  // namespace swerve_driver
