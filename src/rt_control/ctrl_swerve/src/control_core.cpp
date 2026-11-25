#include "ctrl_swerve/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ctrl_swerve
{
namespace
{
bool positive(double x) noexcept
{
  return std::isfinite(x) && x > 0.0;
}
SteeringAngleLimits steering_limits(const CoreConfig & config, size_t index) noexcept
{
  return {
    config.steering_min[index], config.steering_max[index], config.steering_limit_margin[index],
    config.steering_limit_tolerance[index]};
}
SwerveSetpointParameters setpoint_parameters(const CoreConfig & config)
{
  auto parameters = config.setpoint;
  for (size_t i = 0; i < parameters.angle_limits.size(); ++i) {
    parameters.angle_limits[i] = steering_limits(config, i);
  }
  return parameters;
}
BodyShaperLimits body_limits(const CoreConfig & config)
{
  BodyShaperLimits limits;
  limits.linear_acceleration = config.max_linear_acceleration;
  limits.angular_acceleration = config.max_angular_acceleration;
  limits.jerk = config.max_body_jerk;
  double radius{0.0};
  for (const auto & location : config.locations) {
    radius = std::max(radius, std::hypot(location.x, location.y));
  }
  // Invalid geometry is reported by validate(); keep the shaper constructible.
  limits.rotation_radius_m = positive(radius) ? radius : 1.0;
  return limits;
}
void validate(const CoreConfig & c)
{
  if (
    !positive(c.max_linear_speed) || !positive(c.max_angular_speed) ||
    !positive(c.max_wheel_speed) || !positive(c.max_wheel_acceleration) ||
    !positive(c.max_encoder_difference) || !positive(c.max_update_period) ||
    !positive(c.velocity_deadband) || !positive(c.slip_residual_threshold) ||
    !positive(c.encoder_fault_window) || !(c.max_angular_speed * c.max_update_period < kPi)) {
    throw std::invalid_argument("Swerve limits and timing must be explicitly configured");
  }
  if (
    !positive(c.max_rotation_speed) || !positive(c.max_rotation_acceleration) ||
    !positive(c.rotation_stop_speed) || !positive(c.rotation_stop_hold) ||
    !(c.max_rotation_speed * c.max_update_period < kPi)) {
    throw std::invalid_argument("Swerve limits and timing must be explicitly configured");
  }
  std::array<SwerveModuleMeasurement, 4> sample{};
  for (size_t i = 0; i < 4; ++i) {
    if (
      !positive(c.wheel_radius[i]) || !std::isfinite(c.max_wheel_speed / c.wheel_radius[i]) ||
      !std::isfinite(c.locations[i].x) || !std::isfinite(c.locations[i].y) ||
      !std::isfinite(c.steering_min[i]) || !valid_steering_angle_limits(steering_limits(c, i))) {
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
: config_(config), setpoints_(setpoint_parameters(config)), body_shaper_(body_limits(config))
{
  validate(config_);
  last_steering_targets_.fill(std::numeric_limits<double>::quiet_NaN());
  output_.steering_position = last_steering_targets_;
  refresh_snapshot({});
}

void ControlCore::update_rotation_capability(
  const ModuleFeedbackArray & feedback, const ChassisSpeeds & command, bool command_fresh,
  double dt, bool requested) noexcept
{
  rotation_requested_ = requested;
  bool eligible = active_ && requested && command_fresh && !hold_fault_latched_ && positive(dt) &&
                  dt <= config_.max_update_period && std::isfinite(command.omega_radps) &&
                  command.vx_mps == 0.0 && command.vy_mps == 0.0;
  bool stopped = command.omega_radps == 0.0;
  for (size_t i = 0; i < 4; ++i) {
    eligible = eligible && module_ready(feedback[i], i) && encoder_consistent(feedback[i], i);
    stopped = stopped &&
              std::abs(feedback[i].wheel_velocity * config_.wheel_radius[i]) <=
                config_.rotation_stop_speed &&
              output_.drive_velocity[i] == 0.0;
  }
  if (!eligible) {
    rotation_active_ = false;
    rotation_stopped_time_ = 0.0;
  } else if (!rotation_active_) {
    rotation_stopped_time_ = stopped ? rotation_stopped_time_ + dt : 0.0;
    rotation_active_ = rotation_stopped_time_ >= config_.rotation_stop_hold;
  }
}

void ControlCore::refresh_snapshot(const ModuleFeedbackArray & feedback) noexcept
{
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  snapshot_.status = output_.status;
  snapshot_.feedback_valid = true;
  snapshot_.rotation_active = rotation_active_;
  snapshot_.rotation_requested = rotation_requested_;
  snapshot_.gated = setpoints_.gated();
  snapshot_.waiting = body_alignment_waiting_;
  snapshot_.translation_branch = setpoints_.translation_branch();
  snapshot_.gate = config_.setpoint.alignment_threshold_rad;
  snapshot_.release = std::isnan(config_.setpoint.alignment_release_rad)
                        ? snapshot_.gate
                        : config_.setpoint.alignment_release_rad;
  snapshot_.flip_hysteresis = config_.setpoint.flip_hysteresis_rad;
  snapshot_.ready =
    active_ && !hold_fault_latched_ &&
    (output_.status == ControlStatus::running || output_.status == ControlStatus::alignment_gated ||
     output_.status == ControlStatus::command_timeout);
  snapshot_.body_reference = body_shaper_.velocity();
  snapshot_.body_acceleration = body_shaper_.body_acceleration();
  snapshot_.body_measured = output_.measured_twist;
  if (output_.valid_modules == 0U) {
    snapshot_.body_measured = {
      unavailable, unavailable,
      output_.imu_fallback ? unavailable : output_.measured_twist.omega_radps};
  }
  snapshot_.max_linear_speed = config_.max_linear_speed;
  snapshot_.max_angular_speed =
    rotation_active_ ? config_.max_rotation_speed : config_.max_angular_speed;
  snapshot_.max_linear_acceleration = config_.max_linear_acceleration;
  snapshot_.max_angular_acceleration =
    rotation_active_ ? config_.max_rotation_acceleration : config_.max_angular_acceleration;
  snapshot_.max_body_jerk = config_.max_body_jerk;
  const auto & goals = setpoints_.goals();
  for (size_t i = 0; i < 4; ++i) {
    auto & module = snapshot_.modules[i];
    const auto & sample = feedback[i];
    module.ready = module_ready(sample, i) && encoder_consistent(sample, i);
    snapshot_.feedback_valid = snapshot_.feedback_valid && module.ready;
    snapshot_.ready = snapshot_.ready && module.ready;
    module.reversed = goals[i].reversed;
    module.holding = holding_modules_[i];
    module.steering_actual = measurement_valid(sample) ? sample.steering_position : unavailable;
    module.steering_encoder = sample.encoder_valid && std::isfinite(sample.steering_angle)
                                ? sample.steering_angle
                                : unavailable;
    module.steering_target = goals[i].target_angle_rad;
    module.steering_reference = output_.steering_position[i];
    module.steering_rate = setpoints_.steering_rates()[i];
    module.steering_acceleration = setpoints_.steering_accelerations()[i];
    // Match the selected branch's motor-derived, safe-range planning angle, not the
    // independent external encoder monitor (nor the rate-limited reference angle).
    module.steering_error = goals[i].error_rad;
    module.wheel_actual =
      measurement_valid(sample) ? sample.wheel_velocity * config_.wheel_radius[i] : unavailable;
    module.wheel_position = measurement_valid(sample) ? sample.wheel_position : unavailable;
    module.wheel_target = goals[i].speed_mps;
    module.wheel_reference = output_.drive_velocity[i] * config_.wheel_radius[i];
    module.wheel_max_speed = config_.max_wheel_speed;
    module.wheel_max_acceleration = config_.max_wheel_acceleration;
    module.steering_min = config_.steering_min[i] + config_.steering_limit_margin[i];
    module.steering_max = config_.steering_max[i] - config_.steering_limit_margin[i];
    module.steering_max_rate = config_.setpoint.maximum_steering_slew_radps;
    module.steering_max_acceleration = config_.setpoint.maximum_steering_acceleration_radps2;
    module.steering_max_jerk = config_.setpoint.maximum_steering_jerk_radps3;
    module.wheel_radius = config_.wheel_radius[i];
    module.location = config_.locations[i];
  }
}

bool ControlCore::hold_deviation(
  const ModuleFeedbackArray & feedback, const std::array<bool, 4> & holding) const noexcept
{
  for (size_t i = 0; i < 4; ++i) {
    const auto & sample = feedback[i];
    if (
      holding[i] && sample.enabled && measurement_valid(sample) &&
      std::isfinite(last_steering_targets_[i]) &&
      // The hold is judged on the motor-derived angle: synchronous, no gear backlash.
      std::abs(sample.steering_position - last_steering_targets_[i]) >
        config_.max_encoder_difference) {
      return true;
    }
  }
  return false;
}

bool ControlCore::measurement_valid(const ModuleFeedback & f) const noexcept
{
  return f.valid && std::isfinite(f.steering_position) && std::isfinite(f.wheel_position) &&
         std::isfinite(f.wheel_velocity);
}

bool ControlCore::encoder_consistent(const ModuleFeedback & f, size_t index) const noexcept
{
  return f.encoder_valid && std::isfinite(f.steering_angle) &&
         std::abs(f.steering_position - f.steering_angle) <= config_.max_encoder_difference &&
         steering_measurement_within_tolerance(f.steering_angle, steering_limits(config_, index));
}

bool ControlCore::encoder_monitor_tripped(const ModuleFeedbackArray & feedback, double dt) noexcept
{
  bool tripped = false;
  for (size_t i = 0; i < 4; ++i) {
    if (encoder_consistent(feedback[i], i)) {
      encoder_bad_time_[i] = 0.0;
    } else {
      encoder_bad_time_[i] += positive(dt) ? dt : 0.0;
      tripped = tripped || encoder_bad_time_[i] > config_.encoder_fault_window;
    }
  }
  return tripped;
}

bool ControlCore::module_ready(const ModuleFeedback & f, size_t index) const noexcept
{
  const auto limits = steering_limits(config_, index);
  return measurement_valid(f) && f.enabled &&
         std::isfinite(f.wheel_position * config_.wheel_radius[index]) &&
         std::isfinite(f.wheel_velocity * config_.wheel_radius[index]) &&
         steering_angle_within_limits(f.steering_position, limits);
}

bool ControlCore::feedback_ready(const ModuleFeedbackArray & feedback) const noexcept
{
  for (size_t i = 0; i < 4; ++i) {
    if (!module_ready(feedback[i], i)) {
      return false;
    }
  }
  return true;
}

std::array<SwerveModulePosition, 4> ControlCore::positions(
  const ModuleFeedbackArray & feedback) const
{
  std::array<SwerveModulePosition, 4> result{};
  for (size_t i = 0; i < 4; ++i) {
    const auto limits = steering_limits(config_, i);
    // Odometry heading uses the motor-derived angle (1 kHz, synchronous with the wheels).
    const bool angle_valid =
      steering_measurement_within_tolerance(feedback[i].steering_position, limits);
    // A holding module whose angle left its latched target (a rejected steering jump) is
    // excluded, so the cycle that latches the hold fault cannot pollute the pose.
    const bool hold_consistent =
      !(holding_modules_[i] && active_ && std::isfinite(last_steering_targets_[i]) &&
        std::abs(feedback[i].steering_position - last_steering_targets_[i]) >
          config_.max_encoder_difference);
    result[i] = {
      feedback[i].wheel_position * config_.wheel_radius[i],
      angle_valid ? clamp_steering_measurement_to_safe_range(feedback[i].steering_position, limits)
                  : feedback[i].steering_position,
      module_ready(feedback[i], i) && angle_valid && hold_consistent};
    result[i].valid = result[i].valid && std::isfinite(result[i].distance_m);
  }
  return result;
}

bool ControlCore::activate(const ModuleFeedbackArray & feedback)
{
  deactivate();
  refresh_snapshot(feedback);
  // A successful new activation is the only place feedback initializes the hold.
  if (!feedback_ready(feedback)) {
    return false;
  }
  for (size_t i = 0; i < 4; ++i) {
    // Activation needs the external encoder to confirm the motor angle once (startup reference).
    if (!encoder_consistent(feedback[i], i)) {
      return false;
    }
  }
  encoder_bad_time_.fill(0.0);
  for (size_t i = 0; i < 4; ++i) {
    last_steering_targets_[i] = feedback[i].steering_position;
  }
  holding_modules_.fill(true);
  hold_fault_latched_ = false;
  previous_positions_ = positions(feedback);
  odometry_.emplace(config_.locations, yaw_, previous_positions_, output_.pose);
  setpoints_.reset(last_steering_targets_);
  previous_speeds_.fill(0.0);
  body_shaper_.reset();
  imu_fallback_ = true;
  output_ = stop(feedback, ControlStatus::command_timeout);
  active_ = true;
  refresh_snapshot(feedback);
  return true;
}

void ControlCore::deactivate() noexcept
{
  active_ = false;
  previous_speeds_.fill(0.0);
  body_shaper_.reset();
  setpoints_.reset();
  body_alignment_waiting_ = true;
  rotation_active_ = rotation_requested_ = false;
  rotation_stopped_time_ = 0.0;
  output_.status = ControlStatus::inactive;
  output_.drive_velocity.fill(0.0);
  output_.measured_twist = {};
  output_.valid_modules = 0U;
  output_.imu_fallback = true;
  refresh_snapshot({});
}

void ControlCore::observe(const ModuleFeedbackArray & feedback, std::optional<YawSample> imu)
{
  if (!odometry_) {
    return;
  }
  const auto current = positions(feedback);
  auto odometry_positions = current;
  std::array<SwerveModuleMeasurement, 4> measured{};
  std::size_t measured_count{0U};
  for (size_t i = 0; i < 4; ++i) {
    measured[i] = {
      feedback[i].wheel_velocity * config_.wheel_radius[i], current[i].angle_rad, current[i].valid};
    measured[i].valid = measured[i].valid && std::isfinite(measured[i].speed_mps);
    if (measured[i].valid) {
      ++measured_count;
    }
  }

  output_.measured_twist = {};
  output_.slipping_modules.fill(false);
  output_.slip_detected = false;
  output_.valid_modules = measured_count;
  const auto fit = chassis_speeds_with_slip_rejection(
    measured, config_.locations, config_.slip_residual_threshold);
  if (fit.has_value()) {
    output_.measured_twist = fit->speeds;
    output_.slipping_modules = fit->slipping_modules;
    output_.slip_detected = fit->slip_detected();
    output_.valid_modules = fit->used_module_count;
    for (size_t i = 0; i < 4; ++i) {
      if (fit->slipping_modules[i]) {
        odometry_positions[i].valid = false;
        odometry_positions[i].rejected_as_slip = true;
      }
    }
  } else if (measured_count >= 2U) {
    output_.slip_detected = true;
    output_.valid_modules = 0U;
    for (size_t i = 0; i < 4; ++i) {
      if (measured[i].valid) {
        output_.slipping_modules[i] = true;
        odometry_positions[i].valid = false;
        odometry_positions[i].rejected_as_slip = true;
      }
    }
  }

  const auto delta = wheel_chassis_delta_from_position_deltas(
    previous_positions_, odometry_positions, config_.locations);
  const bool fresh_imu = imu && std::isfinite(imu->yaw_rad) && std::isfinite(imu->rate_radps);
  if (fresh_imu) {
    if (!imu_fallback_) {
      yaw_ += wrap_pi(imu->yaw_rad - last_imu_yaw_);
    }
    last_imu_yaw_ = imu->yaw_rad;
  } else if (delta) {
    yaw_ += delta->dtheta_rad;
  }
  imu_fallback_ = !fresh_imu;
  output_.imu_fallback = imu_fallback_;
  output_.pose = odometry_->update(yaw_, odometry_positions);
  for (size_t i = 0; i < 4; ++i) {
    if (current[i].valid) {
      previous_positions_[i] = current[i];
    }
  }
  if (fresh_imu) {
    output_.measured_twist.omega_radps = imu->rate_radps;
  }
  if (
    !std::isfinite(output_.pose.x_m) || !std::isfinite(output_.pose.y_m) ||
    !std::isfinite(output_.pose.heading_rad) || !std::isfinite(output_.measured_twist.vx_mps) ||
    !std::isfinite(output_.measured_twist.vy_mps) ||
    !std::isfinite(output_.measured_twist.omega_radps)) {
    throw std::runtime_error("Non-finite swerve odometry");
  }
}

ControlOutput ControlCore::stop(const ModuleFeedbackArray & feedback, ControlStatus status)
{
  rotation_active_ = false;
  rotation_stopped_time_ = 0.0;
  holding_modules_.fill(true);
  if (active_ && hold_deviation(feedback, holding_modules_)) {
    hold_fault_latched_ = true;
  }
  output_.steering_position = last_steering_targets_;
  output_.drive_velocity.fill(0.0);
  previous_speeds_.fill(0.0);
  body_shaper_.reset();
  body_alignment_waiting_ = true;
  setpoints_.reset(last_steering_targets_);
  output_.status = active_ && hold_fault_latched_ ? ControlStatus::steering_hold_fault : status;
  return output_;
}

ControlOutput ControlCore::ramp_stop(
  const ModuleFeedbackArray & feedback, ControlStatus status, double dt)
{
  if (!positive(dt) || dt > config_.max_update_period) {
    return stop(feedback, status);
  }
  std::array<SwerveModuleState, 4> desired{};
  std::array<double, 4> angles{};
  std::array<bool, 4> holding{};
  holding.fill(true);
  for (size_t i = 0; i < 4; ++i) {
    desired[i] = {0.0, last_steering_targets_[i]};
    angles[i] = feedback[i].steering_position;
  }
  const auto targets = setpoints_.generate(desired, angles, dt, holding);
  std::array<double, 4> next_targets{};
  for (size_t i = 0; i < 4; ++i) {
    next_targets[i] = targets.modules[i].target_angle_rad;
    if (!steering_angle_within_limits(next_targets[i], steering_limits(config_, i))) {
      return stop(feedback, ControlStatus::steering_limit);
    }
  }
  last_steering_targets_ = next_targets;
  holding_modules_ = holding;
  output_.steering_position = next_targets;
  const auto max_step = config_.max_wheel_acceleration * dt;
  for (size_t i = 0; i < 4; ++i) {
    const double slower = std::max(0.0, std::abs(previous_speeds_[i]) - max_step);
    previous_speeds_[i] = std::copysign(slower, previous_speeds_[i]);
    output_.drive_velocity[i] = previous_speeds_[i] / config_.wheel_radius[i];
  }
  output_.status = status;
  return output_;
}

ControlOutput ControlCore::update(
  const ModuleFeedbackArray & feedback, ChassisSpeeds command, bool command_fresh, double dt,
  std::optional<YawSample> imu, bool rotation_requested)
{
  update_rotation_capability(feedback, command, command_fresh, dt, rotation_requested);
  observe(feedback, imu);
  update_command(feedback, command, command_fresh, dt);
  refresh_snapshot(feedback);
  return output_;
}

ControlOutput ControlCore::update_command(
  const ModuleFeedbackArray & feedback, ChassisSpeeds command, bool command_fresh, double dt)
{
  if (!active_) {
    return stop(feedback, ControlStatus::inactive);
  }
  if (hold_fault_latched_) {
    return stop(feedback, ControlStatus::steering_hold_fault);
  }
  if (!feedback_ready(feedback)) {
    return stop(feedback, ControlStatus::feedback_fault);
  }
  if (encoder_monitor_tripped(feedback, dt)) {
    return stop(feedback, ControlStatus::feedback_fault);
  }
  if (hold_deviation(feedback, holding_modules_)) {
    hold_fault_latched_ = true;
    return stop(feedback, ControlStatus::steering_hold_fault);
  }
  if (!command_fresh) {
    // Wheels ramp down on their own limit; a later command restarts the S-curve at rest.
    body_shaper_.reset();
    body_alignment_waiting_ = true;
    return ramp_stop(feedback, ControlStatus::command_timeout, dt);
  }
  if (
    !positive(dt) || dt > config_.max_update_period || !std::isfinite(command.vx_mps) ||
    !std::isfinite(command.vy_mps) || !std::isfinite(command.omega_radps)) {
    return stop(feedback, ControlStatus::invalid_command);
  }
  const auto linear = std::hypot(command.vx_mps, command.vy_mps);
  if (!std::isfinite(linear)) {
    return stop(feedback, ControlStatus::invalid_command);
  }
  if (linear > config_.max_linear_speed) {
    command.vx_mps *= config_.max_linear_speed / linear;
    command.vy_mps *= config_.max_linear_speed / linear;
  }
  const double angular_speed =
    rotation_active_ ? config_.max_rotation_speed : config_.max_angular_speed;
  command.omega_radps = std::clamp(command.omega_radps, -angular_speed, angular_speed);
  const auto previous_body = body_shaper_.velocity();
  const auto previous_acceleration = body_shaper_.body_acceleration();
  body_shaper_.set_angular_acceleration(
    rotation_active_ ? config_.max_rotation_acceleration : config_.max_angular_acceleration);
  // A revoked spin must enter the ordinary envelope before translation starts.
  // Do not suppress already-moving XY: numerical angular overshoot at the settled
  // limit must not repeatedly retarget an ordinary mixed trajectory to zero XY.
  if (
    !rotation_active_ &&
    std::hypot(previous_body.vx_mps, previous_body.vy_mps) <= config_.velocity_deadband &&
    (std::abs(previous_body.omega_radps) > config_.max_angular_speed ||
     std::abs(previous_acceleration.omega_radps) > config_.max_angular_acceleration)) {
    command.vx_mps = command.vy_mps = 0.0;
  }
  std::array<double, 4> angles{};
  for (size_t i = 0; i < 4; ++i) {
    angles[i] = feedback[i].steering_position;
  }
  // Apply the existing deadband to the requested command, not to the smooth ramp: clipping
  // the first/last ramp samples would introduce speed steps or prevent low-speed starts.
  const bool smoothing = body_shaper_.enabled();
  auto requested =
    inverse_kinematics(command, config_.locations, angles, config_.velocity_deadband);
  const bool zero_request = std::all_of(requested.begin(), requested.end(), [this](const auto & s) {
    return std::abs(s.speed_mps) < config_.velocity_deadband;
  });
  if (smoothing && zero_request) {
    command = {};
  }
  const bool waiting = smoothing && body_alignment_waiting_;
  if (waiting && zero_request) {
    return ramp_stop(feedback, ControlStatus::running, dt);
  }
  // In an alignment wait, plan steering from the raw target but keep all drive targets zero.
  // Once stopped and aligned, start the velocity trajectory from exactly zero on the next tick.
  const auto shaped = waiting ? command : body_shaper_.step(command, dt);
  if (!body_shaper_.valid()) {
    return stop(feedback, ControlStatus::invalid_command);
  }
  const double ramp_deadband = smoothing && !waiting ? 1e-12 : config_.velocity_deadband;
  auto desired =
    inverse_kinematics(discretize(shaped, dt), config_.locations, angles, ramp_deadband);
  if (std::all_of(desired.begin(), desired.end(), [ramp_deadband](const auto & state) {
        return std::abs(state.speed_mps) < ramp_deadband;
      })) {
    if (zero_request) {
      body_shaper_.reset();
      body_alignment_waiting_ = true;
    }
    return ramp_stop(feedback, ControlStatus::running, dt);
  }
  std::array<bool, 4> holding{};
  for (size_t i = 0; i < 4; ++i) {
    holding[i] = std::abs(desired[i].speed_mps) < ramp_deadband;
    if (holding[i]) {
      desired[i].angle_rad = last_steering_targets_[i];
      desired[i].speed_mps = 0.0;
    }
  }
  if (hold_deviation(feedback, holding)) {
    hold_fault_latched_ = true;
    return stop(feedback, ControlStatus::steering_hold_fault);
  }
  desaturate_wheel_speeds(desired, config_.max_wheel_speed);
  const auto targets = setpoints_.generate(desired, angles, dt, holding);
  const auto max_step = config_.max_wheel_acceleration * dt;
  std::array<double, 4> next_targets{};
  for (size_t i = 0; i < 4; ++i) {
    // Holding modules receive the generator's ramp-to-latch target, never a measurement.
    const auto target = targets.modules[i].target_angle_rad;
    if (!steering_angle_within_limits(target, steering_limits(config_, i))) {
      return stop(feedback, ControlStatus::steering_limit);
    }
    next_targets[i] = target;
  }
  // Commit the complete target array only after all four targets are valid.
  last_steering_targets_ = next_targets;
  holding_modules_ = holding;
  output_.steering_position = next_targets;
  for (size_t i = 0; i < 4; ++i) {
    // A gated module ramps its wheel down with the wheel acceleration limit (no speed step).
    const double wanted{targets.gated || waiting ? 0.0 : targets.modules[i].speed_mps};
    previous_speeds_[i] =
      std::clamp(wanted, previous_speeds_[i] - max_step, previous_speeds_[i] + max_step);
    output_.drive_velocity[i] = previous_speeds_[i] / config_.wheel_radius[i];
  }
  // Alignment stops retain the established wheel acceleration limit, rather than waiting
  // for the comfort ramp. No jerk guarantee applies across this stop/hold boundary.
  if (smoothing && (targets.gated || waiting)) {
    body_shaper_.reset();
    body_alignment_waiting_ = targets.gated || std::any_of(
                                                 previous_speeds_.begin(), previous_speeds_.end(),
                                                 [](double v) { return v != 0.0; });
  }
  output_.status = targets.gated || (waiting && body_alignment_waiting_)
                     ? ControlStatus::alignment_gated
                     : ControlStatus::running;
  return output_;
}
}  // namespace ctrl_swerve
