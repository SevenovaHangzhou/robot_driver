#include "swerve_driver/relative_move_session.hpp"
#include <algorithm>
#include <cmath>

namespace swerve_driver
{
namespace
{
bool positive(double x) noexcept {return std::isfinite(x) && x > 0;}
bool within(double x, double ceiling) noexcept {return positive(x) && x <= ceiling;}
bool finite_fit(const std::optional<ChassisSpeedFit> & fit) noexcept
{
  if (!fit || !std::isfinite(fit->speeds.vx_mps) || !std::isfinite(fit->speeds.vy_mps) ||
    !std::isfinite(fit->speeds.omega_radps)) {return false;}
  for (double residual : fit->residual_mps) {if (!std::isfinite(residual)) {return false;}}
  return true;
}
}
void RelativeMoveSession::Ramp::reset(double distance, double velocity, double acceleration) noexcept
{
  *this = Ramp{};
  if (distance == 0) {p = 1; return;}
  a = acceleration / distance;
  peak = std::min(velocity / distance, std::sqrt(a));
  ta = peak / a;
  tc = std::max(0.0, 1.0 / peak - ta);
  duration = 2 * ta + tc;
  done = false;
}
void RelativeMoveSession::Ramp::advance(double dt) noexcept
{
  if (done) {return;}
  elapsed = std::min(duration, elapsed + dt);
  if (stopping) {
    p = stop_p + stop_v * elapsed - 0.5 * a * elapsed * elapsed;
    v = std::max(0.0, stop_v - a * elapsed);
  } else if (elapsed < ta) {
    p = 0.5 * a * elapsed * elapsed; v = a * elapsed;
  } else if (elapsed < ta + tc) {
    p = 0.5 * peak * ta + peak * (elapsed - ta); v = peak;
  } else {
    const double remaining = duration - elapsed;
    p = 1.0 - 0.5 * a * remaining * remaining; v = a * remaining;
  }
  if (elapsed >= duration) {done = true; v = 0;}
}
void RelativeMoveSession::Ramp::cancel() noexcept
{
  if (stopping || done) {return;}
  stopping = true; stop_p = p; stop_v = v; elapsed = 0; duration = v / a;
  if (v == 0) {done = true;}
}
bool RelativeMoveSession::configure(const MoveSessionConfig & c) noexcept
{
  if (configured_ || state_.active) {return false;}
  const double values[] = {
    c.max_steering_velocity, c.max_steering_acceleration, c.feedback_timeout, c.imu_timeout,
    c.imu_max_increment, c.max_update_period, c.stationary_wheel_velocity,
    c.stationary_steering_velocity, c.stationary_dwell, c.alignment_tolerance,
    c.alignment_dwell, c.alignment_timeout, c.steering_error, c.steering_error_dwell,
    c.encoder_difference, c.position_tolerance, c.velocity_tolerance, c.settling_dwell,
    c.settling_timeout, c.slip_threshold, c.slip_dwell, c.pose_translation_tolerance,
    c.pose_yaw_tolerance, c.yaw_discrepancy, c.switch_timeout, c.stop_timeout, c.max_goal_duration};
  for (double v : values) {if (!positive(v)) {return false;}}
  if (c.imu_max_increment >= kPi || c.alignment_tolerance > c.steering_error ||
    c.alignment_dwell >= c.alignment_timeout || c.settling_dwell >= c.settling_timeout ||
    c.stationary_dwell >= c.switch_timeout || c.settling_dwell >= c.stop_timeout)
  {return false;}
  RelativeMoveStart initial;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & m = c.motion.modules[i];
    const double center = (m.steering_limits.minimum_rad + m.steering_limits.maximum_rad) / 2;
    initial[i] = {center, center, 0};
  }
  RelativeMovePlanner check;
  if (check.configure({}, c.motion, initial) != RelativeMoveError::none) {return false;}
  std::array<Translation2d, 4> locations{};
  std::array<SwerveModuleMeasurement, 4> zero{};
  for (std::size_t i = 0; i < 4; ++i) {locations[i] = c.motion.modules[i].location;}
  if (!finite_fit(chassis_speeds_with_slip_rejection(zero, locations, c.slip_threshold))) {return false;}
  config_ = c; locations_ = locations; configured_ = true;
  return true;
}
bool RelativeMoveSession::enter_confirmed_operation(const MoveFeedback & f,
  const std::array<double, 4> & wheel, const std::array<double, 4> & steering) noexcept
{
  if (!configured_ || state_.active || state_.fault != MoveFault::none ||
    state_.mode != ChassisMode::unknown || !healthy_feedback(f) || !f.bus_ok || !f.drives_ok ||
    read_mode(f) != ChassisMode::operation) {return false;}
  for (size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(wheel[i]) ||
      std::abs(wheel[i] - f.positions[i].drive_position_rad) > config_.position_tolerance ||
      !steering_angle_within_limits(steering[i], config_.motion.modules[i].steering_limits) ||
      std::abs(f.wheel_velocity[i]) > config_.stationary_wheel_velocity ||
      std::abs(f.steering_velocity[i]) > config_.stationary_steering_velocity) {return false;}
  }
  feedback_ = f; output_.wheel_position = wheel; output_.steering_position = steering;
  output_.requested_mode = ChassisMode::operation; output_.inhibited = false;
  state_.mode = ChassisMode::operation; state_.phase = MovePhase::holding;
  return true;
}
bool RelativeMoveSession::healthy_feedback(const MoveFeedback & f) const noexcept
{
  if (!std::isfinite(f.age) || f.age < 0 || f.age > config_.feedback_timeout) {return false;}
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & p = f.positions[i];
    if (!std::isfinite(p.drive_position_rad) || !std::isfinite(f.wheel_velocity[i]) ||
      !std::isfinite(p.drive_position_rad * config_.motion.modules[i].wheel_radius_m) ||
      !std::isfinite(f.wheel_velocity[i] * config_.motion.modules[i].wheel_radius_m) ||
      !std::isfinite(f.steering_velocity[i]) ||
      !steering_angle_within_limits(p.steering_motor_rad, config_.motion.modules[i].steering_limits) ||
      !steering_measurement_within_tolerance(p.steering_measured_rad,
      config_.motion.modules[i].steering_limits)) {return false;}
  }
  return true;
}
bool RelativeMoveSession::valid_imu(const MoveFeedback & f) const noexcept
{
  return f.imu_valid && std::isfinite(f.imu_yaw) && std::isfinite(f.imu_age) &&
         f.imu_age >= 0 && f.imu_age <= config_.imu_timeout;
}
ChassisMode RelativeMoveSession::read_mode(const MoveFeedback & f) const noexcept
{
  int mode = f.drive_mode[0];
  for (std::size_t i = 0; i < 4; ++i) {
    if (f.steering_mode[i] != 8 || f.drive_mode[i] != mode) {return ChassisMode::unknown;}
  }
  return mode == 8 ? ChassisMode::operation :
         mode == 9 ? ChassisMode::navigation : ChassisMode::unknown;
}
uint16_t RelativeMoveSession::set_mode(ChassisMode mode, bool confirm) noexcept
{
  if (!confirm || (mode != ChassisMode::navigation && mode != ChassisMode::operation)) {return 1;}
  if (state_.active || state_.phase == MovePhase::switching) {return 3;}
  if (state_.fault != MoveFault::none) {return 5;}
  if (!configured_ || !healthy_feedback(feedback_) || !feedback_.bus_ok || !feedback_.drives_ok) {return 4;}
  if (!state_.stationary) {return 2;}
  output_.wheel_velocity.fill(0); output_.steering_velocity.fill(0);
  // The only measured-position seed is this fresh, stationary handoff.
  for (std::size_t i = 0; i < 4; ++i) {
    output_.wheel_position[i] = feedback_.positions[i].drive_position_rad;
    output_.steering_position[i] = feedback_.positions[i].steering_motor_rad;
  }
  output_.requested_mode = mode;
  state_.ready = false; state_.phase = MovePhase::switching; phase_time_ = 0;
  return 0;
}
uint16_t RelativeMoveSession::reset_fault(bool confirm) noexcept
{
  if (!confirm) {return 1;}
  if (state_.active || state_.phase == MovePhase::switching) {return 3;}
  if (!configured_ || !healthy_feedback(feedback_) || !valid_imu(feedback_) ||
    !feedback_.bus_ok || !feedback_.drives_ok) {return 4;}
  if (read_mode(feedback_) == ChassisMode::unknown) {return 5;}
  if (!state_.stationary) {return 2;}
  for (std::size_t i = 0; i < 4; ++i) {
    if (std::abs(feedback_.positions[i].steering_motor_rad -
      feedback_.positions[i].steering_measured_rad) > config_.encoder_difference ||
      (state_.mode == ChassisMode::operation &&
      (std::abs(output_.steering_position[i] - feedback_.positions[i].steering_measured_rad) >
      config_.position_tolerance ||
      std::abs(output_.steering_position[i] - feedback_.positions[i].steering_motor_rad) >
      config_.position_tolerance))) {return 4;}
  }
  // Reset does not change mode, targets or send a drive reset.
  if (read_mode(feedback_) != state_.mode) {return 5;}
  std::array<SwerveModuleMeasurement, 4> measurements{};
  for (std::size_t i = 0; i < 4; ++i) {
    measurements[i] = {feedback_.wheel_velocity[i] * config_.motion.modules[i].wheel_radius_m,
      feedback_.positions[i].steering_measured_rad, true};
    if (state_.mode == ChassisMode::operation &&
      std::abs(output_.wheel_position[i] - feedback_.positions[i].drive_position_rad) >
      config_.position_tolerance) {return 4;}
  }
  const auto fit = chassis_speeds_with_slip_rejection(measurements, locations_, config_.slip_threshold);
  if (!finite_fit(fit) || fit->slip_detected()) {return 4;}
  state_.fault = MoveFault::none; state_.phase = MovePhase::holding;
  state_.result = MoveResult::none; output_.inhibited = false;
  slip_time_ = steering_error_time_ = 0;
  return 0;
}
bool RelativeMoveSession::start(const MoveRequest & request) noexcept
{
  if (!state_.ready || !state_.stationary || !state_.imu_valid || state_.active ||
    state_.mode != ChassisMode::operation || !positive(request.max_duration) ||
    request.max_duration > config_.max_goal_duration) {return false;}
  auto limits = config_.motion;
  const auto & r = request.limits;
  if (!within(r.max_translation_velocity_mps, limits.max_translation_velocity_mps) ||
    !within(r.max_translation_acceleration_mps2, limits.max_translation_acceleration_mps2) ||
    !within(r.max_yaw_velocity_radps, limits.max_yaw_velocity_radps) ||
    !within(r.max_yaw_acceleration_radps2, limits.max_yaw_acceleration_radps2)) {return false;}
  limits.max_translation_velocity_mps = r.max_translation_velocity_mps;
  limits.max_translation_acceleration_mps2 = r.max_translation_acceleration_mps2;
  limits.max_yaw_velocity_radps = r.max_yaw_velocity_radps;
  limits.max_yaw_acceleration_radps2 = r.max_yaw_acceleration_radps2;
  for (std::size_t i = 0; i < 4; ++i) {
    if (!within(r.modules[i].max_wheel_velocity_radps, limits.modules[i].max_wheel_velocity_radps) ||
      !within(r.modules[i].max_wheel_acceleration_radps2, limits.modules[i].max_wheel_acceleration_radps2))
    {return false;}
    limits.modules[i].max_wheel_velocity_radps = r.modules[i].max_wheel_velocity_radps;
    limits.modules[i].max_wheel_acceleration_radps2 = r.modules[i].max_wheel_acceleration_radps2;
  }
  RelativeMovePlanner candidate;
  if (candidate.configure(request.goal, limits, feedback_.positions) != RelativeMoveError::none) {return false;}
  double distance = 0;
  for (const auto & m : candidate.plan().modules) {
    distance = std::max(distance, std::abs(m.steering_target_rad - m.steering_start_rad));
  }
  Ramp alignment;
  alignment.reset(distance, config_.max_steering_velocity, config_.max_steering_acceleration);
  const bool zero = candidate.sample().state == RelativeMoveState::complete;
  const double alignment_budget = zero ? 0 : alignment.duration +
    config_.stationary_dwell + config_.alignment_dwell;
  const double settling_budget = config_.settling_dwell + (zero ? 0 : config_.stationary_dwell);
  const double braking = zero ? 0 : candidate.plan().peak_progress_velocity / candidate.plan().progress_acceleration;
  if (!std::isfinite(alignment.duration) || alignment_budget > config_.alignment_timeout ||
    candidate.plan().duration_s + alignment_budget + settling_budget +
    3 * config_.max_update_period > request.max_duration ||
    std::max(braking, config_.max_steering_velocity / config_.max_steering_acceleration) +
    config_.stationary_dwell + config_.settling_dwell + 2 * config_.max_update_period >
    config_.stop_timeout) {return false;}
  planner_ = candidate; alignment_ = alignment;
  state_.active = true; state_.ready = false; state_.result = MoveResult::none;
  state_.phase = zero ? MovePhase::holding : MovePhase::aligning;
  state_.actual = {}; state_.imu_yaw = state_.wheel_yaw = 0; state_.quality = 0;
  state_.estimate_valid = true; ++state_.measurement_sequence;
  previous_imu_ = feedback_.imu_yaw;
  for (std::size_t i = 0; i < 4; ++i) {
    previous_positions_[i] = {feedback_.positions[i].drive_position_rad * config_.motion.modules[i].wheel_radius_m,
      feedback_.positions[i].steering_measured_rad, true};
  }
  elapsed_ = phase_time_ = settled_time_ = alignment_good_time_ = slip_time_ = steering_error_time_ = 0;
  duration_ = request.max_duration; stopping_alignment_ = false;
  references();
  return true;
}
void RelativeMoveSession::references() noexcept
{
  const auto & plan = planner_.plan();
  for (std::size_t i = 0; i < 4; ++i) {
    const double travel = plan.modules[i].steering_target_rad - plan.modules[i].steering_start_rad;
    output_.steering_position[i] = plan.modules[i].steering_start_rad + travel * alignment_.p;
    output_.steering_velocity[i] = travel * alignment_.v;
    output_.wheel_position[i] = planner_.sample().drive_position_rad[i];
    output_.wheel_velocity[i] = planner_.sample().drive_velocity_radps[i];
  }
  state_.progress = planner_.sample().progress;
}
void RelativeMoveSession::stop(MoveFault fault, MoveResult result) noexcept
{
  if (state_.fault == MoveFault::none) {state_.fault = fault;}
  if (!state_.active) {
    state_.ready = false; state_.phase = MovePhase::fault; output_.inhibited = true; return;
  }
  if (state_.phase != MovePhase::stopping) {
    stopping_alignment_ = state_.phase == MovePhase::aligning;
    alignment_.cancel(); planner_.cancel();
    state_.phase = MovePhase::stopping; phase_time_ = settled_time_ = 0;
    stop_result_ = result;
  } else if (result == MoveResult::faulted ||
    (result == MoveResult::timed_out && stop_result_ == MoveResult::canceled)) {stop_result_ = result;}
}
void RelativeMoveSession::cancel() noexcept
{
  if (state_.active) {stop(MoveFault::none, MoveResult::canceled);}
}
void RelativeMoveSession::finish(MoveResult result) noexcept
{
  state_.active = false; state_.result = result;
  state_.phase = state_.fault == MoveFault::none ? MovePhase::holding : MovePhase::fault;
  if (state_.estimate_valid) {
    const auto & goal = planner_.plan().goal;
    if (std::hypot(state_.actual.x_m - goal.x_m, state_.actual.y_m - goal.y_m) >
      config_.pose_translation_tolerance || std::abs(state_.actual.heading_rad - goal.heading_rad) >
      config_.pose_yaw_tolerance) {state_.quality |= 1U;}
  }
}
void RelativeMoveSession::estimate(const MoveFeedback & f) noexcept
{
  if (!state_.estimate_valid) {return;}
  const double yaw_delta = wrap_pi(f.imu_yaw - previous_imu_);
  if (!healthy_feedback(f) || !valid_imu(f) || !std::isfinite(yaw_delta) ||
    std::abs(yaw_delta) > config_.imu_max_increment) {
    state_.estimate_valid = false; state_.quality |= 4U; return;
  }
  std::array<SwerveModulePosition, 4> positions{};
  for (std::size_t i = 0; i < 4; ++i) {
    positions[i] = {f.positions[i].drive_position_rad * config_.motion.modules[i].wheel_radius_m,
      f.positions[i].steering_measured_rad, true};
  }
  const auto wheel = wheel_chassis_delta_from_position_deltas(previous_positions_, positions, locations_);
  const auto translation = wheel_translation_from_position_deltas(previous_positions_, positions, locations_, yaw_delta);
  if (!wheel || !translation) {state_.estimate_valid = false; state_.quality |= 4U; return;}
  const double imu_yaw = state_.imu_yaw + yaw_delta;
  const double wheel_yaw = state_.wheel_yaw + wheel->dtheta_rad;
  const auto actual = integrate_pose(state_.actual, {translation->x, translation->y, yaw_delta}, imu_yaw);
  if (!std::isfinite(imu_yaw) || !std::isfinite(wheel_yaw) ||
    !std::isfinite(actual.x_m) || !std::isfinite(actual.y_m)) {
    state_.estimate_valid = false; state_.quality |= 4U; return;
  }
  state_.imu_yaw = imu_yaw; state_.wheel_yaw = wheel_yaw; state_.actual = actual;
  if (std::abs(state_.imu_yaw - state_.wheel_yaw) > config_.yaw_discrepancy) {state_.quality |= 2U;}
  previous_positions_ = positions; previous_imu_ = f.imu_yaw; ++state_.measurement_sequence;
}
void RelativeMoveSession::update(double dt, const MoveFeedback & f) noexcept
{
  feedback_ = f;
  if (!configured_) {return;}
  const bool timing = positive(dt) && dt <= config_.max_update_period;
  const bool healthy = healthy_feedback(f);
  state_.imu_valid = valid_imu(f);
  bool stationary = healthy && f.bus_ok && f.drives_ok;
  for (std::size_t i = 0; i < 4; ++i) {
    stationary = stationary && std::abs(f.wheel_velocity[i]) <= config_.stationary_wheel_velocity &&
      std::abs(f.steering_velocity[i]) <= config_.stationary_steering_velocity;
  }
  stationary_time_ = stationary && timing ? stationary_time_ + dt : 0;
  state_.stationary = stationary_time_ >= config_.stationary_dwell;
  state_.ready = false;
  if (!timing) {
    stop(MoveFault::execution, MoveResult::faulted);
    output_.inhibited = true;
    if (state_.active) {
      state_.estimate_valid = false; state_.quality |= 4U; finish(MoveResult::faulted);
    }
    return;
  }
  const auto readback = healthy ? read_mode(f) : ChassisMode::unknown;
  if (state_.mode == ChassisMode::unknown && state_.phase == MovePhase::idle && state_.stationary) {
    state_.mode = readback;
    output_.requested_mode = readback;
    for (std::size_t i = 0; i < 4; ++i) {
      output_.steering_position[i] = f.positions[i].steering_motor_rad;
      output_.wheel_position[i] = f.positions[i].drive_position_rad;
    }
  }
  // No usable drive authority: retain last references, inhibit output, never claim a controlled stop.
  if (!healthy || !f.bus_ok || !f.drives_ok) {
    const auto fault = !f.bus_ok ? MoveFault::bus : !f.drives_ok ? MoveFault::drive : MoveFault::feedback_invalid;
    if (state_.active || state_.mode != ChassisMode::unknown || state_.phase == MovePhase::switching) {
      stop(fault, MoveResult::faulted);
      output_.inhibited = true;
      if (state_.active) {state_.estimate_valid = false; state_.quality |= 4U; finish(MoveResult::faulted);}
    }
    return;
  }
  if (state_.phase == MovePhase::switching) {
    phase_time_ += dt;
    state_.mode = readback;
    if (!state_.stationary || phase_time_ > config_.switch_timeout) {
      stop(MoveFault::mode_switch, MoveResult::faulted);
    } else if (readback == output_.requested_mode) {
      state_.phase = MovePhase::holding; output_.inhibited = false;
    }
    return;
  }
  if (state_.mode != ChassisMode::unknown && readback != state_.mode) {
    state_.mode = readback;
    stop(MoveFault::mode_switch, MoveResult::faulted); output_.inhibited = true;
    if (state_.active) {finish(MoveResult::faulted);}
    return;
  }
  bool steering_bad = false;
  std::array<SwerveModuleMeasurement, 4> measurements{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & p = f.positions[i];
    steering_bad = steering_bad || std::abs(p.steering_motor_rad - p.steering_measured_rad) > config_.encoder_difference;
    if (state_.active) {
      steering_bad = steering_bad || std::abs(output_.steering_position[i] - p.steering_measured_rad) > config_.steering_error;
    }
    measurements[i] = {f.wheel_velocity[i] * config_.motion.modules[i].wheel_radius_m, p.steering_measured_rad, true};
  }
  const auto fit = chassis_speeds_with_slip_rejection(measurements, locations_, config_.slip_threshold);
  if (!finite_fit(fit)) {
    stop(MoveFault::feedback_invalid, MoveResult::faulted); output_.inhibited = true;
    if (state_.active) {
      state_.estimate_valid = false; state_.quality |= 4U; finish(MoveResult::faulted);
    }
    return;
  }
  const bool slipping = fit->slip_detected();
  slip_time_ = slipping ? slip_time_ + dt : 0;
  steering_error_time_ = steering_bad ? steering_error_time_ + dt : 0;
  if (slip_time_ >= config_.slip_dwell) {stop(MoveFault::slip, MoveResult::faulted);}
  if (steering_error_time_ >= config_.steering_error_dwell) {stop(MoveFault::steering_error, MoveResult::faulted);}
  if (!state_.active) {
    state_.ready = state_.fault == MoveFault::none && state_.mode != ChassisMode::unknown &&
      state_.imu_valid && !steering_bad && !slipping;
    output_.inhibited = state_.fault != MoveFault::none || state_.mode == ChassisMode::unknown;
    return;
  }
  const double imu_increment = wrap_pi(f.imu_yaw - previous_imu_);
  if (state_.imu_valid && (!std::isfinite(imu_increment) ||
    std::abs(imu_increment) > config_.imu_max_increment)) {
    state_.imu_valid = false;
  }
  estimate(f);
  if (!state_.imu_valid) {stop(MoveFault::imu_lost, MoveResult::faulted);}
  else if (!state_.estimate_valid && state_.fault == MoveFault::none) {stop(MoveFault::feedback_invalid, MoveResult::faulted);}
  elapsed_ += dt; phase_time_ += dt;
  if (elapsed_ > duration_ && state_.fault == MoveFault::none) {stop(MoveFault::timeout, MoveResult::timed_out);}
  if (state_.phase == MovePhase::aligning) {
    alignment_.advance(dt); references();
    bool aligned = alignment_.done && state_.stationary;
    for (std::size_t i = 0; i < 4; ++i) {
      aligned = aligned && std::abs(f.positions[i].steering_measured_rad - output_.steering_position[i]) <= config_.alignment_tolerance;
    }
    alignment_good_time_ = aligned ? alignment_good_time_ + dt : 0;
    if (phase_time_ > config_.alignment_timeout) {stop(MoveFault::steering_error, MoveResult::faulted);}
    else if (alignment_good_time_ >= config_.alignment_dwell) {state_.phase = MovePhase::executing; phase_time_ = 0;}
  } else if (state_.phase == MovePhase::executing) {
    (void)planner_.advance(dt); references();
    if (planner_.sample().state == RelativeMoveState::complete) {state_.phase = MovePhase::holding; phase_time_ = 0;}
  } else if (state_.phase == MovePhase::stopping) {
    if (stopping_alignment_) {alignment_.advance(dt);}
    else {(void)planner_.advance(dt);}
    references();
    if (phase_time_ > config_.stop_timeout) {
      if (state_.fault == MoveFault::none) {state_.fault = MoveFault::execution;}
      output_.inhibited = true; finish(MoveResult::faulted); return;
    }
  }
  const bool reference_stopped = alignment_.done &&
    (planner_.sample().state == RelativeMoveState::complete || planner_.sample().state == RelativeMoveState::canceled);
  if (state_.phase == MovePhase::holding || (state_.phase == MovePhase::stopping && reference_stopped)) {
    bool settled = state_.stationary;
    for (std::size_t i = 0; i < 4; ++i) {
      settled = settled && std::abs(f.positions[i].drive_position_rad - output_.wheel_position[i]) <= config_.position_tolerance &&
        std::abs(f.wheel_velocity[i]) <= config_.velocity_tolerance &&
        std::abs(f.steering_velocity[i]) <= config_.velocity_tolerance &&
        std::abs(f.positions[i].steering_motor_rad - output_.steering_position[i]) <= config_.position_tolerance &&
        std::abs(f.positions[i].steering_measured_rad - output_.steering_position[i]) <= config_.position_tolerance;
    }
    settled_time_ = settled ? settled_time_ + dt : 0;
    if (settled_time_ >= config_.settling_dwell) {
      finish(state_.phase == MovePhase::stopping ? stop_result_ : MoveResult::succeeded);
    } else if (state_.phase == MovePhase::holding && phase_time_ > config_.settling_timeout) {
      stop(MoveFault::execution, MoveResult::faulted);
    }
  }
}
}  // namespace swerve_driver
