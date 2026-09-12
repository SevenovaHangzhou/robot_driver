#include "kinco_control_impl.hpp"
#include <algorithm>
#include <cmath>

namespace dm_swerve_driver {
namespace {
KincoAxisCondition policy_condition(
  const KincoAxisFeedback & axis, const KincoFaultReport & fault)
{
  if (fault.disposition == FaultDisposition::latch) {
    return KincoAxisCondition::latching_fault;
  }
  if (fault.disposition == FaultDisposition::recoverable) {
    return KincoAxisCondition::recoverable_fault;
  }
  const auto state = decode_ds402_state(axis.status_word);
  if (state == Ds402State::fault || state == Ds402State::fault_reaction_active ||
    state == Ds402State::unknown)
  {
    return KincoAxisCondition::latching_fault;
  }
  return state == Ds402State::operation_enabled ?
         KincoAxisCondition::enabled : KincoAxisCondition::disabled;
}
}

void KincoControlLoop::Impl::update_health(KincoClock::time_point now)
{
  const bool domain_ok = feedback_.raw.domain.healthy();
  std::array<bool, kKincoAxisCount> received{}, enabled{};
  for (std::size_t i{0U}; i < kKincoAxisCount; ++i) {
    const auto & axis = feedback_.raw.feedback[i];
    received[i] = domain_ok && axis.online;
    auto & health = health_[i];
    if (received[i]) {
      health.has_feedback = true;
      ++health.received_frames;
      health.consecutive_missed_frames = 0U;
      health.condition = policy_condition(axis, feedback_.axis_faults[i]);
      const auto expected_mode = static_cast<std::int8_t>(i < kSwerveModuleCount ? 8 : 9);
      if (axis.mode_display != expected_mode) {
        health.condition = KincoAxisCondition::latching_fault;
      }
    } else {
      ++health.missed_frames;
      ++health.consecutive_missed_frames;
    }
    enabled[i] = received[i] && health.enabled();
  }
  if (!domain_ok || !std::all_of(received.begin(), received.end(), [](bool v) {return v;})) {
    safety_.mark_transport_failure();
  }
  else {safety_.observe_feedback(received);}
  auto actions = safety_.recovery_actions(health_, now);
  if (clear_requested_.exchange(false) && !source_fault_ && domain_ok) {
    actions = safety_.manual_clear_actions();
    clearing_ = true;
    clear_deadline_ = now + std::chrono::seconds{2};
  }
  for (std::size_t i{0U}; i < kKincoAxisCount; ++i) {
    if (actions.reenable[i]) {
      recovering_[i] = true;
      if (decode_ds402_state(feedback_.raw.feedback[i].status_word) == Ds402State::fault) {
        reset_sequences_[i].request();
      }
    }
    if (safety_.fault_latched() && !clearing_) {
      recovering_[i] = false;
      reset_sequences_[i] = {};
    }
    if (enabled[i]) {recovering_[i] = false;}
    controls_[i] = enabled[i] ? 0x000FU : 0x0000U;
    if (recovering_[i] && received[i]) {
      controls_[i] = reset_sequences_[i].next_control_word(
        decode_ds402_state(feedback_.raw.feedback[i].status_word));
    }
  }
  if (clearing_) {
    if (std::all_of(enabled.begin(), enabled.end(), [](bool v) {return v;})) {
      static_cast<void>(safety_.complete_manual_clear(health_, enabled));
      clearing_ = false;
    } else if (now >= clear_deadline_) {
      static_cast<void>(safety_.complete_manual_clear(health_, enabled));
      clearing_ = false;
    }
  }
}

std::array<KincoModuleTarget, kSwerveModuleCount> KincoControlLoop::Impl::plan(
  KincoClock::time_point now, const ChassisSpeeds & command, bool timeout)
{
  const double nominal{1.0 / common_.control.rate_hz};
  const double dt = std::clamp(std::chrono::duration<double>{now - last_step_}.count(),
    0.5 * nominal, 2.0 * nominal);
  last_step_ = now;
  ChassisSpeeds limited{command};
  const double magnitude{std::hypot(command.vx_mps, command.vy_mps)};
  if (magnitude > common_.chassis.max_linear_speed_mps) {
    limited.vx_mps *= common_.chassis.max_linear_speed_mps / magnitude;
    limited.vy_mps *= common_.chassis.max_linear_speed_mps / magnitude;
  }
  limited.omega_radps = std::clamp(limited.omega_radps,
    -common_.chassis.max_angular_speed_radps, common_.chassis.max_angular_speed_radps);
  const bool hold = safety_.faulted() || (timeout && common_.control.hold_steer_on_timeout);
  const SteeringAngleLimits limits{common_.steering.joint_limit_min_rad,
    common_.steering.joint_limit_max_rad, common_.steering.joint_limit_margin_rad,
    common_.steering.joint_limit_tolerance_rad};
  std::array<double, kSwerveModuleCount> angles{};
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  for (std::size_t i{0U}; i < angles.size(); ++i) {
    angles[i] = clamp_steering_measurement_to_safe_range(sources_[i].angle_rad, limits);
    desired[i] = {0.0, angles[i]};
  }
  if (hold) {planner_.reset();}
  else {
    auto last = angles;
    if (timeout) {last.fill(0.0);}
    desired = inverse_kinematics(discretize(limited, dt), module_locations(common_), last,
      common_.chassis.velocity_deadband_mps);
    desaturate_wheel_speeds(desired, common_.chassis.max_wheel_speed_mps);
  }
  const auto setpoint = planner_.generate(desired, angles, dt);
  const bool gated = hold || setpoint.gated || safety_.faulted();
  std::array<KincoModuleTarget, kSwerveModuleCount> targets{};
  const double speed_step{common_.chassis.max_wheel_acceleration_mps2 * dt};
  for (std::size_t i{0U}; i < targets.size(); ++i) {
    previous_speed_[i] = gated ? 0.0 : std::clamp(setpoint.modules[i].speed_mps,
      previous_speed_[i] - speed_step, previous_speed_[i] + speed_step);
    targets[i] = {setpoint.modules[i].target_angle_rad, previous_speed_[i], !gated};
  }
  return targets;
}

ControlLoopOutput KincoControlLoop::Impl::update_odometry(KincoClock::time_point now,
  const std::optional<TimedYawSample> & imu, double yaw_rate)
{
  ControlLoopOutput result;
  result.timestamp = now;
  std::array<SwerveModulePosition, kSwerveModuleCount> positions{};
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measurements{};
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    const auto & module = feedback_.modules[i];
    const bool valid = feedback_.raw.domain.healthy() && sources_[i].valid &&
      health_[i].enabled() && health_[i + kSwerveModuleCount].enabled() &&
      feedback_.raw.feedback[i].online && feedback_.raw.feedback[i + kSwerveModuleCount].online;
    result.steering_angle_rad[i] = sources_[i].angle_rad;
    result.wheel_distance_m[i] = module.wheel_distance_m;
    result.wheel_velocity_mps[i] = module.wheel_speed_mps;
    positions[i] = {module.wheel_distance_m, sources_[i].angle_rad, valid};
    measurements[i] = {module.wheel_speed_mps, sources_[i].angle_rad, valid};
  }
  result.valid_module_count = static_cast<std::size_t>(std::count_if(
      measurements.begin(), measurements.end(), [](const auto & m) {return m.valid;}));
  const auto fit = chassis_speeds_with_slip_rejection(
    measurements, module_locations(common_), common_.odometry.slip_residual_threshold);
  if (fit) {
    result.measured_twist = fit->speeds;
    result.slip_detected = fit->slip_detected();
    result.slipping_modules = fit->slipping_modules;
    result.valid_module_count = fit->used_module_count;
    for (std::size_t i{0U}; i < positions.size(); ++i) {
      if (fit->slipping_modules[i]) {positions[i].valid = false; positions[i].rejected_as_slip = true;}
    }
  } else if (result.valid_module_count >= 2U) {
    // No consistent solution: do not integrate the rejected observations.
    result.slip_detected = true;
    result.valid_module_count = 0U;
    for (auto & p : positions) {p.rejected_as_slip = p.valid; p.valid = false;}
  }
  const auto wheel_delta = wheel_chassis_delta_from_position_deltas(
    previous_positions_, positions, module_locations(common_));
  const auto yaw = safety_.update_yaw(imu, wheel_delta ? wheel_delta->dtheta_rad : 0.0, now);
  result.imu_fallback = yaw.imu_fallback;
  if (!yaw.imu_fallback) {result.measured_twist.omega_radps = yaw_rate;}
  result.pose = odometry_->update(yaw.yaw_rad, positions);
  for (std::size_t i{0U}; i < positions.size(); ++i) {
    if (positions[i].valid || positions[i].rejected_as_slip) {
      previous_positions_[i] = positions[i];
      previous_positions_[i].valid = true;
      previous_positions_[i].rejected_as_slip = false;
    }
  }
  return result;
}

void KincoControlLoop::Impl::refresh_status()
{
  std::lock_guard<std::mutex> lock{status_mutex_};
  status_.initialized = initialized_;
  status_.running = running_.load();
  status_.faulted = safety_.faulted();
  status_.fault_latched = safety_.fault_latched();
  status_.transport_faulted = safety_.transport_faulted();
  status_.steering_limit_faulted = source_fault_;
  status_.recovery_attempts = safety_.recovery_attempts();
  status_.ethercat = feedback_.raw;
  status_.steering_sources = sources_;
  status_.encoder_heartbeat = encoder_cycle_.heartbeat_seen;
  status_.encoder_nmt_state = encoder_cycle_.nmt_state;
  status_.pose = output_.pose;
  status_.imu_fallback = output_.imu_fallback;
  status_.slip_detected = output_.slip_detected;
  status_.slipping_modules = output_.slipping_modules;
}
}  // namespace dm_swerve_driver
