#include "swerve_driver/relative_move_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace swerve_driver
{
namespace
{
bool positive(double value) noexcept {return std::isfinite(value) && value > 0.0;}

bool valid_config(const RelativeMoveConfig & config) noexcept
{
  if (!positive(config.max_translation_velocity_mps) ||
    config.max_translation_velocity_mps > 0.2 ||
    !positive(config.max_translation_acceleration_mps2) ||
    !positive(config.max_yaw_velocity_radps) || !positive(config.max_yaw_acceleration_radps2))
  {
    return false;
  }
  for (std::size_t i = 0; i < kSwerveModuleCount; ++i) {
    const auto & module = config.modules[i];
    if (!std::isfinite(module.location.x) || !std::isfinite(module.location.y) ||
      !positive(module.wheel_radius_m) || !valid_steering_angle_limits(module.steering_limits) ||
      !positive(module.max_wheel_velocity_radps) ||
      !positive(module.max_wheel_acceleration_radps2))
    {
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (module.location.x == config.modules[j].location.x &&
        module.location.y == config.modules[j].location.y)
      {
        return false;
      }
    }
  }
  return true;
}

// SE(2) log uses the goal's bounded yaw directly, preserving tiny signed rotations.
BodyDelta relative_log(const Pose2d & goal) noexcept
{
  const double half = goal.heading_rad / 2.0;
  const double theta2 = goal.heading_rad * goal.heading_rad;
  const double coefficient = std::abs(goal.heading_rad) < 1e-4 ?
    1.0 - theta2 / 12.0 - theta2 * theta2 / 720.0 : half / std::tan(half);
  return {coefficient * goal.x_m + half * goal.y_m,
    coefficient * goal.y_m - half * goal.x_m, goal.heading_rad};
}

Pose2d relative_exp(const BodyDelta & log, double progress) noexcept
{
  const double theta = log.dtheta_rad * progress;
  const double theta2 = theta * theta;
  const double sine_scale = std::abs(theta) < 1e-4 ?
    1.0 - theta2 / 6.0 + theta2 * theta2 / 120.0 : std::sin(theta) / theta;
  // 2*sin(theta/2)^2 avoids the cancellation in (1-cos(theta))/theta.
  const double half_sine = std::sin(theta / 2.0);
  const double cosine_scale = std::abs(theta) < 1e-4 ?
    theta * (0.5 - theta2 / 24.0 + theta2 * theta2 / 720.0) :
    2.0 * half_sine * half_sine / theta;
  return {progress * (log.dx_m * sine_scale - log.dy_m * cosine_scale),
    progress * (log.dx_m * cosine_scale + log.dy_m * sine_scale), theta};
}

void constrain(double & normalized_limit, double physical_limit, double travel) noexcept
{
  if (travel > 0.0) {
    normalized_limit = std::min(normalized_limit, physical_limit / travel);
  }
}
}  // namespace

RelativeMoveError RelativeMovePlanner::configure(
  const Pose2d & goal, const RelativeMoveConfig & config,
  const RelativeMoveStart & start) noexcept
{
  if (sample_.state == RelativeMoveState::running || sample_.state == RelativeMoveState::stopping) {
    return RelativeMoveError::busy;
  }
  if (!std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) || !std::isfinite(goal.heading_rad)) {
    return RelativeMoveError::invalid_goal;
  }
  if (std::hypot(goal.x_m, goal.y_m) > 0.700 || std::abs(goal.heading_rad) > kPi / 12.0) {
    return RelativeMoveError::goal_out_of_range;
  }
  if (!valid_config(config)) {
    return RelativeMoveError::invalid_config;
  }

  RelativeMovePlan candidate{};
  candidate.goal = goal;
  candidate.body_log = relative_log(goal);
  const double translation = std::hypot(candidate.body_log.dx_m, candidate.body_log.dy_m);
  const double yaw = std::abs(candidate.body_log.dtheta_rad);
  const bool no_motion = translation == 0.0 && yaw == 0.0;
  double velocity = std::numeric_limits<double>::max();
  double acceleration = std::numeric_limits<double>::max();
  constrain(velocity, config.max_translation_velocity_mps, translation);
  constrain(velocity, config.max_yaw_velocity_radps, yaw);
  constrain(acceleration, config.max_yaw_acceleration_radps2, yaw);
  // For mixed motion reserve equal acceleration budget for tangential and normal
  // components: |a_base| = translation*hypot(s_ddot, yaw*s_dot^2).
  // This conservative constant bound preserves a true scalar trapezoid on the curve.
  const double linear_budget = config.max_translation_acceleration_mps2 /
    (yaw > 0.0 && translation > 0.0 ? std::sqrt(2.0) : 1.0);
  constrain(acceleration, linear_budget, translation);
  if (yaw > 0.0 && translation > 0.0) {
    velocity = std::min(velocity, std::sqrt((linear_budget / translation) / yaw));
  }

  for (std::size_t i = 0; i < kSwerveModuleCount; ++i) {
    const auto & module = config.modules[i];
    const auto & initial = start[i];
    if (!steering_angle_within_limits(initial.steering_motor_rad, module.steering_limits) ||
      !steering_measurement_within_tolerance(initial.steering_measured_rad, module.steering_limits) ||
      !std::isfinite(initial.drive_position_rad))
    {
      return RelativeMoveError::invalid_start;
    }
    auto & output = candidate.modules[i];
    output.steering_start_rad = initial.steering_motor_rad;
    output.steering_target_rad = initial.steering_motor_rad;
    output.drive_start_rad = initial.drive_position_rad;
    const double wheel_x = candidate.body_log.dx_m - candidate.body_log.dtheta_rad * module.location.y;
    const double wheel_y = candidate.body_log.dy_m + candidate.body_log.dtheta_rad * module.location.x;
    const double distance = std::hypot(wheel_x, wheel_y);
    const double travel = distance / module.wheel_radius_m;
    if (!std::isfinite(travel) || (distance > 0.0 && travel == 0.0)) {
      return RelativeMoveError::numeric_range;
    }
    if (distance > 0.0) {
      const double measured = std::clamp(initial.steering_measured_rad,
        module.steering_limits.minimum_rad + module.steering_limits.margin_rad,
        module.steering_limits.maximum_rad - module.steering_limits.margin_rad);
      const double heading = std::atan2(wheel_y, wheel_x);
      const auto forward = nearest_equivalent_steering_target(heading, measured, module.steering_limits);
      const auto reverse = nearest_equivalent_steering_target(heading + kPi, measured, module.steering_limits);
      if (!forward && !reverse) {
        return RelativeMoveError::steering_infeasible;
      }
      const bool reversed = reverse && (!forward || std::abs(*reverse - measured) < std::abs(*forward - measured));
      output.steering_target_rad = reversed ? *reverse : *forward;
      output.drive_travel_rad = reversed ? -travel : travel;
      if (!steering_angle_within_limits(output.steering_target_rad, module.steering_limits)) {
        return RelativeMoveError::steering_infeasible;
      }
      const double end_position = output.drive_start_rad + output.drive_travel_rad;
      if (!std::isfinite(end_position) || end_position == output.drive_start_rad) {
        return RelativeMoveError::numeric_range;
      }
      constrain(velocity, module.max_wheel_velocity_radps, travel);
      constrain(acceleration, module.max_wheel_acceleration_radps2, travel);
    }
    output.steering_alignment_error_rad = output.steering_target_rad - initial.steering_measured_rad;
  }

  if (!no_motion) {
    if (!positive(velocity) || !positive(acceleration)) {
      return RelativeMoveError::numeric_range;
    }
    candidate.progress_velocity_limit = velocity;
    candidate.progress_acceleration = acceleration;
    candidate.peak_progress_velocity = std::min(velocity, std::sqrt(acceleration));
    candidate.acceleration_duration_s = candidate.peak_progress_velocity / acceleration;
    const double ramp_distance = candidate.peak_progress_velocity * candidate.acceleration_duration_s;
    candidate.cruise_duration_s = std::max(0.0, 1.0 - ramp_distance) / candidate.peak_progress_velocity;
    candidate.duration_s = 2.0 * candidate.acceleration_duration_s + candidate.cruise_duration_s;
    if (!positive(candidate.acceleration_duration_s) || !positive(candidate.duration_s) ||
      !std::isfinite(candidate.cruise_duration_s))
    {
      return RelativeMoveError::numeric_range;
    }
  }
  plan_ = candidate;
  elapsed_s_ = 0.0;
  sample_ = {};
  sample_.state = no_motion ? RelativeMoveState::complete : RelativeMoveState::running;
  sample_.progress = no_motion ? 1.0 : 0.0;
  sample_.progress_acceleration = candidate.progress_acceleration;
  update_targets();
  return RelativeMoveError::none;
}

bool RelativeMovePlanner::advance(double dt_seconds) noexcept
{
  if (!positive(dt_seconds) || sample_.state == RelativeMoveState::unconfigured) {
    return false;
  }
  if (sample_.state == RelativeMoveState::complete || sample_.state == RelativeMoveState::canceled) {
    return true;
  }
  const bool stopping = sample_.state == RelativeMoveState::stopping;
  const double duration = stopping ? cancel_duration_s_ : plan_.duration_s;
  // Subtract first so an arbitrarily large finite dt cannot overflow elapsed time.
  elapsed_s_ = dt_seconds >= duration - elapsed_s_ ? duration : elapsed_s_ + dt_seconds;
  const double acceleration = plan_.progress_acceleration;
  if (stopping) {
    const double remaining = duration - elapsed_s_;
    sample_.progress_velocity = acceleration * remaining;
    sample_.progress = cancel_start_progress_ + elapsed_s_ *
      (cancel_start_velocity_ - 0.5 * acceleration * elapsed_s_);
    sample_.progress_acceleration = -acceleration;
    if (elapsed_s_ >= duration) {
      sample_.progress = cancel_end_progress_;
      sample_.progress_velocity = 0.0;
      sample_.progress_acceleration = 0.0;
      sample_.state = RelativeMoveState::canceled;
    }
  } else if (elapsed_s_ >= duration) {
    sample_.progress = 1.0;
    sample_.progress_velocity = 0.0;
    sample_.progress_acceleration = 0.0;
    sample_.state = RelativeMoveState::complete;
  } else if (elapsed_s_ < plan_.acceleration_duration_s) {
    sample_.progress_velocity = acceleration * elapsed_s_;
    sample_.progress = 0.5 * sample_.progress_velocity * elapsed_s_;
    sample_.progress_acceleration = acceleration;
  } else if (elapsed_s_ < plan_.acceleration_duration_s + plan_.cruise_duration_s) {
    sample_.progress_velocity = plan_.peak_progress_velocity;
    sample_.progress = plan_.peak_progress_velocity *
      (elapsed_s_ - 0.5 * plan_.acceleration_duration_s);
    sample_.progress_acceleration = 0.0;
  } else {
    const double remaining = duration - elapsed_s_;
    sample_.progress_velocity = acceleration * remaining;
    sample_.progress = 1.0 - 0.5 * sample_.progress_velocity * remaining;
    sample_.progress_acceleration = -acceleration;
  }
  update_targets();
  return true;
}

void RelativeMovePlanner::cancel() noexcept
{
  if (sample_.state != RelativeMoveState::running) {
    return;
  }
  cancel_start_progress_ = sample_.progress;
  cancel_start_velocity_ = sample_.progress_velocity;
  cancel_duration_s_ = cancel_start_velocity_ / plan_.progress_acceleration;
  cancel_end_progress_ = std::min(1.0,
    cancel_start_progress_ + 0.5 * cancel_start_velocity_ * cancel_duration_s_);
  elapsed_s_ = 0.0;
  sample_.state = cancel_duration_s_ > 0.0 ? RelativeMoveState::stopping : RelativeMoveState::canceled;
  sample_.progress_acceleration = cancel_duration_s_ > 0.0 ? -plan_.progress_acceleration : 0.0;
}

void RelativeMovePlanner::update_targets() noexcept
{
  sample_.planned_pose = relative_exp(plan_.body_log, sample_.progress);
  sample_.body_velocity = {plan_.body_log.dx_m * sample_.progress_velocity,
    plan_.body_log.dy_m * sample_.progress_velocity,
    plan_.body_log.dtheta_rad * sample_.progress_velocity};
  for (std::size_t i = 0; i < kSwerveModuleCount; ++i) {
    sample_.drive_position_rad[i] = plan_.modules[i].drive_start_rad +
      plan_.modules[i].drive_travel_rad * sample_.progress;
    sample_.drive_velocity_radps[i] = plan_.modules[i].drive_travel_rad * sample_.progress_velocity;
  }
}
}  // namespace swerve_driver
