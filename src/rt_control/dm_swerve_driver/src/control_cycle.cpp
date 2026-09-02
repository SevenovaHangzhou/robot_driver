#include "control_loop_impl.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] ChassisSpeeds limit_chassis_command(
  ChassisSpeeds command, const ChassisParameters & limits)
{
  const double linear_speed{std::hypot(command.vx_mps, command.vy_mps)};
  if (linear_speed > limits.max_linear_speed_mps) {
    const double scale{limits.max_linear_speed_mps / linear_speed};
    command.vx_mps *= scale;
    command.vy_mps *= scale;
  }
  command.omega_radps = std::clamp(
    command.omega_radps,
    -limits.max_angular_speed_radps,
    limits.max_angular_speed_radps);
  return command;
}

}  // namespace

std::array<DmMotor *, kMotorCount> ControlLoop::Impl::motor_pointers() noexcept
{
  std::array<DmMotor *, kMotorCount> motors{};
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    motors[index] = &modules_[index].steering_motor();
    motors[index + kSwerveModuleCount] = &modules_[index].drive_motor();
  }
  return motors;
}

StartupLogger ControlLoop::Impl::logger()
{
  return [this](DriverLogLevel level, const std::string & message) {
      log(level, message);
    };
}

void ControlLoop::Impl::log(
  DriverLogLevel level, const std::string & message) const noexcept
{
  try {
    if (callbacks_.log) {
      callbacks_.log(level, message);
    }
  } catch (...) {
  }
}

MailboxSnapshot ControlLoop::Impl::mailbox_snapshot() const
{
  std::lock_guard<std::mutex> lock{mailbox_mutex_};
  return MailboxSnapshot{command_mailbox_, yaw_mailbox_};
}

SteadyClock::duration ControlLoop::Impl::publish_period() const
{
  return std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>{1.0 / parameters_.odometry.publish_rate_hz});
}

std::array<SwerveModulePosition, kSwerveModuleCount>
ControlLoop::Impl::current_module_positions(
  const std::optional<std::array<bool, kMotorCount>> & received) const
{
  std::array<SwerveModulePosition, kSwerveModuleCount> positions{};
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    const bool initialized = modules_[index].steering_motor().position_initialized() &&
      modules_[index].drive_motor().position_initialized();
    const bool fresh = !received.has_value() ||
      ((*received)[index] && (*received)[index + kSwerveModuleCount]);
    if (initialized) {
      positions[index] = SwerveModulePosition{
        modules_[index].wheel_distance_m(),
        modules_[index].steering_angle_rad(),
        fresh};
    } else {
      positions[index].valid = false;
    }
  }
  return positions;
}

std::array<double, kSwerveModuleCount> ControlLoop::Impl::current_angles()
{
  std::array<double, kSwerveModuleCount> angles{};
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    if (modules_[index].steering_motor().position_initialized()) {
      angles[index] = modules_[index].steering_angle_rad();
      last_angles_[index] = angles[index];
    } else {
      angles[index] = last_angles_[index];
    }
  }
  return angles;
}

std::array<DmMotorHealth, kMotorCount> ControlLoop::Impl::motor_health() noexcept
{
  std::array<DmMotorHealth, kMotorCount> health{};
  const auto motors = motor_pointers();
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    health[index] = motors[index]->health();
  }
  return health;
}

CyclePlan ControlLoop::Impl::prepare_cycle(SteadyClock::time_point now)
{
  CyclePlan plan;
  plan.mailbox = mailbox_snapshot();
  const auto command_timestamp = plan.mailbox.command.valid ?
    std::optional<SteadyClock::time_point>{plan.mailbox.command.timestamp} : std::nullopt;
  plan.command = safety_.command_for_cycle(
    plan.mailbox.command.value, command_timestamp, now);
  const ChassisSpeeds limited{limit_chassis_command(
      plan.command.command, parameters_.chassis)};
  plan.discrete_command = discretize(limited, 1.0 / parameters_.control.rate_hz);
  const auto angles = current_angles();
  auto held_angles = angles;
  if (plan.command.timed_out && !parameters_.control.hold_steer_on_timeout) {
    held_angles.fill(0.0);
  }
  auto desired = inverse_kinematics(
    plan.discrete_command, module_locations(parameters_), held_angles,
    parameters_.chassis.velocity_deadband_mps);
  desaturate_wheel_speeds(desired, parameters_.chassis.max_wheel_speed_mps);
  plan.alignment = optimize_and_apply_alignment(
    desired, angles, parameters_.chassis.align_threshold_rad);
  plan.drive_gated = plan.alignment.gated || safety_.all_bus_silent(motor_health());
  return plan;
}

std::vector<CanFrame> ControlLoop::Impl::make_cycle_frames(const CyclePlan & plan)
{
  const double dt{1.0 / parameters_.control.rate_hz};
  std::vector<CanFrame> commands;
  commands.reserve(kMotorCount);
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    const auto command = modules_[index].make_command(
      plan.alignment.modules[index], dt, plan.drive_gated);
    const auto frames = modules_[index].encode_command_frames(command);
    commands.push_back(frames[0]);
    commands.push_back(frames[1]);
  }
  return commands;
}

FeedbackRouteResult ControlLoop::Impl::exchange_cycle_frames(
  const std::vector<CanFrame> & commands, SteadyClock::time_point now)
{
  transport_->write_batch(commands);
  const auto frames = transport_->collect(
    kMotorCount,
    now + std::chrono::microseconds{parameters_.can.feedback_deadline_us});
  const auto route = route_feedback_frames(
    frames, motor_pointers(),
    [&](const std::string & message) {log(DriverLogLevel::warning, message);});
  mark_missing_feedback(route.received);
  return route;
}

Pose2d ControlLoop::Impl::update_cycle_odometry(
  const MailboxSnapshot & mailbox,
  const std::array<bool, kMotorCount> & received,
  SteadyClock::time_point now,
  bool & imu_fallback)
{
  const auto positions = current_module_positions(received);
  double wheel_delta_yaw{0.0};
  if (previous_positions_.has_value()) {
    const auto wheel_delta = wheel_chassis_delta_from_position_deltas(
      *previous_positions_, positions, module_locations(parameters_));
    wheel_delta_yaw = wheel_delta.has_value() ? wheel_delta->dtheta_rad : 0.0;
  }
  previous_positions_ = positions;
  const auto imu = mailbox.yaw.valid ?
    std::optional<TimedYawSample>{TimedYawSample{mailbox.yaw.value, mailbox.yaw.timestamp}} :
    std::nullopt;
  const YawDecision yaw{safety_.update_yaw(imu, wheel_delta_yaw, now)};
  if (yaw.source_changed) {
    log(DriverLogLevel::warning,
      yaw.imu_fallback ? "IMU stale; using wheel-derived yaw" :
      "IMU recovered; realigned yaw offset without a pose jump");
  }
  imu_fallback = yaw.imu_fallback;
  return odometry_->update(yaw.yaw_rad, positions);
}

bool ControlLoop::Impl::execute_cycle(SteadyClock::time_point now)
{
  const CyclePlan plan{prepare_cycle(now)};
  const auto route = exchange_cycle_frames(make_cycle_frames(plan), now);
  bool imu_fallback{false};
  const Pose2d pose{update_cycle_odometry(
      plan.mailbox, route.received, now, imu_fallback)};
  process_recovery(now);
  const bool bus_silent{safety_.all_bus_silent(motor_health())};
  update_cycle_status(route, pose, plan.command.timed_out, imu_fallback, bus_silent);
  maybe_publish(now, pose, plan.discrete_command, plan.alignment.gated);
  return true;
}

void ControlLoop::Impl::process_recovery(SteadyClock::time_point now)
{
  dispatch_recovery_actions(safety_.recovery_actions(motor_health(), now), now);
  if (!clear_faults_requested_.exchange(false)) {
    return;
  }
  RecoveryActions manual{};
  manual.clear_fault.fill(true);
  manual.reenable.fill(true);
  dispatch_recovery_actions(manual, now);
}

void ControlLoop::Impl::mark_missing_feedback(
  const std::array<bool, kMotorCount> & received)
{
  const auto motors = motor_pointers();
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    if (!received[index]) {
      motors[index]->mark_feedback_missed();
    }
  }
}

void ControlLoop::Impl::update_cycle_status(
  const FeedbackRouteResult & route,
  const Pose2d & pose,
  bool command_timed_out,
  bool imu_fallback,
  bool bus_silent)
{
  const auto motors = motor_pointers();
  std::lock_guard<std::mutex> lock{status_mutex_};
  ++status_.completed_cycles;
  status_.unknown_frames += route.unknown_frames;
  status_.rejected_frames += route.rejected_frames;
  status_.command_timed_out = command_timed_out;
  status_.imu_fallback = imu_fallback;
  status_.bus_silent = bus_silent;
  status_.pose = pose;
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    status_.motors[index] = motors[index]->health();
    status_.motor_limits[index] = motors[index]->limits();
  }
}

void ControlLoop::Impl::dispatch_recovery_actions(
  const RecoveryActions & actions, SteadyClock::time_point now)
{
  send_special_actions(actions.clear_fault, SpecialCommand::clear_fault, now);
  send_special_actions(actions.reenable, SpecialCommand::enable, now);
}

void ControlLoop::Impl::send_special_actions(
  const std::array<bool, kMotorCount> & selected,
  SpecialCommand command,
  SteadyClock::time_point now)
{
  const auto motors = motor_pointers();
  std::vector<CanFrame> frames;
  for (std::size_t index{0U}; index < selected.size(); ++index) {
    if (selected[index]) {
      frames.push_back(motors[index]->special_command(command));
    }
  }
  if (frames.empty()) {
    return;
  }
  transport_->write_batch(frames);
  const auto replies = transport_->collect(
    frames.size(), now + std::chrono::microseconds{parameters_.can.feedback_deadline_us});
  static_cast<void>(route_feedback_frames(
      replies, motors,
      [&](const std::string & message) {log(DriverLogLevel::warning, message);}));
}

}  // namespace dm_swerve_driver
