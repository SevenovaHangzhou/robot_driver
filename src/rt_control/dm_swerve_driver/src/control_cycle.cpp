#include "control_loop_impl.hpp"
#include <algorithm>
#include <chrono>
#include <optional>
#include <utility>
namespace dm_swerve_driver {
namespace {
constexpr std::size_t kMaxFeedbackCollections{16U};
void update_valid_positions(
  std::array<SwerveModulePosition, kSwerveModuleCount> & baselines,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & positions)
{
  for (std::size_t index{0U}; index < positions.size(); ++index) {
    if (positions[index].valid || positions[index].rejected_as_slip) {
      baselines[index] = positions[index];
      baselines[index].valid = true;
      baselines[index].rejected_as_slip = false;
    }
  }
}
[[nodiscard]] std::size_t count_valid_modules(
  const std::array<bool, kMotorCount> & received,
  const std::array<SwerveModule, kSwerveModuleCount> & modules) noexcept
{
  std::size_t count{0U};
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    if (received[index] && received[index + kSwerveModuleCount] &&
      modules[index].steering_motor().health().enabled() &&
      modules[index].drive_motor().health().enabled())
    {
      ++count;
    }
  }
  return count;
}
void merge_feedback_routes(
  FeedbackRouteResult & destination,
  const FeedbackRouteResult & source) noexcept
{
  for (std::size_t index{0U}; index < destination.received.size(); ++index) {
    destination.received[index] = destination.received[index] || source.received[index];
  }
  destination.accepted_frames += source.accepted_frames;
  destination.unknown_frames += source.unknown_frames;
  destination.rejected_frames += source.rejected_frames;
  destination.stale_frames += source.stale_frames;
}
[[nodiscard]] bool all_feedback_received(
  const std::array<bool, kMotorCount> & received) noexcept
{
  return std::all_of(received.begin(), received.end(), [](bool value) {return value;});
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
      modules_[index].drive_motor().position_initialized() &&
      modules_[index].steering_motor().health().enabled() &&
      modules_[index].drive_motor().health().enabled();
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

std::array<SwerveModuleMeasurement, kSwerveModuleCount>
ControlLoop::Impl::current_module_measurements(
  const std::array<bool, kMotorCount> & received) const
{
  std::array<SwerveModuleMeasurement, kSwerveModuleCount> measurements{};
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    const bool initialized = modules_[index].steering_motor().position_initialized() &&
      modules_[index].drive_motor().position_initialized() &&
      modules_[index].steering_motor().health().enabled() &&
      modules_[index].drive_motor().health().enabled();
    measurements[index].valid = initialized && received[index] &&
      received[index + kSwerveModuleCount];
    if (measurements[index].valid) {
      measurements[index].speed_mps = modules_[index].wheel_velocity_mps();
      measurements[index].angle_rad = modules_[index].steering_angle_rad();
    }
  }
  return measurements;
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

std::vector<CanFrame> ControlLoop::Impl::make_cycle_frames(const CyclePlan & plan)
{
  std::vector<CanFrame> commands;
  commands.reserve(kMotorCount);
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    const auto command = modules_[index].make_command(
      plan.alignment.modules[index], plan.dt_seconds, plan.drive_gated,
      plan.hold_steering);
    const auto frames = modules_[index].encode_command_frames(command);
    commands.push_back(frames[0]);
    commands.push_back(frames[1]);
  }
  return commands;
}

FeedbackRouteResult ControlLoop::Impl::exchange_cycle_frames(
  const std::vector<CanFrame> & commands, SteadyClock::time_point now)
{
  const auto command_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  transport_->write_batch(commands);
  const auto deadline = now +
    std::chrono::microseconds{parameters_.can.feedback_deadline_us};
  FeedbackRouteResult route{};
  for (std::size_t attempt{0U}; attempt < kMaxFeedbackCollections &&
    !all_feedback_received(route.received); ++attempt)
  {
    const auto frames = transport_->collect(kMotorCount, deadline);
    if (frames.empty()) {
      break;
    }
    const auto chunk = route_feedback_frames(
      frames, motor_pointers(),
      [&](const std::string & message) {log(DriverLogLevel::warning, message);},
      command_timestamp);
    merge_feedback_routes(route, chunk);
  }
  safety_.observe_feedback(route.received);
  mark_missing_feedback(route.received);
  return route;
}

CycleOdometry ControlLoop::Impl::update_cycle_odometry(
  const MailboxSnapshot & mailbox,
  const std::array<bool, kMotorCount> & received,
  SteadyClock::time_point now)
{
  CycleOdometry result;
  auto positions = current_module_positions(received);
  const auto wheel_fit = chassis_speeds_with_slip_rejection(
    current_module_measurements(received), module_locations(parameters_),
    parameters_.odometry.slip_residual_threshold);
  if (wheel_fit.has_value()) {
    result.measured_twist = wheel_fit->speeds;
    result.slipping_modules = wheel_fit->slipping_modules;
    result.valid_module_count = wheel_fit->used_module_count;
    for (std::size_t index{0U}; index < positions.size(); ++index) {
      if (result.slipping_modules[index]) {
        positions[index].valid = false;
        positions[index].rejected_as_slip = true;
      }
    }
  } else {
    result.valid_module_count = count_valid_modules(received, modules_);
  }
  double wheel_delta_yaw{0.0};
  if (previous_positions_.has_value()) {
    const auto wheel_delta = wheel_chassis_delta_from_position_deltas(
      *previous_positions_, positions, module_locations(parameters_));
    wheel_delta_yaw = wheel_delta.has_value() ? wheel_delta->dtheta_rad : 0.0;
  }
  if (previous_positions_.has_value()) {
    update_valid_positions(*previous_positions_, positions);
  } else {
    previous_positions_ = positions;
  }
  const auto imu = mailbox.yaw.valid ?
    std::optional<TimedYawSample>{TimedYawSample{mailbox.yaw.value, mailbox.yaw.timestamp}} :
    std::nullopt;
  const YawDecision yaw{safety_.update_yaw(imu, wheel_delta_yaw, now)};
  if (yaw.source_changed) {
    log(DriverLogLevel::warning,
      yaw.imu_fallback ? "IMU stale; using wheel-derived yaw" :
      "IMU recovered; realigned yaw offset without a pose jump");
  }
  result.imu_fallback = yaw.imu_fallback;
  if (!yaw.imu_fallback) {
    result.measured_twist.omega_radps = mailbox.yaw.rate_radps;
  }
  result.pose = odometry_->update(yaw.yaw_rad, positions);
  return result;
}

bool ControlLoop::Impl::execute_cycle(SteadyClock::time_point now)
{
  const CyclePlan plan{prepare_cycle(now)};
  const auto route = exchange_cycle_frames(make_cycle_frames(plan), now);
  const CycleOdometry odometry{update_cycle_odometry(plan.mailbox, route.received, now)};
  process_recovery(now);
  const bool bus_silent{safety_.all_bus_silent(motor_health())};
  update_cycle_status(route, odometry, plan.command.timed_out, bus_silent);
  maybe_publish(now, odometry, plan.discrete_command, plan.alignment.gated);
  return true;
}

void ControlLoop::Impl::process_recovery(SteadyClock::time_point now)
{
  if (clear_faults_requested_.exchange(false)) {
    const auto enable_confirmed = dispatch_recovery_actions(
      safety_.manual_clear_actions(), now);
    if (!safety_.complete_manual_clear(motor_health(), enable_confirmed)) {
      log(DriverLogLevel::error,
        "manual fault clear did not re-enable every motor; latch remains active");
    } else {
      log(DriverLogLevel::info, "manual fault clear verified all motors enabled");
    }
    return;
  }
  static_cast<void>(dispatch_recovery_actions(
      safety_.recovery_actions(motor_health(), now), now));
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
  const CycleOdometry & odometry,
  bool command_timed_out,
  bool bus_silent)
{
  const auto motors = motor_pointers();
  std::lock_guard<std::mutex> lock{status_mutex_};
  ++status_.completed_cycles;
  status_.unknown_frames += route.unknown_frames;
  status_.rejected_frames += route.rejected_frames;
  status_.stale_frames += route.stale_frames;
  status_.unknown_frames_last_cycle = route.unknown_frames;
  status_.rejected_frames_last_cycle = route.rejected_frames;
  status_.stale_frames_last_cycle = route.stale_frames;
  status_.command_timed_out = command_timed_out;
  status_.imu_fallback = odometry.imu_fallback;
  status_.bus_silent = bus_silent;
  status_.faulted = safety_.faulted();
  status_.fault_latched = safety_.fault_latched();
  status_.transport_faulted = safety_.transport_faulted();
  status_.steering_limit_faulted = safety_.steering_limit_faulted();
  status_.recovery_attempts = safety_.recovery_attempts();
  status_.pose = odometry.pose;
  status_.slipping_modules = odometry.slipping_modules;
  status_.slip_detected = std::any_of(
    odometry.slipping_modules.begin(), odometry.slipping_modules.end(),
    [](bool slipping) {return slipping;});
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    status_.motors[index] = motors[index]->health();
    status_.motor_limits[index] = motors[index]->limits();
  }
}
}  // namespace dm_swerve_driver
