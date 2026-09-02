#include "control_loop_impl.hpp"

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <chrono>
#include <exception>
#include <string>
#include <vector>

namespace dm_swerve_driver {
namespace {

void add_period(timespec & value, std::chrono::nanoseconds period) noexcept
{
  constexpr long nanoseconds_per_second{1000000000L};
  value.tv_nsec += static_cast<long>(period.count());
  while (value.tv_nsec >= nanoseconds_per_second) {
    value.tv_nsec -= nanoseconds_per_second;
    ++value.tv_sec;
  }
}

[[nodiscard]] bool later_than(const timespec & left, const timespec & right) noexcept
{
  return left.tv_sec > right.tv_sec ||
         (left.tv_sec == right.tv_sec && left.tv_nsec > right.tv_nsec);
}

}  // namespace

ControlLoopOutput ControlLoop::Impl::make_output(
  SteadyClock::time_point now,
  const Pose2d & pose,
  const ChassisSpeeds & command,
  bool alignment_gated) const
{
  ControlLoopOutput output{};
  output.timestamp = now;
  output.pose = pose;
  output.command = command;
  output.alignment_gated = alignment_gated;
  for (std::size_t index{0U}; index < modules_.size(); ++index) {
    if (modules_[index].steering_motor().position_initialized()) {
      output.steering_angle_rad[index] = modules_[index].steering_angle_rad();
    }
    if (modules_[index].drive_motor().position_initialized()) {
      output.wheel_distance_m[index] = modules_[index].wheel_distance_m();
      output.wheel_velocity_mps[index] = modules_[index].wheel_velocity_mps();
    }
  }
  return output;
}

void ControlLoop::Impl::maybe_publish(
  SteadyClock::time_point now,
  const Pose2d & pose,
  const ChassisSpeeds & command,
  bool alignment_gated)
{
  if (now - last_publish_time_ < publish_period()) {
    return;
  }
  last_publish_time_ = now;
  const ControlLoopOutput output{make_output(now, pose, command, alignment_gated)};
  try {
    if (callbacks_.publish_output) {
      callbacks_.publish_output(output);
    }
  } catch (const std::exception & error) {
    log(DriverLogLevel::warning, std::string{"output callback failed: "} + error.what());
  } catch (...) {
    log(DriverLogLevel::warning, "output callback failed with an unknown error");
  }
}

void ControlLoop::Impl::refresh_status()
{
  const auto motors = motor_pointers();
  std::lock_guard<std::mutex> lock{status_mutex_};
  status_.initialized = initialized_;
  status_.running = running_.load();
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    status_.motors[index] = motors[index]->health();
    status_.motor_limits[index] = motors[index]->limits();
  }
  if (odometry_.has_value()) {
    status_.pose = odometry_->pose();
  }
}

std::vector<CanFrame> ControlLoop::Impl::make_zero_frames(double dt)
{
  std::vector<CanFrame> frames;
  frames.reserve(kMotorCount);
  for (auto & module : modules_) {
    const double angle = module.steering_motor().position_initialized() ?
      module.steering_angle_rad() : 0.0;
    const auto command = module.make_command(
      OptimizedModuleState{0.0, angle, 0.0}, dt, true);
    const auto pair = module.encode_command_frames(command);
    frames.push_back(pair[0]);
    frames.push_back(pair[1]);
  }
  return frames;
}

void ControlLoop::Impl::send_zero_cycles() noexcept
{
  try {
    constexpr int zero_cycles{2};
    const double dt{1.0 / parameters_.control.rate_hz};
    for (int cycle{0}; cycle < zero_cycles; ++cycle) {
      transport_->write_batch(make_zero_frames(dt));
      const auto replies = transport_->collect(
        kMotorCount,
        SteadyClock::now() + std::chrono::microseconds{
          parameters_.can.feedback_deadline_us});
      static_cast<void>(route_feedback_frames(replies, motor_pointers(), {}));
    }
  } catch (const std::exception & error) {
    log(DriverLogLevel::warning, std::string{"zero-speed shutdown failed: "} + error.what());
  } catch (...) {
    log(DriverLogLevel::warning, "zero-speed shutdown failed with an unknown error");
  }
}

void ControlLoop::Impl::configure_realtime_priority() noexcept
{
  if (parameters_.control.realtime_priority == 0) {
    return;
  }
  sched_param scheduling{};
  scheduling.sched_priority = parameters_.control.realtime_priority;
  if (::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &scheduling) != 0) {
    log(DriverLogLevel::warning,
      "failed to apply SCHED_FIFO priority; continuing without realtime scheduling");
  }
}

void ControlLoop::Impl::run() noexcept
{
  configure_realtime_priority();
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>{1.0 / parameters_.control.rate_hz});
  timespec next{};
  static_cast<void>(::clock_gettime(CLOCK_MONOTONIC, &next));
  while (running_.load()) {
    add_period(next, period);
    static_cast<void>(step(SteadyClock::now()));
    timespec current{};
    static_cast<void>(::clock_gettime(CLOCK_MONOTONIC, &current));
    if (later_than(current, next)) {
      std::lock_guard<std::mutex> lock{status_mutex_};
      ++status_.loop_overruns;
      next = current;
    }
    static_cast<void>(::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr));
  }
}

}  // namespace dm_swerve_driver
