#include "control_loop_impl.hpp"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] std::array<SwerveModule, kSwerveModuleCount> make_modules(
  const DriverParameters & parameters)
{
  return {
    SwerveModule{
      steering_motor_config(parameters, 0U), drive_motor_config(parameters, 0U),
      steering_module_config(parameters, 0U), drive_module_config(parameters, 0U)},
    SwerveModule{
      steering_motor_config(parameters, 1U), drive_motor_config(parameters, 1U),
      steering_module_config(parameters, 1U), drive_module_config(parameters, 1U)},
    SwerveModule{
      steering_motor_config(parameters, 2U), drive_motor_config(parameters, 2U),
      steering_module_config(parameters, 2U), drive_module_config(parameters, 2U)},
    SwerveModule{
      steering_motor_config(parameters, 3U), drive_motor_config(parameters, 3U),
      steering_module_config(parameters, 3U), drive_module_config(parameters, 3U)}};
}

}  // namespace

ControlLoop::Impl::Impl(
  DriverParameters parameters,
  std::unique_ptr<CanTransport> transport,
  ControlLoopCallbacks callbacks)
: parameters_{std::move(parameters)},
  transport_{std::move(transport)},
  callbacks_{std::move(callbacks)},
  modules_{make_modules(parameters_)},
  safety_{parameters_}
{
  validate_parameters(parameters_);
  if (transport_ == nullptr) {
    throw std::invalid_argument{"ControlLoop requires a CAN transport"};
  }
}

ControlLoop::Impl::~Impl() noexcept
{
  stop();
}

bool ControlLoop::Impl::initialize(SteadyClock::time_point now)
{
  std::lock_guard<std::mutex> io_lock{io_mutex_};
  if (initialized_) {
    return true;
  }
  try {
    transport_->open();
  } catch (const std::exception & error) {
    log(DriverLogLevel::error, std::string{"failed to open CAN transport: "} + error.what());
    transport_->close();
    return false;
  }
  try {
    initialize_motors(parameters_, *transport_, motor_pointers(), logger());
  } catch (const std::exception & error) {
    log(DriverLogLevel::warning,
      std::string{"motor startup incomplete; continuing degraded: "} + error.what());
  }
  try {
    initialize_odometry(now);
  } catch (const std::exception & error) {
    log(DriverLogLevel::error,
      std::string{"odometry initialization failed: "} + error.what());
    disable_motors(*transport_, motor_pointers(), {});
    transport_->close();
    return false;
  } catch (...) {
    log(DriverLogLevel::error, "odometry initialization failed with an unknown error");
    disable_motors(*transport_, motor_pointers(), {});
    transport_->close();
    return false;
  }
  initialized_ = true;
  refresh_status();
  return true;
}

void ControlLoop::Impl::initialize_odometry(SteadyClock::time_point now)
{
  const MailboxSnapshot mailbox{mailbox_snapshot()};
  const auto imu = mailbox.yaw.valid ?
    std::optional<TimedYawSample>{TimedYawSample{mailbox.yaw.value, mailbox.yaw.timestamp}} :
    std::nullopt;
  const YawDecision initial_yaw{safety_.update_yaw(imu, 0.0, now)};
  const auto initial_positions = current_module_positions(std::nullopt);
  odometry_.emplace(
    module_locations(parameters_), initial_yaw.yaw_rad, initial_positions);
  previous_positions_ = initial_positions;
  last_publish_time_ = now - publish_period();
}

bool ControlLoop::Impl::step(SteadyClock::time_point now)
{
  std::lock_guard<std::mutex> io_lock{io_mutex_};
  if (!initialized_ || !transport_->is_open()) {
    return false;
  }
  try {
    return execute_cycle(now);
  } catch (const std::exception & error) {
    log(DriverLogLevel::error, std::string{"control cycle failed: "} + error.what());
  } catch (...) {
    log(DriverLogLevel::error, "control cycle failed with an unknown error");
  }
  return false;
}

void ControlLoop::Impl::start()
{
  if (!initialized_) {
    throw std::logic_error{"ControlLoop must be initialized before start"};
  }
  bool expected{false};
  if (!running_.compare_exchange_strong(expected, true)) {
    return;
  }
  thread_ = std::thread{[this] {run();}};
}

void ControlLoop::Impl::stop() noexcept
{
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
  std::lock_guard<std::mutex> io_lock{io_mutex_};
  if (initialized_ && transport_->is_open()) {
    send_zero_cycles();
    disable_motors(*transport_, motor_pointers(), logger());
  }
  transport_->close();
  initialized_ = false;
  refresh_status();
}

void ControlLoop::Impl::submit_command(
  const ChassisSpeeds & command, SteadyClock::time_point timestamp)
{
  if (!std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
    !std::isfinite(command.omega_radps))
  {
    log(DriverLogLevel::warning, "ignored non-finite cmd_vel command");
    return;
  }
  std::lock_guard<std::mutex> lock{mailbox_mutex_};
  command_mailbox_ = TimedCommand{command, timestamp, true};
}

void ControlLoop::Impl::submit_imu_yaw(double yaw_rad, SteadyClock::time_point timestamp)
{
  if (!std::isfinite(yaw_rad)) {
    log(DriverLogLevel::warning, "ignored non-finite IMU yaw");
    return;
  }
  std::lock_guard<std::mutex> lock{mailbox_mutex_};
  yaw_mailbox_ = TimedYaw{yaw_rad, timestamp, true};
}

void ControlLoop::Impl::request_clear_faults() noexcept
{
  clear_faults_requested_.store(true);
}

bool ControlLoop::Impl::is_running() const noexcept
{
  return running_.load();
}

ControlLoopStatus ControlLoop::Impl::status() const
{
  std::lock_guard<std::mutex> lock{status_mutex_};
  ControlLoopStatus copy{status_};
  copy.running = running_.load();
  return copy;
}

ControlLoop::ControlLoop(
  DriverParameters parameters,
  std::unique_ptr<CanTransport> transport,
  ControlLoopCallbacks callbacks)
: impl_{std::make_unique<Impl>(
      std::move(parameters), std::move(transport), std::move(callbacks))}
{
}

ControlLoop::~ControlLoop() noexcept = default;

bool ControlLoop::initialize(SteadyClock::time_point now)
{
  return impl_->initialize(now);
}

bool ControlLoop::step(SteadyClock::time_point now)
{
  return impl_->step(now);
}

void ControlLoop::start()
{
  impl_->start();
}

void ControlLoop::stop() noexcept
{
  impl_->stop();
}

void ControlLoop::submit_command(
  const ChassisSpeeds & command, SteadyClock::time_point timestamp)
{
  impl_->submit_command(command, timestamp);
}

void ControlLoop::submit_imu_yaw(double yaw_rad, SteadyClock::time_point timestamp)
{
  impl_->submit_imu_yaw(yaw_rad, timestamp);
}

void ControlLoop::request_clear_faults() noexcept
{
  impl_->request_clear_faults();
}

bool ControlLoop::is_running() const noexcept
{
  return impl_->is_running();
}

ControlLoopStatus ControlLoop::status() const
{
  return impl_->status();
}

}  // namespace dm_swerve_driver
