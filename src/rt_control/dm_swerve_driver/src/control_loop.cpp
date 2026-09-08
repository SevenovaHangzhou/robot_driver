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
  safety_{parameters_},
  setpoint_generator_{SwerveSetpointParameters{
      parameters_.chassis.align_threshold_rad,
      parameters_.steering.flip_hysteresis_rad,
      parameters_.steering.max_slew_radps}}
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
  if (initialized_.load()) {
    return true;
  }
  try {
    transport_->open();
  } catch (const std::exception & error) {
    log(DriverLogLevel::error, std::string{"failed to open CAN transport: "} + error.what());
    transport_->close();
    return false;
  } catch (...) {
    log(DriverLogLevel::error, "failed to open CAN transport with an unknown error");
    transport_->close();
    return false;
  }
  try {
    if (!initialize_motors(parameters_, *transport_, motor_pointers(), logger())) {
      log(DriverLogLevel::error, "motor startup gate failed; refusing to enable driver");
      disable_motors(*transport_, motor_pointers(), logger());
      transport_->close();
      return false;
    }
  } catch (const std::exception & error) {
    log(DriverLogLevel::error,
      std::string{"motor startup failed; refusing to enable driver: "} + error.what());
    disable_motors(*transport_, motor_pointers(), logger());
    transport_->close();
    return false;
  } catch (...) {
    log(DriverLogLevel::error,
      "motor startup failed with an unknown error; refusing to enable driver");
    disable_motors(*transport_, motor_pointers(), logger());
    transport_->close();
    return false;
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
  initialized_.store(true);
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
  setpoint_generator_.reset();
  last_cycle_time_ = now;
  last_publish_time_ = now - publish_period();
}

bool ControlLoop::Impl::step(SteadyClock::time_point now)
{
  std::lock_guard<std::mutex> io_lock{io_mutex_};
  if (!initialized_.load()) {
    return false;
  }
  if (!transport_->is_open()) {
    safety_.mark_transport_failure();
    refresh_status();
    return false;
  }
  try {
    return execute_cycle(now);
  } catch (const std::exception & error) {
    safety_.mark_transport_failure();
    refresh_status();
    log(DriverLogLevel::error, std::string{"control cycle failed: "} + error.what());
  } catch (...) {
    safety_.mark_transport_failure();
    refresh_status();
    log(DriverLogLevel::error, "control cycle failed with an unknown error");
  }
  return false;
}

void ControlLoop::Impl::start()
{
  if (!initialized_.load()) {
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
  if (initialized_.load() && transport_->is_open()) {
    send_zero_cycles();
    disable_motors(*transport_, motor_pointers(), logger());
  }
  transport_->close();
  initialized_.store(false);
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
  bool rejected_timestamp{false};
  {
    std::lock_guard<std::mutex> lock{mailbox_mutex_};
    rejected_timestamp = command_mailbox_.valid && timestamp < command_mailbox_.timestamp;
    if (!rejected_timestamp) {
      command_mailbox_ = TimedCommand{command, timestamp, true};
    }
  }
  if (rejected_timestamp) {
    log(DriverLogLevel::warning, "ignored out-of-order cmd_vel timestamp");
  }
}

bool ControlLoop::Impl::submit_imu_yaw(
  double yaw_rad, SteadyClock::time_point timestamp, double yaw_rate_radps)
{
  if (!std::isfinite(yaw_rad) || !std::isfinite(yaw_rate_radps)) {
    log(DriverLogLevel::warning, "ignored non-finite IMU yaw");
    return false;
  }
  bool rejected_jump{false};
  bool rejected_timestamp{false};
  {
    std::lock_guard<std::mutex> lock{mailbox_mutex_};
    rejected_timestamp = yaw_mailbox_.valid && timestamp < yaw_mailbox_.timestamp;
    const bool temporally_adjacent = yaw_mailbox_.valid && !rejected_timestamp &&
      timestamp - yaw_mailbox_.timestamp <=
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>{parameters_.odometry.imu_timeout_s});
    rejected_jump = temporally_adjacent &&
      std::abs(wrap_pi(yaw_rad - yaw_mailbox_.value)) >
      parameters_.odometry.max_imu_yaw_step_rad;
    if (!rejected_jump && !rejected_timestamp) {
      yaw_mailbox_ = TimedYaw{yaw_rad, yaw_rate_radps, timestamp, true};
    }
  }
  if (rejected_timestamp) {
    log(DriverLogLevel::warning, "ignored out-of-order IMU timestamp");
    return false;
  }
  if (rejected_jump) {
    log(DriverLogLevel::warning, "ignored implausible IMU yaw jump");
    return false;
  }
  return true;
}

void ControlLoop::Impl::request_clear_faults() noexcept
{
  clear_faults_requested_.store(true);
}

void ControlLoop::Impl::restore_fault_state(
  bool fault_latched,
  const std::array<std::uint32_t, kMotorCount> & recovery_attempts)
{
  std::lock_guard<std::mutex> io_lock{io_mutex_};
  if (initialized_.load() || running_.load()) {
    throw std::logic_error{"fault state must be restored before control initialization"};
  }
  safety_.restore_recovery_state(recovery_attempts, fault_latched);
  refresh_status();
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

bool ControlLoop::submit_imu_yaw(
  double yaw_rad, SteadyClock::time_point timestamp, double yaw_rate_radps)
{
  return impl_->submit_imu_yaw(yaw_rad, timestamp, yaw_rate_radps);
}

void ControlLoop::request_clear_faults() noexcept
{
  impl_->request_clear_faults();
}

void ControlLoop::restore_fault_state(
  bool fault_latched,
  const std::array<std::uint32_t, kMotorCount> & recovery_attempts)
{
  impl_->restore_fault_state(fault_latched, recovery_attempts);
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
