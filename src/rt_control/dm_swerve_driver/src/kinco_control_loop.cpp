#include "kinco_control_impl.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <pthread.h>
#include <sched.h>

namespace dm_swerve_driver {
namespace {
SteeringAngleLimits angle_limits(const DriverParameters & p)
{
  return {p.steering.joint_limit_min_rad, p.steering.joint_limit_max_rad,
    p.steering.joint_limit_margin_rad, p.steering.joint_limit_tolerance_rad};
}
}

KincoControlLoop::Impl::Impl(DriverParameters common, KincoParameters parameters,
  std::unique_ptr<KincoEthercatBus> bus, std::unique_ptr<CanTransport> encoders,
  ControlLoopCallbacks callbacks)
: common_{std::move(common)}, parameters_{std::move(parameters)},
  hardware_{kinco_hardware_config(common_, parameters_), std::move(bus)},
  encoders_{encoder_canopen_config(parameters_), std::move(encoders)},
  store_{parameters_.encoder_snapshot_path}, callbacks_{std::move(callbacks)},
  safety_{common_}, planner_{SwerveSetpointParameters{common_.chassis.align_threshold_rad,
      common_.steering.flip_hysteresis_rad, common_.steering.max_slew_radps, angle_limits(common_)}}
{
  const double cycle_ns{1e9 / common_.control.rate_hz};
  if (std::abs(cycle_ns - static_cast<double>(parameters_.dc_cycle_ns)) > 1.0) {
    throw std::invalid_argument{"control.rate_hz must match kinco.ethercat.dc_cycle_ns"};
  }
  for (std::size_t i{0U}; i < selectors_.size(); ++i) {
    encoder_configs_[i] = external_encoder_config(parameters_, i);
    selectors_[i].emplace(parameters_.encoder_source_disagreement_threshold_rad,
      parameters_.encoder_maximum_rejoin_correction_rad);
  }
  controls_.fill(0x000FU);
  status_.kinco_backend = true;
}

bool KincoControlLoop::Impl::initialize(KincoClock::time_point now)
{
  std::lock_guard<std::mutex> guard{io_mutex_};
  if (initialized_) {return true;}
  try {
    const auto snapshot = store_.load();
    if (!snapshot) {throw std::runtime_error{"encoder snapshot missing; calibrate steering while disabled"};}
    encoders_.open();
    encoder_info_ = encoders_.read_hardware_info();
    encoders_.configure_sync();
    if (!hardware_.prepare()) {throw std::runtime_error{"Kinco disabled startup sampling failed"};}
    feedback_ = hardware_.feedback();
    encoder_cycle_ = encoders_.sample(KincoClock::now() +
      std::chrono::microseconds{parameters_.encoder_feedback_deadline_us});
    if (!encoder_cycle_.complete()) {throw std::runtime_error{"all four startup encoders are required"};}
    for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
      const auto result = validate_external_encoder_startup(external_encoder_config(parameters_, i),
        encoder_info_[i], encoder_cycle_.positions[i], (*snapshot)[i],
        feedback_.modules[i].motor_steering_angle_rad, angle_limits(common_));
      if (!result.accepted()) {
        throw std::runtime_error{"encoder startup rejected module " + std::to_string(i) +
                ", reason " + std::to_string(static_cast<int>(result.failure))};
      }
    }
    if (!hardware_.enable()) {throw std::runtime_error{"Kinco enable confirmation failed"};}
    feedback_ = hardware_.feedback();
    update_sources(now);
    if (source_fault_) {throw std::runtime_error{"steering source invalid after enable"};}
    for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
      previous_positions_[i] = {feedback_.modules[i].wheel_distance_m, sources_[i].angle_rad, true};
    }
    const auto yaw = safety_.update_yaw(std::nullopt, 0.0, now);
    odometry_.emplace(module_locations(common_), yaw.yaw_rad, previous_positions_);
    initialized_ = true;
    last_step_ = now;
    last_publish_ = now - std::chrono::seconds{1};
    update_health(now);
    refresh_status();
    return true;
  } catch (const std::exception & error) {
    log(DriverLogLevel::error, error.what());
  } catch (...) {log(DriverLogLevel::error, "unknown Kinco initialization failure");}
  hardware_.stop();
  encoders_.close();
  refresh_status();
  return false;
}

void KincoControlLoop::Impl::update_sources(KincoClock::time_point)
{
  source_fault_ = false;
  const auto limits = angle_limits(common_);
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    const auto & config = encoder_configs_[i];
    const auto total = static_cast<std::uint64_t>(config.expected_counts_per_revolution) *
      config.expected_distinguishable_revolutions;
    const bool external_fresh = encoder_cycle_.received[i] && encoder_cycle_.positions[i] < total;
    const double external = external_fresh ?
      external_encoder_position_to_axis_angle(encoder_cycle_.positions[i], config) : 0.0;
    const bool motor_fresh = feedback_.raw.domain.healthy() && feedback_.raw.feedback[i].online &&
      feedback_.axis_faults[i].disposition == FaultDisposition::none;
    sources_[i] = selectors_[i]->select({external, external_fresh},
      {feedback_.modules[i].motor_steering_angle_rad, motor_fresh});
    // Check raw absolute sources as well as the continuity-adjusted estimate.
    const bool outside = (external_fresh && !steering_measurement_within_tolerance(external, limits)) ||
      (motor_fresh && !steering_measurement_within_tolerance(
        feedback_.modules[i].motor_steering_angle_rad, limits)) ||
      (encoder_cycle_.received[i] && !external_fresh);
    sources_[i].valid = sources_[i].valid && !outside &&
      steering_measurement_within_tolerance(sources_[i].angle_rad, limits);
    if (!sources_[i].valid) {sources_[i].degraded = true;}
    source_fault_ |= !sources_[i].valid;
  }
  static_cast<void>(safety_.observe_steering_limit_violation(source_fault_));
}

bool KincoControlLoop::Impl::step(KincoClock::time_point now)
{
  std::lock_guard<std::mutex> guard{io_mutex_};
  if (!initialized_) {return false;}
  try {
    ChassisSpeeds command;
    std::optional<KincoClock::time_point> command_time;
    std::optional<TimedYawSample> imu;
    double yaw_rate{0.0};
    {
      std::lock_guard<std::mutex> lock{mailbox_mutex_};
      command = command_;
      command_time = command_time_;
      imu = imu_;
      yaw_rate = yaw_rate_;
    }
    const auto decision = safety_.command_for_cycle(command, command_time, now);
    const auto targets = plan(now, decision.command, decision.timed_out);
    feedback_ = hardware_.exchange(targets, controls_);
    try {
      encoder_cycle_ = encoders_.sample(KincoClock::now() +
        std::chrono::microseconds{parameters_.encoder_feedback_deadline_us});
    } catch (...) {
      encoder_cycle_ = {};  // A healthy motor source remains usable on encoder CAN loss.
    }
    update_sources(now);
    update_health(now);
    output_ = update_odometry(now, imu, yaw_rate);
    output_.command = decision.command;
    output_.alignment_gated = std::all_of(targets.begin(), targets.end(),
      [](const auto & t) {return !t.drive_enabled;});
    refresh_status();
    {
      std::lock_guard<std::mutex> lock{status_mutex_};
      status_.command_timed_out = decision.timed_out;
      ++status_.completed_cycles;
    }
    if (now - last_publish_ >= std::chrono::duration<double>{1.0 / common_.odometry.publish_rate_hz}) {
      last_publish_ = now;
      try {if (callbacks_.publish_output) {callbacks_.publish_output(output_);}}
      catch (...) {log(DriverLogLevel::warning, "Kinco output callback failed");}
    }
    return true;
  } catch (const std::exception & error) {
    safety_.mark_transport_failure();
    log(DriverLogLevel::error, error.what());
  } catch (...) {
    safety_.mark_transport_failure();
    log(DriverLogLevel::error, "unknown Kinco cycle failure");
  }
  refresh_status();
  return false;
}

void KincoControlLoop::Impl::start()
{
  if (!initialized_) {throw std::logic_error{"initialize Kinco loop before start"};}
  if (running_.exchange(true)) {return;}
  try {
    thread_ = std::thread{[this] {
      if (common_.control.realtime_priority > 0) {
        sched_param p{};
        p.sched_priority = common_.control.realtime_priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) != 0) {
          log(DriverLogLevel::warning, "Kinco loop could not apply SCHED_FIFO");
        }
      }
      auto next = KincoClock::now();
      const auto period = std::chrono::nanoseconds{parameters_.dc_cycle_ns};
      while (running_.load()) {
        next += period;
        static_cast<void>(step(KincoClock::now()));
        if (KincoClock::now() > next + period) {
          std::lock_guard<std::mutex> lock{status_mutex_};
          ++status_.loop_overruns;
          next = KincoClock::now();
        }
        std::this_thread::sleep_until(next);
      }
    }};
  } catch (...) {running_.store(false); throw;}
}

void KincoControlLoop::Impl::stop() noexcept
{
  running_.store(false);
  if (thread_.joinable()) {thread_.join();}
  std::lock_guard<std::mutex> guard{io_mutex_};
  hardware_.stop();
  if (initialized_ && !source_fault_ && encoders_.is_open()) {
    try {
      const auto last = encoders_.sample(KincoClock::now() +
        std::chrono::microseconds{parameters_.encoder_feedback_deadline_us});
      if (!last.complete()) {throw std::runtime_error{"encoder shutdown snapshot incomplete; previous retained"};}
      EncoderSnapshot snapshot{};
      for (std::size_t i{0U}; i < snapshot.size(); ++i) {snapshot[i] = {last.positions[i], encoder_info_[i]};}
      store_.save(snapshot);
    } catch (const std::exception & error) {log(DriverLogLevel::error, error.what());}
    catch (...) {log(DriverLogLevel::error, "encoder shutdown snapshot failed");}
  }
  encoders_.close();
  initialized_ = false;
  refresh_status();
}

void KincoControlLoop::Impl::log(DriverLogLevel level, const std::string & message) const noexcept
{
  try {if (callbacks_.log) {callbacks_.log(level, message);}} catch (...) {}
}

KincoControlLoop::KincoControlLoop(DriverParameters common, KincoParameters parameters,
  std::unique_ptr<KincoEthercatBus> bus, std::unique_ptr<CanTransport> encoders,
  ControlLoopCallbacks callbacks)
: impl_{std::make_unique<Impl>(std::move(common), std::move(parameters),
    std::move(bus), std::move(encoders), std::move(callbacks))} {}
KincoControlLoop::~KincoControlLoop() noexcept = default;
bool KincoControlLoop::initialize(KincoClock::time_point now) {return impl_->initialize(now);}
bool KincoControlLoop::step(KincoClock::time_point now) {return impl_->step(now);}
void KincoControlLoop::start() {impl_->start();}
void KincoControlLoop::stop() noexcept {impl_->stop();}
bool KincoControlLoop::is_running() const noexcept {return impl_->running_.load();}
void KincoControlLoop::request_clear_faults() noexcept {impl_->clear_requested_.store(true);}
void KincoControlLoop::restore_fault_state(bool latched, const std::array<std::uint32_t, kMotorCount> & tries)
{
  std::lock_guard<std::mutex> lock{impl_->io_mutex_};
  impl_->safety_.restore_recovery_state(tries, latched);
}
ControlLoopStatus KincoControlLoop::status() const
{
  std::lock_guard<std::mutex> lock{impl_->status_mutex_};
  return impl_->status_;
}
void KincoControlLoop::submit_command(const ChassisSpeeds & command, KincoClock::time_point now)
{
  if (!std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
    !std::isfinite(command.omega_radps)) {return;}
  std::lock_guard<std::mutex> lock{impl_->mailbox_mutex_};
  if (impl_->command_time_ && now < *impl_->command_time_) {return;}
  impl_->command_ = command;
  impl_->command_time_ = now;
}
bool KincoControlLoop::submit_imu_yaw(double yaw, KincoClock::time_point now, double rate)
{
  if (!std::isfinite(yaw) || !std::isfinite(rate)) {return false;}
  std::lock_guard<std::mutex> lock{impl_->mailbox_mutex_};
  if (impl_->imu_) {
    if (now < impl_->imu_->timestamp) {return false;}
    if (now - impl_->imu_->timestamp < std::chrono::duration<double>{impl_->common_.odometry.imu_timeout_s} &&
      std::abs(wrap_pi(yaw - impl_->imu_->yaw_rad)) > impl_->common_.odometry.max_imu_yaw_step_rad)
    {return false;}
  }
  impl_->imu_ = TimedYawSample{yaw, now};
  impl_->yaw_rate_ = rate;
  return true;
}

std::unique_ptr<CanTransport> make_encoder_transport(const KincoParameters & parameters)
{
  std::vector<std::uint16_t> ids;
  for (const auto node : parameters.encoder_node_ids) {
    for (const auto base : {0x280U, 0x580U, 0x700U}) {
      ids.push_back(static_cast<std::uint16_t>(base + node));
    }
  }
  return std::make_unique<SocketCanTransport>(SocketCanOptions{
      parameters.encoder_can_interface, ids, std::chrono::microseconds{2000}, false});
}
}  // namespace dm_swerve_driver
