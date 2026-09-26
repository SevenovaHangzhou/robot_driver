#include "swerve_driver/chassis_mode_handoff.hpp"

#include <algorithm>
#include <cmath>

namespace swerve_driver
{
namespace
{
bool positive(double x) noexcept {return std::isfinite(x) && x > 0;}
bool matches(uint16_t word, HandoffStatusPredicate p) noexcept
{return (word & p.mask) == p.value;}
bool valid(HandoffStatusPredicate p) noexcept
{return p.mask != 0 && (p.value & p.mask) == p.value;}
}
bool ChassisModeHandoff::configure(const ChassisHandoffConfig & c, uint64_t generation) noexcept
{
  if (configured_ || phase_ != HandoffPhase::inactive || generation == 0 ||
    generation == std::numeric_limits<uint64_t>::max()) {return false;}
  const double required[] = {c.stationary_wheel_velocity, c.stationary_steering_velocity,
    c.stationary_dwell, c.feedback_timeout, c.command_timeout, c.max_update_period,
    c.stop_timeout, c.switch_timeout};
  for (double v : required) {
    if (!positive(v)) {
      return false;
    }
  }
  if (!valid(c.enabled) || !valid(c.csv_ready) || !valid(c.csp_ready) ||
    c.stationary_dwell >= c.stop_timeout || c.stationary_dwell >= c.switch_timeout ||
    c.max_update_period >= c.stop_timeout || c.max_update_period >= c.switch_timeout ||
    ((c.csp_ready.mask & 0x1000U) != 0 && (c.csp_ready.value & 0x1000U) == 0))
  {return false;}
  for (std::size_t i = 0; i < 4; ++i) {
    if (!positive(c.max_wheel_velocity[i]) || !positive(c.max_wheel_acceleration[i]) ||
      !positive(c.position_tolerance[i]) ||
      !std::isfinite(c.max_wheel_velocity[i] * 2) ||
      !positive(c.max_wheel_acceleration[i] * c.max_update_period) ||
      c.max_wheel_velocity[i] / c.max_wheel_acceleration[i] + c.stationary_dwell +
      2 * c.max_update_period >= c.stop_timeout) {return false;}
  }
  config_ = c; generation_ = generation; configured_ = true;
  return true;
}
HandoffCause ChassisModeHandoff::check_feedback(
  double now,
  const HandoffFeedback & f) const noexcept
{
  for (const auto & d : f) {
    if (!std::isfinite(d.sample_time) || d.sample_time < 0 || d.sample_time > now ||
      now - d.sample_time > config_.feedback_timeout || d.drive_sequence == 0 ||
      !std::isfinite(d.wheel_position) || !std::isfinite(d.wheel_velocity) ||
      !std::isfinite(d.steering_velocity)) {return HandoffCause::feedback;}
    if (!d.healthy || !matches(d.status_word, config_.enabled)) {return HandoffCause::drive;}
    if (d.mode != 8 && d.mode != 9) {return HandoffCause::mode;}
  }
  return HandoffCause::none;
}
bool ChassisModeHandoff::request_fresh(double now) const noexcept
{
  return configured_ && std::isfinite(now) && now >= last_update_ &&
         now - last_update_ <= config_.max_update_period &&
         check_feedback(now, feedback_) == HandoffCause::none;
}
bool ChassisModeHandoff::all_mode(int8_t mode) const noexcept
{
  for (const auto & d : feedback_) {
    if (d.mode != mode) {
      return false;
    }
  }
  return true;
}
bool ChassisModeHandoff::stationary_sample() const noexcept
{
  for (const auto & d : feedback_) {
    if (std::abs(d.wheel_velocity) > config_.stationary_wheel_velocity ||
      std::abs(d.steering_velocity) > config_.stationary_steering_velocity) {return false;}
  }
  return true;
}
bool ChassisModeHandoff::stationary() const noexcept
{
  if (stationary_since_ < 0) {return false;}
  for (const auto & d : feedback_) {
    if (d.sample_time - stationary_since_ < config_.stationary_dwell) {return false;}
  }
  return true;
}
bool ChassisModeHandoff::seed_stable() const noexcept
{
  for (std::size_t i = 0; i < 4; ++i) {
    const double error = feedback_[i].wheel_position - output_.wheel_position[i];
    if (!std::isfinite(error) || std::abs(error) > config_.position_tolerance[i]) {return false;}
  }
  return true;
}
bool ChassisModeHandoff::acknowledged(
  const HandoffAcknowledgements & a, bool newer_feedback) const noexcept
{
  for (std::size_t i = 0; i < 4; ++i) {
    if (a[i].write_sequence != output_.write_sequence ||
      (newer_feedback && feedback_[i].drive_sequence <= a[i].feedback_sequence_at_send))
    {return false;}
  }
  return true;
}
void ChassisModeHandoff::inhibit(HandoffCause cause) noexcept
{
  if (cause_ == HandoffCause::none) {cause_ = cause;}
  phase_ = HandoffPhase::inhibited;
  output_.inhibited = true;
  output_.write_velocity = output_.write_position = output_.write_mode = false;
  have_command_ = false;
}
bool ChassisModeHandoff::bump_generation() noexcept
{
  have_command_ = false;
  if (generation_ == std::numeric_limits<uint64_t>::max()) {
    inhibit(HandoffCause::sequence_exhausted); return false;
  }
  ++generation_; return true;
}
bool ChassisModeHandoff::next_write() noexcept
{
  if (output_.write_sequence == std::numeric_limits<uint64_t>::max()) {
    inhibit(HandoffCause::sequence_exhausted); return false;
  }
  ++output_.write_sequence; return true;
}
bool ChassisModeHandoff::start_navigation(
  double now, const HandoffFeedback & f,
  const HandoffWheels & last_sent_velocity) noexcept
{
  if (!configured_ || phase_ != HandoffPhase::inactive || !std::isfinite(now) || now < 0 ||
    check_feedback(now, f) != HandoffCause::none) {return false;}
  for (std::size_t i = 0; i < 4; ++i) {
    if (last_sent_velocity[i] != 0 || f[i].mode != 9 ||
      std::abs(f[i].wheel_velocity) > config_.stationary_wheel_velocity ||
      std::abs(f[i].steering_velocity) > config_.stationary_steering_velocity) {return false;}
  }
  feedback_ = f; last_update_ = phase_started_ = stationary_since_ = now;
  phase_ = HandoffPhase::seed_csv; source_mode_ = 9;
  output_.inhibited = false; output_.write_velocity = true; output_.requested_mode = 9;
  return next_write();
}
bool ChassisModeHandoff::accept_navigation(
  const HandoffNavigationCommand & command, double now) noexcept
{
  if (!navigation_open() || !request_fresh(now) || command.generation != generation_ ||
    !std::isfinite(command.received_at) || command.received_at <= admission_since_ ||
    command.received_at > now || now - command.received_at > config_.command_timeout ||
    (have_command_ && command.received_at <= command_.received_at)) {return false;}
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(command.wheel_velocity[i]) ||
      std::abs(command.wheel_velocity[i]) > config_.max_wheel_velocity[i]) {return false;}
  }
  command_ = command; have_command_ = true; return true;
}
bool ChassisModeHandoff::refine_navigation(
  const HandoffNavigationCommand & command,
  double now) noexcept
{
  if (!have_command_ || command.received_at != command_.received_at ||
    !navigation_open() || !request_fresh(now) || command.generation != generation_ ||
    now - command.received_at > config_.command_timeout) {return false;}
  for (size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(command.wheel_velocity[i]) ||
      std::abs(command.wheel_velocity[i]) > config_.max_wheel_velocity[i]) {return false;}
  }
  command_.wheel_velocity = command.wheel_velocity; return true;
}
bool ChassisModeHandoff::reconfirm_navigation(double now) noexcept
{
  if (!navigation_open() || !request_fresh(now) || !stationary()) {return false;}
  for (double v : output_.wheel_velocity) {
    if (v != 0) {
      return false;
    }
  }
  reopen_navigation(now); return navigation_open();
}
bool ChassisModeHandoff::begin_operation(double now) noexcept
{
  if (!navigation_open() || !request_fresh(now)) {return false;}
  if (!bump_generation()) {return false;}
  phase_ = HandoffPhase::stopping_csv; phase_started_ = now; stationary_since_ = -1;
  // Retain the last emitted reference; update() decelerates it, never reseeds it to measured speed.
  return true;
}
bool ChassisModeHandoff::begin_navigation(double now, const HandoffWheels & held) noexcept
{
  if (!operation_ready() || !request_fresh(now) || !stationary()) {return false;}
  for (std::size_t i = 0; i < 4; ++i) {
    const double error = held[i] - feedback_[i].wheel_position;
    if (!std::isfinite(held[i]) || !std::isfinite(error) ||
      std::abs(error) > config_.position_tolerance[i]) {return false;}
  }
  if (!bump_generation()) {return false;}
  phase_ = HandoffPhase::seed_csv; phase_started_ = now; source_mode_ = 8;
  output_.wheel_position = held; output_.wheel_velocity.fill(0);
  output_.write_position = output_.write_velocity = true; output_.write_mode = false;
  output_.requested_mode = 8;
  return next_write();
}
void ChassisModeHandoff::reopen_navigation(double now) noexcept
{
  if (!bump_generation()) {return;}
  admission_since_ = now; phase_ = HandoffPhase::navigation;
  output_.write_position = output_.write_mode = false;
  output_.write_velocity = true; output_.wheel_velocity.fill(0);
}
void ChassisModeHandoff::slew_velocity(double dt, const HandoffWheels & desired) noexcept
{
  for (std::size_t i = 0; i < 4; ++i) {
    const double current = output_.wheel_velocity[i];
    const double step = config_.max_wheel_acceleration[i] * dt;
    const double delta = desired[i] - current;
    output_.wheel_velocity[i] = std::abs(delta) <= step ? desired[i] :
      current + std::copysign(step, delta);
  }
}
void ChassisModeHandoff::update(
  double now, const HandoffFeedback & f,
  const HandoffAcknowledgements & a) noexcept
{
  if (phase_ == HandoffPhase::inactive || phase_ == HandoffPhase::inhibited) {return;}
  if (!std::isfinite(now) || now <= last_update_ ||
    now - last_update_ > config_.max_update_period) {inhibit(HandoffCause::timing); return;}
  const double dt = now - last_update_; last_update_ = now;
  const auto fault = check_feedback(now, f);
  if (fault != HandoffCause::none) {inhibit(fault); return;}
  for (std::size_t i = 0; i < 4; ++i) {
    if (f[i].drive_sequence < feedback_[i].drive_sequence ||
      f[i].sample_time < feedback_[i].sample_time) {inhibit(HandoffCause::feedback); return;}
    if (a[i].write_sequence > output_.write_sequence ||
      a[i].feedback_sequence_at_send > f[i].drive_sequence ||
      (a[i].write_sequence != 0 && a[i].feedback_sequence_at_send == 0))
    {inhibit(HandoffCause::acknowledgement); return;}
  }
  feedback_ = f;
  if (!stationary_sample()) {stationary_since_ = -1;} else if (stationary_since_ < 0) {
    stationary_since_ = now;
  }

  if (phase_ == HandoffPhase::navigation || phase_ == HandoffPhase::stopping_csv) {
    if (!all_mode(9)) {inhibit(HandoffCause::mode); return;}
    for (const auto & d : f) {
      if (!matches(d.status_word, config_.csv_ready)) {inhibit(HandoffCause::drive); return;}
    }
    if (phase_ == HandoffPhase::stopping_csv && now - phase_started_ >= config_.stop_timeout) {
      inhibit(HandoffCause::stop_timeout); return;
    }
    if (have_command_ && now - command_.received_at > config_.command_timeout) {
      have_command_ = false;
    }
    slew_velocity(dt, have_command_ ? command_.wheel_velocity : HandoffWheels{});
    if (!next_write()) {return;}
    if (phase_ == HandoffPhase::navigation) {return;}
    for (double v : output_.wheel_velocity) {
      if (v != 0) {
        return;
      }
    }
    if (!stationary()) {return;}
    for (std::size_t i = 0; i < 4; ++i) {
      output_.wheel_position[i] = f[i].wheel_position;
    }
    phase_ = HandoffPhase::seed_csp; phase_started_ = now;
    output_.write_position = true; output_.write_mode = false;
    return;
  }
  if (phase_ == HandoffPhase::operation) {
    for (const auto & d : f) {
      if (d.mode != 8) {inhibit(HandoffCause::mode); return;}
      if (!matches(d.status_word, config_.csp_ready) || (d.status_word & 0x1000U) == 0) {
        inhibit(HandoffCause::drive); return;
      }
    }
    return;
  }
  if (now - phase_started_ >= config_.switch_timeout) {
    inhibit(HandoffCause::switch_timeout); return;
  }
  if (!stationary_sample() || (output_.write_position && !seed_stable())) {
    inhibit(HandoffCause::seed_moved); return;
  }
  const bool csp = phase_ == HandoffPhase::seed_csp || phase_ == HandoffPhase::confirm_csp;
  const bool seeding = phase_ == HandoffPhase::seed_csp || phase_ == HandoffPhase::seed_csv;
  if (seeding) {
    if (!all_mode(csp ? 9 : source_mode_)) {inhibit(HandoffCause::mode); return;}
    if (!stationary() || !acknowledged(a, false)) {return;}
    output_.requested_mode = csp ? 8 : 9; output_.write_mode = true;
    phase_ = csp ? HandoffPhase::confirm_csp : HandoffPhase::confirm_csv;
    (void)next_write(); return;
  }
  if (!acknowledged(a, true) || !all_mode(csp ? 8 : 9)) {return;}
  for (const auto & d : f) {
    if (!d.mode_ack || !matches(d.status_word, csp ? config_.csp_ready : config_.csv_ready) ||
      (csp && (d.status_word & 0x1000U) == 0)) {return;}
  }
  if (csp) {
    phase_ = HandoffPhase::operation;
    output_.write_position = output_.write_velocity = output_.write_mode = false;
  } else {reopen_navigation(now);}
}
void ChassisModeHandoff::cancel() noexcept {inhibit(HandoffCause::canceled);}
void ChassisModeHandoff::deactivate() noexcept {inhibit(HandoffCause::deactivated);}
}  // namespace swerve_driver
