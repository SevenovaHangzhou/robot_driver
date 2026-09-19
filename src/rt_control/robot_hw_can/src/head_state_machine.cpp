#include "robot_hw_can/head_state_machine.hpp"

#include <algorithm>
#include <stdexcept>

namespace robot_hw_can
{
namespace
{

[[nodiscard]] bool any_fault(const std::array<MotorStatus, 2U> & motors) noexcept
{
  return std::any_of(motors.begin(), motors.end(), [](const MotorStatus & motor) {
      return motor.valid && motor.status > 1U;
    });
}

[[nodiscard]] bool all_status(
  const std::array<MotorStatus, 2U> & motors, std::uint8_t expected) noexcept
{
  return std::all_of(motors.begin(), motors.end(), [expected](const MotorStatus & motor) {
      return motor.valid && motor.status == expected;
    });
}

[[nodiscard]] bool none_enabled(const std::array<MotorStatus, 2U> & motors) noexcept
{
  return std::none_of(motors.begin(), motors.end(), [](const MotorStatus & motor) {
      return motor.valid && motor.status == 1U;
    });
}

}  // namespace

HeadStateMachine::HeadStateMachine(
  std::chrono::nanoseconds feedback_timeout,
  std::chrono::nanoseconds transition_timeout)
: feedback_timeout_(feedback_timeout), transition_timeout_(transition_timeout)
{
  if (feedback_timeout_ <= std::chrono::nanoseconds::zero() ||
    transition_timeout_ <= std::chrono::nanoseconds::zero())
  {
    throw std::invalid_argument{"DaMiao head state-machine timeouts must be positive"};
  }
}

HeadActions HeadStateMachine::step(
  std::chrono::nanoseconds now, bool enable_requested,
  std::uint64_t reset_generation,
  const std::array<MotorStatus, 2U> & motors)
{
  const bool fresh = all_fresh(now, motors);
  const bool transition_timed_out = now - transition_started_at_ > transition_timeout_;

  switch (phase_) {
    case HeadPhase::disabled:
      if (any_fault(motors)) {
        return latch_fault();
      }
      if (reset_generation != handled_reset_generation_) {
        handled_reset_generation_ = reset_generation;
      }
      if (enable_requested && fresh && all_status(motors, 0U)) {
        transition(HeadPhase::enabling, now);
        return HeadActions{true, true, true, false, false, false};
      }
      return {};

    case HeadPhase::enabling:
      if (!enable_requested) {
        transition(HeadPhase::disabling, now);
        return HeadActions{false, false, false, true, false, false};
      }
      if (!fresh || any_fault(motors) || transition_timed_out) {
        return latch_fault();
      }
      if (all_received_after_transition(motors) && all_status(motors, 1U)) {
        transition(HeadPhase::enabled, now);
        return HeadActions{true, true, false, false, false, true};
      }
      return HeadActions{false, true, true, false, false, false};

    case HeadPhase::enabled:
      if (!enable_requested) {
        transition(HeadPhase::disabling, now);
        return HeadActions{false, false, false, true, false, false};
      }
      if (!fresh || any_fault(motors) || !all_status(motors, 1U)) {
        return latch_fault();
      }
      return HeadActions{false, false, false, false, false, true};

    case HeadPhase::disabling:
      if (!fresh || any_fault(motors) || transition_timed_out) {
        return latch_fault();
      }
      if (all_received_after_transition(motors) && all_status(motors, 0U)) {
        transition(HeadPhase::disabled, now);
        return {};
      }
      return HeadActions{false, false, false, true, false, false};

    case HeadPhase::fault_latched:
      if (!enable_requested && reset_generation != handled_reset_generation_) {
        pending_reset_generation_ = reset_generation;
        transition(HeadPhase::reset_disabling, now);
        return HeadActions{false, false, false, true, false, false};
      }
      return HeadActions{false, false, false, true, false, false};

    case HeadPhase::reset_disabling:
      if (enable_requested || !fresh || transition_timed_out) {
        return latch_fault();
      }
      if (all_received_after_transition(motors) && none_enabled(motors)) {
        transition(HeadPhase::resetting, now);
        return HeadActions{false, false, false, false, true, false};
      }
      return HeadActions{false, false, false, true, false, false};

    case HeadPhase::resetting:
      if (enable_requested || !fresh || transition_timed_out) {
        return latch_fault();
      }
      if (all_received_after_transition(motors) && all_status(motors, 0U)) {
        phase_ = HeadPhase::disabled;
        fault_latched_ = false;
        handled_reset_generation_ = pending_reset_generation_;
        return {};
      }
      return HeadActions{false, false, false, false, true, false};
  }
  return latch_fault();
}

HeadPhase HeadStateMachine::phase() const noexcept
{
  return phase_;
}

bool HeadStateMachine::fault_latched() const noexcept
{
  return fault_latched_;
}

std::uint64_t HeadStateMachine::handled_reset_generation() const noexcept
{
  return handled_reset_generation_;
}

HeadActions HeadStateMachine::force_fault() noexcept
{
  return latch_fault();
}

bool HeadStateMachine::all_fresh(
  std::chrono::nanoseconds now,
  const std::array<MotorStatus, 2U> & motors) const noexcept
{
  return std::all_of(motors.begin(), motors.end(), [&](const MotorStatus & motor) {
      return motor.valid && now >= motor.received_at &&
             now - motor.received_at <= feedback_timeout_;
    });
}

bool HeadStateMachine::all_received_after_transition(
  const std::array<MotorStatus, 2U> & motors) const noexcept
{
  return std::all_of(motors.begin(), motors.end(), [&](const MotorStatus & motor) {
      return motor.valid && motor.received_at > transition_started_at_;
    });
}

void HeadStateMachine::transition(HeadPhase phase, std::chrono::nanoseconds now) noexcept
{
  phase_ = phase;
  transition_started_at_ = now;
}

HeadActions HeadStateMachine::latch_fault() noexcept
{
  phase_ = HeadPhase::fault_latched;
  fault_latched_ = true;
  return HeadActions{false, false, false, true, false, false};
}

CommandRateLimiter::CommandRateLimiter(std::chrono::nanoseconds period)
: period_(period)
{
  if (period_ <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument{"DaMiao command period must be positive"};
  }
}

bool CommandRateLimiter::due(std::chrono::nanoseconds now) noexcept
{
  if (!last_emitted_at_.has_value() || now < *last_emitted_at_ ||
    now - *last_emitted_at_ >= period_)
  {
    last_emitted_at_ = now;
    return true;
  }
  return false;
}

void CommandRateLimiter::reset() noexcept
{
  last_emitted_at_.reset();
}

}  // namespace robot_hw_can
