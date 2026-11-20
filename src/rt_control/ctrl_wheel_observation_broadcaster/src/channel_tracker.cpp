#include "ctrl_wheel_observation_broadcaster/channel_tracker.hpp"

#include <cmath>

namespace ctrl_wheel_observation_broadcaster
{

void ChannelTracker::reset() noexcept
{
  state_ = ChannelState{};
  seen_ = had_valid_ = gap_ = false;
}

const ChannelState & ChannelTracker::update(
  const ChannelInput & input, std::int64_t now_ns, double timeout_ms) noexcept
{
  const bool has_age = std::isfinite(input.age_ms) && input.age_ms >= 0.0;
  if (input.new_sample && has_age) {
    seen_ = true;
    ++state_.sample_sequence;
    state_.sample_ns = now_ns - static_cast<std::int64_t>(std::llround(input.age_ms * 1.0e6));
  }
  if (!seen_) {
    state_.reason = Reason::not_ready;
  } else if (!has_age || input.age_ms > timeout_ms) {
    state_.reason = Reason::stale;
  } else if (!input.finite) {
    state_.reason = Reason::nonfinite;
  } else if (input.fault) {
    state_.reason = Reason::device_fault;
  } else {
    state_.reason = Reason::ok;
  }
  state_.valid = state_.reason == Reason::ok;
  // Any loss of validity may hide a reset, so the next valid sample starts a new epoch.
  if (!state_.valid) {
    gap_ = gap_ || had_valid_;
  } else {
    if (gap_ || (input.new_sample && input.discontinuity && had_valid_)) {
      ++state_.epoch;
    }
    gap_ = false;
    had_valid_ = true;
  }
  return state_;
}

}  // namespace ctrl_wheel_observation_broadcaster
