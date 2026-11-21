#ifndef CTRL_ENABLE_MANAGER__EMERGENCY_STOP_INPUT_HPP_
#define CTRL_ENABLE_MANAGER__EMERGENCY_STOP_INPUT_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "rt_control_interfaces/msg/emergency_stop_state.hpp"

namespace ctrl_enable_manager
{
// observe() is serialized in its own non-RT callback group. RT reads only lock-free atomics.
class EmergencyStopInput
{
public:
  using Message = rt_control_interfaces::msg::EmergencyStopState;
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

  void configure(std::uint64_t timeout_ns)
  {
    timeout_ns_ = timeout_ns;
    deadline_.store(0);
    event_.store(false);
    reported_state_.store(Message::UNKNOWN);
    seen_ = false;
    sequence_ = 0;
    sample_deadline_ = 0;
  }

  void observe(const Message & message, std::uint64_t steady_ns, std::int64_t ros_ns)
  {
    reported_state_.store(message.valid ? message.state : Message::UNKNOWN);
    const auto stamp = static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                       message.header.stamp.nanosec;
    const auto transport_age = ros_ns - stamp;
    const bool identified =
      message.sample_sequence > 0 &&
      std::any_of(
        message.producer_instance_id.uuid.begin(), message.producer_instance_id.uuid.end(),
        [](uint8_t b) { return b != 0; });
    const bool same_producer = seen_ && producer_ == message.producer_instance_id;
    if (message.valid && message.state == Message::PRESSED) {
      // A stop is effective even when stale. Remember its sequence so an older or conflicting
      // RELEASED sample cannot become permission to reset after the RT loop consumes the event.
      if (identified && (!same_producer || message.sample_sequence >= sequence_)) {
        producer_ = message.producer_instance_id;
        sequence_ = message.sample_sequence;
        seen_ = true;
      }
      sample_deadline_ = 0;
      deadline_.store(1U, std::memory_order_release);
      event_.store(true, std::memory_order_release);
      return;
    }
    const bool released =
      message.valid && message.state == Message::RELEASED && !message.stop_requested &&
      identified && std::isfinite(message.sample_age_seconds) &&
      message.sample_age_seconds >= 0.0 && transport_age >= 0 && timeout_ns_ > 0;
    const double age_ns = message.sample_age_seconds * 1e9 + static_cast<double>(transport_age);
    if (!released || !std::isfinite(age_ns) || age_ns >= static_cast<double>(timeout_ns_)) {
      deadline_.store(0U, std::memory_order_release);
      event_.store(true, std::memory_order_release);
      return;
    }
    if (same_producer && message.sample_sequence < sequence_) {
      deadline_.store(0, std::memory_order_release);
      event_.store(true, std::memory_order_release);
      return;
    }
    const auto candidate = steady_ns + timeout_ns_ - static_cast<std::uint64_t>(age_ns);
    if (same_producer && message.sample_sequence == sequence_) {
      // Republishing one sample must never refresh its local freshness deadline.
      sample_deadline_ = std::min(sample_deadline_, candidate);
    } else {
      if (seen_ && !same_producer) {
        event_.store(true, std::memory_order_release);  // Restart is not permission to resume.
      }
      sample_deadline_ = candidate;
    }
    producer_ = message.producer_instance_id;
    sequence_ = message.sample_sequence;
    seen_ = true;
    deadline_.store(sample_deadline_, std::memory_order_release);
  }

  std::uint8_t state(std::uint64_t now_ns) const noexcept
  {
    const auto value = deadline_.load(std::memory_order_acquire);
    if (value == 1) return Message::PRESSED;
    return value > 1 && now_ns < value ? Message::RELEASED : Message::UNKNOWN;
  }
  std::uint8_t reportedState() const noexcept { return reported_state_.load(); }
  bool released(std::uint64_t now_ns) const noexcept
  {
    return state(now_ns) == Message::RELEASED && !event_.load(std::memory_order_acquire);
  }
  bool consumeEvent() noexcept { return event_.exchange(false, std::memory_order_acq_rel); }

private:
  std::uint64_t timeout_ns_{0};
  std::atomic<std::uint64_t> deadline_{0};
  std::atomic<std::uint8_t> reported_state_{Message::UNKNOWN};
  std::atomic_bool event_{false};
  bool seen_{false};
  unique_identifier_msgs::msg::UUID producer_;
  std::uint64_t sequence_{0}, sample_deadline_{0};
};
}  // namespace ctrl_enable_manager
#endif
