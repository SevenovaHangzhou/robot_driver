#ifndef SWERVE_DRIVER__STOP_TRANSITIONS_HPP_
#define SWERVE_DRIVER__STOP_TRANSITIONS_HPP_
#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
namespace ctrl_swerve
{
// The clock must be sampled AFTER copying inputs: a callback between an earlier
// clock read and the buffer read must not turn a new command into a future one.
template <class Reader, class Clock>
auto readThenTimestamp(Reader reader, Clock clock)
{
  auto snapshot = reader();
  return std::make_pair(snapshot, clock());
}
struct StopTransition
{
  uint64_t sequence{0};
  double ros_stamp{0}, period{0}, command_age{0};
  int status{0}, admission{0};
  bool gated{false}, waiting{false}, navigation{false};
  std::array<double, 3> command{}, body{};
  double max_error{0}, angle{0}, goal{0}, max_wheel_reference{0};
  int module{0};
  unsigned branches{0};
};
// Single RT producer / diagnostic-timer consumer. No RT allocation or blocking.
// Preserve one-cycle causes until the consumer drains them; report overflow.
class StopTransitionQueue
{
  static constexpr unsigned capacity = 64;
  std::array<StopTransition, capacity> entries_{};
  std::atomic<unsigned> head_{0}, tail_{0};
  std::atomic<uint64_t> dropped_{0};

public:
  bool push(const StopTransition & value) noexcept
  {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = (head + 1U) % capacity;
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    entries_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(StopTransition & value) noexcept
  {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
    value = entries_[tail];
    tail_.store((tail + 1U) % capacity, std::memory_order_release);
    return true;
  }
  uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
};
// Stable diagnostic names, never used to relax command admission.
inline const char * admissionName(int reason) noexcept
{
  switch (reason) {
    case 0:
      return "fresh";
    case 1:
      return "invalid_or_missing";
    case 2:
      return "generation_mismatch";
    case 3:
      return "handoff_or_feedback_barrier";
    case 4:
      return "negative_receive_age";
    case 5:
      return "command_age_timeout";
    case 6:
      return "navigation_owner_closed";
    default:
      return "unknown";
  }
}
}  // namespace ctrl_swerve
#endif
