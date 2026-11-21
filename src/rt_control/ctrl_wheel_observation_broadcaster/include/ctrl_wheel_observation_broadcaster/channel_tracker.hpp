#ifndef CTRL_WHEEL_OBSERVATION_BROADCASTER__CHANNEL_TRACKER_HPP_
#define CTRL_WHEEL_OBSERVATION_BROADCASTER__CHANNEL_TRACKER_HPP_

#include <cstdint>

namespace ctrl_wheel_observation_broadcaster
{

// Values match robot_rt_control_interfaces/msg/WheelFeedbackChannel REASON_*.
enum class Reason : std::uint8_t {
  ok = 0,
  not_ready = 1,
  stale = 2,
  nonfinite = 3,
  device_fault = 4
};

// One cycle's view of one feedback channel, as read from the state interfaces.
struct ChannelInput
{
  // Milliseconds since the sample was received; non-finite when there is none.
  double age_ms{0.0};
  bool finite{true};
  bool fault{false};
  // A new device sample arrived since the previous cycle.
  bool new_sample{false};
  // The new sample cannot continue the previous one (counter reset, impossible jump).
  bool discontinuity{false};
};

struct ChannelState
{
  // ROS time of the sample, nanoseconds; 0 before the first sample.
  std::int64_t sample_ns{0};
  std::uint64_t sample_sequence{0U};
  std::uint32_t epoch{0U};
  bool valid{false};
  Reason reason{Reason::not_ready};
};

// Sample identity and continuity of one channel. Called once per controller cycle; bounded,
// allocation-free.
class ChannelTracker
{
public:
  void reset() noexcept;
  const ChannelState & update(
    const ChannelInput & input, std::int64_t now_ns, double timeout_ms) noexcept;
  const ChannelState & state() const noexcept { return state_; }

private:
  ChannelState state_{};
  bool seen_{false};
  bool had_valid_{false};
  bool gap_{false};
};

}  // namespace ctrl_wheel_observation_broadcaster

#endif  // CTRL_WHEEL_OBSERVATION_BROADCASTER__CHANNEL_TRACKER_HPP_
