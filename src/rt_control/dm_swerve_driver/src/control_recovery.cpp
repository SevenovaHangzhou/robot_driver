#include "control_loop_impl.hpp"

#include <chrono>
#include <vector>

namespace dm_swerve_driver {
namespace {

constexpr std::size_t kMaxFeedbackCollections{16U};

void merge_feedback_routes(
  FeedbackRouteResult & destination,
  const FeedbackRouteResult & source) noexcept
{
  for (std::size_t index{0U}; index < destination.received.size(); ++index) {
    destination.received[index] = destination.received[index] || source.received[index];
  }
  destination.accepted_frames += source.accepted_frames;
  destination.unknown_frames += source.unknown_frames;
  destination.rejected_frames += source.rejected_frames;
  destination.stale_frames += source.stale_frames;
}

[[nodiscard]] bool selected_feedback_received(
  const std::array<bool, kMotorCount> & received,
  const std::array<bool, kMotorCount> & selected) noexcept
{
  for (std::size_t index{0U}; index < selected.size(); ++index) {
    if (selected[index] && !received[index]) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::array<bool, kMotorCount> ControlLoop::Impl::dispatch_recovery_actions(
  const RecoveryActions & actions, SteadyClock::time_point now)
{
  static_cast<void>(send_special_actions(
      actions.clear_fault, SpecialCommand::clear_fault, now));
  return send_special_actions(actions.reenable, SpecialCommand::enable, now).received;
}

FeedbackRouteResult ControlLoop::Impl::send_special_actions(
  const std::array<bool, kMotorCount> & selected,
  SpecialCommand command,
  SteadyClock::time_point now)
{
  const auto motors = motor_pointers();
  std::vector<CanFrame> frames;
  for (std::size_t index{0U}; index < selected.size(); ++index) {
    if (selected[index]) {
      frames.push_back(motors[index]->special_command(command));
    }
  }
  if (frames.empty()) {
    return {};
  }
  const auto command_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  transport_->write_batch(frames);
  const auto deadline = now +
    std::chrono::microseconds{parameters_.can.feedback_deadline_us};
  FeedbackRouteResult route{};
  for (std::size_t attempt{0U}; attempt < kMaxFeedbackCollections &&
    !selected_feedback_received(route.received, selected); ++attempt)
  {
    const auto replies = transport_->collect(frames.size(), deadline);
    if (replies.empty()) {
      break;
    }
    const auto chunk = route_feedback_frames(
      replies, motors,
      [&](const std::string & message) {log(DriverLogLevel::warning, message);},
      command_timestamp);
    merge_feedback_routes(route, chunk);
  }
  return route;
}

}  // namespace dm_swerve_driver
