#include "dm_swerve_driver/feedback_router.hpp"

namespace dm_swerve_driver {
namespace {

void log_safely(
  const FeedbackRouteLogger & log_warning, const char * message) noexcept
{
  try {
    if (log_warning) {
      log_warning(message);
    }
  } catch (...) {
  }
}

[[nodiscard]] std::size_t find_motor(
  std::uint16_t mst_id,
  const std::array<DmMotor *, kMotorCount> & motors) noexcept
{
  for (std::size_t index{0U}; index < motors.size(); ++index) {
    if (motors[index] != nullptr && motors[index]->mst_id() == mst_id) {
      return index;
    }
  }
  return motors.size();
}

}  // namespace

FeedbackRouteResult route_feedback_frames(
  const std::vector<ReceivedCanFrame> & frames,
  const std::array<DmMotor *, kMotorCount> & motors,
  const FeedbackRouteLogger & log_warning) noexcept
{
  FeedbackRouteResult result{};
  for (const auto & received : frames) {
    const std::size_t index{find_motor(received.frame.id, motors)};
    if (index == motors.size()) {
      ++result.unknown_frames;
      log_safely(log_warning, "dropping frame with unknown MST_ID");
      continue;
    }

    try {
      static_cast<void>(motors[index]->accept_feedback(received.frame));
      result.received[index] = true;
      ++result.accepted_frames;
    } catch (...) {
      ++result.rejected_frames;
      log_safely(log_warning, "dropping rejected motor feedback frame");
    }
  }
  return result;
}

}  // namespace dm_swerve_driver
