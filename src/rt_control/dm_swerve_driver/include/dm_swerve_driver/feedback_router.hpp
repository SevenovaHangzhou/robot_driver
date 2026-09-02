#ifndef DM_SWERVE_DRIVER__FEEDBACK_ROUTER_HPP_
#define DM_SWERVE_DRIVER__FEEDBACK_ROUTER_HPP_

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "dm_swerve_driver/dm_motor.hpp"
#include "dm_swerve_driver/socketcan_interface.hpp"

namespace dm_swerve_driver {

inline constexpr std::size_t kMotorCount{8U};

struct FeedbackRouteResult {
  std::array<bool, kMotorCount> received{};
  std::size_t accepted_frames{0U};
  std::size_t unknown_frames{0U};
  std::size_t rejected_frames{0U};
};

using FeedbackRouteLogger = std::function<void(const std::string &)>;

[[nodiscard]] FeedbackRouteResult route_feedback_frames(
  const std::vector<ReceivedCanFrame> & frames,
  const std::array<DmMotor *, kMotorCount> & motors,
  const FeedbackRouteLogger & log_warning) noexcept;

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__FEEDBACK_ROUTER_HPP_
