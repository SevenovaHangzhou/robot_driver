#ifndef DM_SWERVE_DRIVER__SAFETY_MONITOR_HPP_
#define DM_SWERVE_DRIVER__SAFETY_MONITOR_HPP_

#include <array>
#include <chrono>
#include <optional>

#include "dm_swerve_driver/feedback_router.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

struct TimedYawSample {
  double yaw_rad{0.0};
  std::chrono::steady_clock::time_point timestamp{};
};

struct CommandDecision {
  ChassisSpeeds command{};
  bool timed_out{true};
  bool state_changed{false};
};

struct YawDecision {
  double yaw_rad{0.0};
  bool imu_fallback{true};
  bool source_changed{false};
};

struct RecoveryActions {
  std::array<bool, kMotorCount> clear_fault{};
  std::array<bool, kMotorCount> reenable{};
};

class SafetyMonitor final {
public:
  explicit SafetyMonitor(const DriverParameters & parameters);

  [[nodiscard]] CommandDecision command_for_cycle(
    const ChassisSpeeds & command,
    std::optional<std::chrono::steady_clock::time_point> command_timestamp,
    std::chrono::steady_clock::time_point now);

  [[nodiscard]] YawDecision update_yaw(
    const std::optional<TimedYawSample> & imu,
    double wheel_delta_yaw_rad,
    std::chrono::steady_clock::time_point now);

  [[nodiscard]] RecoveryActions recovery_actions(
    const std::array<DmMotorHealth, kMotorCount> & motors,
    std::chrono::steady_clock::time_point now);

  [[nodiscard]] bool all_bus_silent(
    const std::array<DmMotorHealth, kMotorCount> & motors) const noexcept;

private:
  DriverParameters parameters_;
  bool command_timed_out_{true};
  bool yaw_initialized_{false};
  bool imu_fallback_{true};
  double yaw_rad_{0.0};
  double imu_offset_rad_{0.0};
  std::array<std::optional<std::chrono::steady_clock::time_point>, kMotorCount>
    last_clear_fault_{};
  std::array<std::optional<std::chrono::steady_clock::time_point>, kMotorCount>
    last_reenable_{};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SAFETY_MONITOR_HPP_
