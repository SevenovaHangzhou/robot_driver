#ifndef DM_SWERVE_DRIVER__KINCO_SAFETY_MONITOR_HPP_
#define DM_SWERVE_DRIVER__KINCO_SAFETY_MONITOR_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

#include "dm_swerve_driver/kinco_pdo.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

enum class KincoAxisCondition {
  disabled,
  enabled,
  recoverable_fault,
  latching_fault,
};

struct KincoAxisHealth {
  bool has_feedback{false};
  KincoAxisCondition condition{KincoAxisCondition::disabled};
  std::uint64_t received_frames{0U};
  std::uint64_t missed_frames{0U};
  std::uint64_t consecutive_missed_frames{0U};

  [[nodiscard]] bool enabled() const noexcept;
};

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

struct KincoRecoveryActions {
  std::array<bool, kKincoAxisCount> clear_fault{};
  std::array<bool, kKincoAxisCount> reenable{};
};

class KincoSafetyMonitor final {
public:
  explicit KincoSafetyMonitor(const DriverParameters & parameters);

  [[nodiscard]] CommandDecision command_for_cycle(
    const ChassisSpeeds & command,
    std::optional<std::chrono::steady_clock::time_point> command_timestamp,
    std::chrono::steady_clock::time_point now);
  [[nodiscard]] YawDecision update_yaw(
    const std::optional<TimedYawSample> & imu,
    double wheel_delta_yaw_rad,
    std::chrono::steady_clock::time_point now);
  [[nodiscard]] KincoRecoveryActions recovery_actions(
    const std::array<KincoAxisHealth, kKincoAxisCount> & axes,
    std::chrono::steady_clock::time_point now);
  [[nodiscard]] KincoRecoveryActions manual_clear_actions() noexcept;
  [[nodiscard]] bool complete_manual_clear(
    const std::array<KincoAxisHealth, kKincoAxisCount> & axes,
    const std::array<bool, kKincoAxisCount> & enable_confirmed) noexcept;
  void restore_recovery_state(
    const std::array<std::uint32_t, kKincoAxisCount> & recovery_attempts,
    bool fault_latched) noexcept;
  void mark_transport_failure() noexcept;
  void observe_feedback(const std::array<bool, kKincoAxisCount> & received) noexcept;
  [[nodiscard]] bool observe_steering_limit_violation(bool active) noexcept;

  [[nodiscard]] bool faulted() const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept;
  [[nodiscard]] bool transport_faulted() const noexcept;
  [[nodiscard]] bool steering_limit_faulted() const noexcept;
  [[nodiscard]] const std::array<std::uint32_t, kKincoAxisCount> & recovery_attempts()
    const noexcept;

private:
  DriverParameters parameters_;
  bool command_timed_out_{true};
  bool yaw_initialized_{false};
  bool imu_fallback_{true};
  double yaw_rad_{0.0};
  double imu_offset_rad_{0.0};
  std::array<std::optional<std::chrono::steady_clock::time_point>, kKincoAxisCount>
    last_recovery_{};
  std::array<std::uint32_t, kKincoAxisCount> recovery_attempts_{};
  bool faulted_{false};
  bool fault_latched_{false};
  bool transport_faulted_{false};
  bool steering_limit_faulted_{false};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_SAFETY_MONITOR_HPP_
