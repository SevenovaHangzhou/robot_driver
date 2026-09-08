#ifndef DM_SWERVE_DRIVER__CONTROL_LOOP_HPP_
#define DM_SWERVE_DRIVER__CONTROL_LOOP_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/feedback_router.hpp"
#include "dm_swerve_driver/params.hpp"
#include "dm_swerve_driver/swerve_odometry.hpp"

namespace dm_swerve_driver {

enum class DriverLogLevel {
  debug,
  info,
  warning,
  error,
};

struct ControlLoopOutput {
  std::chrono::steady_clock::time_point timestamp{};
  Pose2d pose{};
  ChassisSpeeds command{};
  std::array<double, kSwerveModuleCount> steering_angle_rad{};
  std::array<double, kSwerveModuleCount> wheel_distance_m{};
  std::array<double, kSwerveModuleCount> wheel_velocity_mps{};
  bool alignment_gated{false};
  ChassisSpeeds measured_twist{};
  bool imu_fallback{false};
  std::size_t valid_module_count{kSwerveModuleCount};
};

struct ControlLoopStatus {
  bool initialized{false};
  bool running{false};
  bool command_timed_out{true};
  bool imu_fallback{false};
  bool bus_silent{false};
  std::uint64_t completed_cycles{0U};
  std::uint64_t loop_overruns{0U};
  std::uint64_t unknown_frames{0U};
  std::uint64_t rejected_frames{0U};
  std::array<DmMotorHealth, kMotorCount> motors{};
  std::array<MotorLimits, kMotorCount> motor_limits{};
  Pose2d pose{};
  bool faulted{false};
  bool fault_latched{false};
  bool transport_faulted{false};
  std::uint64_t stale_frames{0U};
  std::uint64_t unknown_frames_last_cycle{0U};
  std::uint64_t rejected_frames_last_cycle{0U};
  std::uint64_t stale_frames_last_cycle{0U};
  std::array<std::uint32_t, kMotorCount> recovery_attempts{};
};

struct ControlLoopCallbacks {
  std::function<void(const ControlLoopOutput &)> publish_output;
  std::function<void(DriverLogLevel, const std::string &)> log;
};

class ControlLoop final {
public:
  ControlLoop(
    DriverParameters parameters,
    std::unique_ptr<CanTransport> transport,
    ControlLoopCallbacks callbacks = {});
  ~ControlLoop() noexcept;

  ControlLoop(const ControlLoop &) = delete;
  ControlLoop & operator=(const ControlLoop &) = delete;
  ControlLoop(ControlLoop &&) = delete;
  ControlLoop & operator=(ControlLoop &&) = delete;

  [[nodiscard]] bool initialize(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  [[nodiscard]] bool step(std::chrono::steady_clock::time_point now);
  void start();
  void stop() noexcept;

  void submit_command(
    const ChassisSpeeds & command,
    std::chrono::steady_clock::time_point timestamp);
  bool submit_imu_yaw(
    double yaw_rad,
    std::chrono::steady_clock::time_point timestamp,
    double yaw_rate_radps = 0.0);
  void request_clear_faults() noexcept;
  void restore_fault_state(
    bool fault_latched,
    const std::array<std::uint32_t, kMotorCount> & recovery_attempts);

  [[nodiscard]] bool is_running() const noexcept;
  [[nodiscard]] ControlLoopStatus status() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CONTROL_LOOP_HPP_
