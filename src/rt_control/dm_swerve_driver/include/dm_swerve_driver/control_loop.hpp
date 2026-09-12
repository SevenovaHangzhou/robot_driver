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
#include "dm_swerve_driver/kinco_backend.hpp"
#include "dm_swerve_driver/external_steering_encoder.hpp"

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
  std::array<bool, kSwerveModuleCount> slipping_modules{};
  bool slip_detected{false};
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
  bool steering_limit_faulted{false};
  std::array<bool, kSwerveModuleCount> slipping_modules{};
  bool slip_detected{false};
  bool kinco_backend{false};
  KincoEthercatCycle ethercat{};
  std::array<SteeringAngleSelection, kSwerveModuleCount> steering_sources{};
  std::array<bool, kSwerveModuleCount> encoder_heartbeat{};
  std::array<std::uint8_t, kSwerveModuleCount> encoder_nmt_state{};
};

struct ControlLoopCallbacks {
  std::function<void(const ControlLoopOutput &)> publish_output;
  std::function<void(DriverLogLevel, const std::string &)> log;
};

class ControlRunner {
public:
  virtual ~ControlRunner() = default;
  [[nodiscard]] virtual bool initialize(std::chrono::steady_clock::time_point now) = 0;
  virtual void start() = 0;
  virtual void stop() noexcept = 0;
  virtual void submit_command(const ChassisSpeeds &, std::chrono::steady_clock::time_point) = 0;
  virtual bool submit_imu_yaw(double, std::chrono::steady_clock::time_point, double = 0.0) = 0;
  virtual void request_clear_faults() noexcept = 0;
  virtual void restore_fault_state(
    bool, const std::array<std::uint32_t, kMotorCount> &) = 0;
  [[nodiscard]] virtual bool is_running() const noexcept = 0;
  [[nodiscard]] virtual ControlLoopStatus status() const = 0;
};

class ControlLoop final : public ControlRunner {
public:
  ControlLoop(
    DriverParameters parameters,
    std::unique_ptr<CanTransport> transport,
    ControlLoopCallbacks callbacks = {});
  ~ControlLoop() noexcept override;

  ControlLoop(const ControlLoop &) = delete;
  ControlLoop & operator=(const ControlLoop &) = delete;
  ControlLoop(ControlLoop &&) = delete;
  ControlLoop & operator=(ControlLoop &&) = delete;

  [[nodiscard]] bool initialize(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()) override;
  [[nodiscard]] bool step(std::chrono::steady_clock::time_point now);
  void start() override;
  void stop() noexcept override;

  void submit_command(
    const ChassisSpeeds & command,
    std::chrono::steady_clock::time_point timestamp) override;
  bool submit_imu_yaw(
    double yaw_rad,
    std::chrono::steady_clock::time_point timestamp,
    double yaw_rate_radps = 0.0) override;
  void request_clear_faults() noexcept override;
  void restore_fault_state(
    bool fault_latched,
    const std::array<std::uint32_t, kMotorCount> & recovery_attempts) override;

  [[nodiscard]] bool is_running() const noexcept override;
  [[nodiscard]] ControlLoopStatus status() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CONTROL_LOOP_HPP_
