#ifndef DM_SWERVE_DRIVER__CONTROL_TYPES_HPP_
#define DM_SWERVE_DRIVER__CONTROL_TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "dm_swerve_driver/external_steering_encoder.hpp"
#include "dm_swerve_driver/kinco_backend.hpp"
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
  std::array<bool, kSwerveModuleCount> slipping_modules{};
  bool slip_detected{false};
};

struct ControlLoopStatus {
  bool initialized{false};
  bool running{false};
  bool command_timed_out{true};
  bool imu_fallback{false};
  std::uint64_t completed_cycles{0U};
  std::uint64_t loop_overruns{0U};
  Pose2d pose{};
  bool faulted{false};
  bool fault_latched{false};
  bool transport_faulted{false};
  std::array<std::uint32_t, kKincoAxisCount> recovery_attempts{};
  bool steering_limit_faulted{false};
  std::array<bool, kSwerveModuleCount> slipping_modules{};
  bool slip_detected{false};
  KincoEthercatCycle ethercat{};
  std::array<SteeringAngleSelection, kSwerveModuleCount> steering_sources{};
  std::array<bool, kSwerveModuleCount> encoder_heartbeat{};
  std::array<std::uint8_t, kSwerveModuleCount> encoder_nmt_state{};
};

struct ControlLoopCallbacks {
  std::function<void(const ControlLoopOutput &)> publish_output;
  std::function<void(DriverLogLevel, const std::string &)> log;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__CONTROL_TYPES_HPP_
