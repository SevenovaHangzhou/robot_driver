#ifndef DM_SWERVE_DRIVER__PARAMS_HPP_
#define DM_SWERVE_DRIVER__PARAMS_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "dm_swerve_driver/dm_motor.hpp"
#include "dm_swerve_driver/swerve_kinematics.hpp"
#include "dm_swerve_driver/swerve_module.hpp"

namespace dm_swerve_driver {

inline constexpr std::uint32_t kTimeoutCountsPerMillisecond{20U};

struct CanParameters {
  std::string interface_name{"vcan0"};
  std::int64_t feedback_deadline_us{4000};
  bool write_timeout_register{false};
  std::int64_t timeout_register_ms{100};
};

struct ControlParameters {
  double rate_hz{100.0};
  int realtime_priority{0};
  double cmd_vel_timeout_s{0.25};
  bool hold_steer_on_timeout{true};
};

struct ChassisParameters {
  double wheelbase_m{0.5};
  double track_m{0.4};
  double wheel_radius_m{0.1};
  double max_wheel_speed_mps{2.0};
  double max_linear_speed_mps{1.5};
  double max_angular_speed_radps{3.0};
  double max_wheel_acceleration_mps2{3.0};
  double velocity_deadband_mps{0.01};
  double align_threshold_rad{0.349};
};

struct SteeringParameters {
  double gear_ratio{1.0};
  double kp{30.0};
  double kd{1.0};
  double kff_omega{0.9};
  double max_ff_speed_radps{3.0};
  std::array<double, kSwerveModuleCount> zero_offset_rad{};
  std::array<bool, kSwerveModuleCount> inverted{};
};

struct DriveParameters {
  double gear_ratio{1.0};
  double kd{2.0};
  double ks{0.1};
  double kv{1.0};
  double ka{0.1};
  std::array<bool, kSwerveModuleCount> inverted{};
};

struct MotorParameters {
  std::array<std::uint16_t, kSwerveModuleCount> steering_esc_id{1U, 2U, 3U, 4U};
  std::array<std::uint16_t, kSwerveModuleCount> steering_mst_id{0x11U, 0x12U, 0x13U, 0x14U};
  std::array<std::uint16_t, kSwerveModuleCount> drive_esc_id{5U, 6U, 7U, 8U};
  std::array<std::uint16_t, kSwerveModuleCount> drive_mst_id{0x15U, 0x16U, 0x17U, 0x18U};
};

struct SafetyParameters {
  std::uint64_t feedback_silent_cycles{50U};
  double reenable_period_s{1.0};
};

struct OdometryParameters {
  std::string imu_topic{"/imu/data"};
  double imu_timeout_s{0.2};
  bool publish_tf{true};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
  double publish_rate_hz{50.0};
};

struct DriverParameters {
  CanParameters can{};
  ControlParameters control{};
  ChassisParameters chassis{};
  MotorLimits limits_fallback{12.5, 30.0, 10.0};
  SteeringParameters steering{};
  DriveParameters drive{};
  MotorParameters motors{};
  SafetyParameters safety{};
  OdometryParameters odometry{};
};

[[nodiscard]] DriverParameters default_parameters();
[[nodiscard]] std::vector<std::string> parameter_errors(
  const DriverParameters & parameters);
void validate_parameters(const DriverParameters & parameters);

[[nodiscard]] std::array<Translation2d, kSwerveModuleCount> module_locations(
  const DriverParameters & parameters);
[[nodiscard]] SteeringModuleConfig steering_module_config(
  const DriverParameters & parameters, std::size_t module_index);
[[nodiscard]] DriveModuleConfig drive_module_config(
  const DriverParameters & parameters, std::size_t module_index);
[[nodiscard]] DmMotorConfig steering_motor_config(
  const DriverParameters & parameters, std::size_t module_index);
[[nodiscard]] DmMotorConfig drive_motor_config(
  const DriverParameters & parameters, std::size_t module_index);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__PARAMS_HPP_
