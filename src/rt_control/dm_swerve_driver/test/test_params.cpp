#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <limits>

#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] bool contains_error(
  const std::vector<std::string> & errors, const std::string & fragment)
{
  for (const auto & error : errors) {
    if (error.find(fragment) != std::string::npos) {
      return true;
    }
  }
  return false;
}

TEST(DriverParametersTest, DefaultsAreRunnablePlaceholders)
{
  const auto parameters = default_parameters();
  EXPECT_TRUE(parameter_errors(parameters).empty());
  EXPECT_NO_THROW(validate_parameters(parameters));
  EXPECT_EQ(parameters.can.interface_name, "vcan0");
  EXPECT_FALSE(parameters.can.allow_fallback_limits);
  EXPECT_TRUE(parameters.can.write_timeout_register);
  EXPECT_EQ(parameters.can.write_timeout_us, 2000);
  EXPECT_DOUBLE_EQ(parameters.steering.max_ff_speed_radps, 3.0);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -kPi);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_max_rad, kPi);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_margin_rad, 0.0);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_tolerance_rad, 0.0);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_residual_threshold, 0.25);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_covariance_scale, 4.0);
}

TEST(DriverParametersTest, AllowsFallbackLimitsOnlyWhenExplicitlyUsingVcan)
{
  auto parameters = default_parameters();
  parameters.can.allow_fallback_limits = true;
  EXPECT_TRUE(parameter_errors(parameters).empty());

  parameters.can.interface_name = "can0";
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "allow_fallback_limits"));

  parameters.can.interface_name = "fake0";
  EXPECT_FALSE(contains_error(parameter_errors(parameters), "allow_fallback_limits"));
}

TEST(DriverParametersTest, DerivesRep103ModuleLocations)
{
  auto parameters = default_parameters();
  parameters.chassis.wheelbase_m = 0.6;
  parameters.chassis.track_m = 0.4;
  const auto locations = module_locations(parameters);

  EXPECT_DOUBLE_EQ(locations[0].x, 0.3);
  EXPECT_DOUBLE_EQ(locations[0].y, 0.2);
  EXPECT_DOUBLE_EQ(locations[1].x, 0.3);
  EXPECT_DOUBLE_EQ(locations[1].y, -0.2);
  EXPECT_DOUBLE_EQ(locations[2].x, -0.3);
  EXPECT_DOUBLE_EQ(locations[2].y, 0.2);
  EXPECT_DOUBLE_EQ(locations[3].x, -0.3);
  EXPECT_DOUBLE_EQ(locations[3].y, -0.2);
}

TEST(DriverParametersTest, BuildsPerModuleMotorAndFeedforwardConfiguration)
{
  auto parameters = default_parameters();
  parameters.steering.zero_offset_rad[2] = 0.4;
  parameters.steering.inverted[2] = true;
  parameters.drive.inverted[2] = true;
  parameters.steering.max_ff_speed_radps = 0.75;
  parameters.steering.joint_limit_min_rad = -2.8;
  parameters.steering.joint_limit_max_rad = 2.9;
  parameters.steering.joint_limit_margin_rad = 0.1;
  parameters.steering.joint_limit_tolerance_rad = 0.02;

  const auto steering = steering_module_config(parameters, 2U);
  const auto drive = drive_module_config(parameters, 2U);
  const auto steering_motor = steering_motor_config(parameters, 2U);
  const auto drive_motor = drive_motor_config(parameters, 2U);

  EXPECT_DOUBLE_EQ(steering.zero_offset_rad, 0.4);
  EXPECT_TRUE(steering.inverted);
  EXPECT_DOUBLE_EQ(steering.max_ff_speed_radps, 0.75);
  EXPECT_DOUBLE_EQ(steering.angle_limits.minimum_rad, -2.8);
  EXPECT_DOUBLE_EQ(steering.angle_limits.maximum_rad, 2.9);
  EXPECT_DOUBLE_EQ(steering.angle_limits.margin_rad, 0.1);
  EXPECT_DOUBLE_EQ(steering.angle_limits.measurement_tolerance_rad, 0.02);
  EXPECT_TRUE(drive.inverted);
  EXPECT_DOUBLE_EQ(drive.wheel_radius_m, parameters.chassis.wheel_radius_m);
  EXPECT_EQ(steering_motor.esc_id, 3U);
  EXPECT_EQ(steering_motor.mst_id, 0x13U);
  EXPECT_EQ(drive_motor.esc_id, 7U);
  EXPECT_EQ(drive_motor.mst_id, 0x17U);
}

TEST(DriverParametersTest, RejectsInvalidGeometryTimingGainsAndIdentifiers)
{
  auto parameters = default_parameters();
  parameters.chassis.wheel_radius_m = 0.0;
  parameters.control.rate_hz = 0.0;
  parameters.can.write_timeout_us = 0;
  parameters.steering.max_ff_speed_radps = -1.0;
  parameters.odometry.publish_rate_hz = 101.0;
  parameters.motors.drive_mst_id[3] = parameters.motors.steering_mst_id[0];
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "wheel_radius_m"));
  EXPECT_TRUE(contains_error(errors, "rate_hz"));
  EXPECT_TRUE(contains_error(errors, "write_timeout_us"));
  EXPECT_TRUE(contains_error(errors, "max_ff_speed_radps"));
  EXPECT_TRUE(contains_error(errors, "publish_rate_hz"));
  EXPECT_TRUE(contains_error(errors, "MST_ID"));
  EXPECT_THROW(validate_parameters(parameters), std::invalid_argument);
}

TEST(DriverParametersTest, RejectsOutOfRangeModuleIndex)
{
  const auto parameters = default_parameters();
  EXPECT_THROW(
    static_cast<void>(steering_module_config(parameters, kSwerveModuleCount)),
    std::out_of_range);
  EXPECT_THROW(
    static_cast<void>(drive_motor_config(parameters, kSwerveModuleCount)),
    std::out_of_range);
}

TEST(DriverParametersTest, RejectsCanIdOverlapAndTimeoutRegisterOverflow)
{
  auto parameters = default_parameters();
  parameters.motors.drive_mst_id[0] = parameters.motors.steering_esc_id[0];
  parameters.motors.steering_esc_id[1] = kRegisterCanId;
  parameters.can.timeout_register_ms = std::numeric_limits<std::int64_t>::max();
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "overlap"));
  EXPECT_TRUE(contains_error(errors, "0x7FF"));
  EXPECT_TRUE(contains_error(errors, "timeout_register_ms"));
}

TEST(DriverParametersTest, RejectsInvalidPhaseSevenControlQualityLimits)
{
  auto parameters = default_parameters();
  parameters.steering.flip_hysteresis_rad = -0.1;
  parameters.steering.max_slew_radps = 0.0;
  parameters.odometry.max_imu_yaw_step_rad = kPi + 0.1;
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "flip_hysteresis_rad"));
  EXPECT_TRUE(contains_error(errors, "max_slew_radps"));
  EXPECT_TRUE(contains_error(errors, "max_imu_yaw_step_rad"));
}

TEST(DriverParametersTest, RejectsInvalidBoundedSteeringRange)
{
  auto parameters = default_parameters();
  parameters.steering.joint_limit_min_rad = -0.5;
  parameters.steering.joint_limit_max_rad = 0.5;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));

  parameters.steering.joint_limit_min_rad = -kPi - 0.1;
  parameters.steering.joint_limit_max_rad = kPi + 0.1;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));

  parameters.steering.joint_limit_min_rad = 0.1;
  parameters.steering.joint_limit_max_rad = kPi;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "contain zero"));

  parameters = default_parameters();
  parameters.steering.joint_limit_margin_rad = kPi / 2.0 + 0.01;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));

  parameters = default_parameters();
  parameters.steering.joint_limit_tolerance_rad = -0.01;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "joint_limit_tolerance_rad"));
}

TEST(DriverParametersTest, RejectsInvalidOdometryQualityAndRezeroParameters)
{
  auto parameters = default_parameters();
  parameters.steering.rezero_tolerance_rad = 0.0;
  parameters.odometry.pose_covariance_diagonal[1] = -1.0;
  parameters.odometry.twist_covariance_diagonal[2] =
    std::numeric_limits<double>::infinity();
  parameters.odometry.imu_fallback_covariance_scale = 0.5;
  parameters.odometry.missing_module_covariance_scale = 0.0;
  parameters.odometry.slip_residual_threshold = 0.0;
  parameters.odometry.slip_covariance_scale = 0.5;
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "rezero_tolerance_rad"));
  EXPECT_TRUE(contains_error(errors, "pose_covariance_diagonal"));
  EXPECT_TRUE(contains_error(errors, "twist_covariance_diagonal"));
  EXPECT_TRUE(contains_error(errors, "imu_fallback_covariance_scale"));
  EXPECT_TRUE(contains_error(errors, "missing_module_covariance_scale"));
  EXPECT_TRUE(contains_error(errors, "slip_residual_threshold"));
  EXPECT_TRUE(contains_error(errors, "slip_covariance_scale"));
}

TEST(DriverParametersTest, RejectsUnsafeAutomaticRecoveryConfiguration)
{
  auto parameters = default_parameters();
  parameters.safety.reenable_period_s = 0.5;
  parameters.safety.auto_recovery_limit = 0U;
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "reenable_period_s"));
  EXPECT_TRUE(contains_error(errors, "auto_recovery_limit"));
}

}  // namespace
}  // namespace dm_swerve_driver
