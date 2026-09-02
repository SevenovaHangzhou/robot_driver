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
  EXPECT_DOUBLE_EQ(parameters.steering.max_ff_speed_radps, 3.0);
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

  const auto steering = steering_module_config(parameters, 2U);
  const auto drive = drive_module_config(parameters, 2U);
  const auto steering_motor = steering_motor_config(parameters, 2U);
  const auto drive_motor = drive_motor_config(parameters, 2U);

  EXPECT_DOUBLE_EQ(steering.zero_offset_rad, 0.4);
  EXPECT_TRUE(steering.inverted);
  EXPECT_DOUBLE_EQ(steering.max_ff_speed_radps, 0.75);
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
  parameters.steering.max_ff_speed_radps = -1.0;
  parameters.odometry.publish_rate_hz = 101.0;
  parameters.motors.drive_mst_id[3] = parameters.motors.steering_mst_id[0];
  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "wheel_radius_m"));
  EXPECT_TRUE(contains_error(errors, "rate_hz"));
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

}  // namespace
}  // namespace dm_swerve_driver
