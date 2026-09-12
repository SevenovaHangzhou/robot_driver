#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

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

TEST(DriverParametersTest, DefaultsAreValidForTheCommonControlLayer)
{
  const auto parameters = default_parameters();

  EXPECT_TRUE(parameter_errors(parameters).empty());
  EXPECT_NO_THROW(validate_parameters(parameters));
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_min_rad, -kPi);
  EXPECT_DOUBLE_EQ(parameters.steering.joint_limit_max_rad, kPi);
  EXPECT_DOUBLE_EQ(parameters.odometry.slip_residual_threshold, 0.25);
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

TEST(DriverParametersTest, RejectsInvalidGeometryAndTiming)
{
  auto parameters = default_parameters();
  parameters.control.rate_hz = 0.0;
  parameters.chassis.wheel_radius_m = 0.0;
  parameters.odometry.publish_rate_hz = 101.0;

  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "control.rate_hz"));
  EXPECT_TRUE(contains_error(errors, "wheel_radius_m"));
  EXPECT_TRUE(contains_error(errors, "publish_rate_hz"));
  EXPECT_THROW(validate_parameters(parameters), std::invalid_argument);
}

TEST(DriverParametersTest, RejectsInvalidBoundedSteeringRange)
{
  auto parameters = default_parameters();
  parameters.steering.joint_limit_min_rad = -0.5;
  parameters.steering.joint_limit_max_rad = 0.5;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));

  parameters = default_parameters();
  parameters.steering.joint_limit_margin_rad = kPi / 2.0 + 0.01;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));

  parameters = default_parameters();
  parameters.steering.joint_limit_tolerance_rad = -0.01;
  EXPECT_TRUE(contains_error(parameter_errors(parameters), "steering angle range"));
}

TEST(DriverParametersTest, RejectsUnsafeSafetyAndOdometryValues)
{
  auto parameters = default_parameters();
  parameters.safety.feedback_silent_cycles = 0U;
  parameters.safety.auto_recovery_limit = 0U;
  parameters.odometry.pose_covariance_diagonal[0] = -1.0;
  parameters.odometry.slip_residual_threshold = 0.0;
  parameters.odometry.slip_covariance_scale =
    std::numeric_limits<double>::infinity();

  const auto errors = parameter_errors(parameters);

  EXPECT_TRUE(contains_error(errors, "feedback_silent_cycles"));
  EXPECT_TRUE(contains_error(errors, "auto_recovery_limit"));
  EXPECT_TRUE(contains_error(errors, "pose_covariance_diagonal"));
  EXPECT_TRUE(contains_error(errors, "slip_residual_threshold"));
  EXPECT_TRUE(contains_error(errors, "slip_covariance_scale"));
}

}  // namespace
}  // namespace dm_swerve_driver
