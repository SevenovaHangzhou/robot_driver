#include <gtest/gtest.h>

#include <limits>
#include <sensor_msgs/msg/imu.hpp>

#include "dm_swerve_driver/imu_validation.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] sensor_msgs::msg::Imu valid_imu()
{
  sensor_msgs::msg::Imu message;
  message.orientation.w = 1.0;
  message.orientation_covariance[0] = 0.01;
  return message;
}

TEST(ImuValidationTest, AcceptsUnitQuaternionWithOrientationEstimate)
{
  EXPECT_TRUE(imu_orientation_valid(valid_imu()));
}

TEST(ImuValidationTest, RejectsMissingOrientationAndInvalidQuaternionNorm)
{
  auto missing = valid_imu();
  missing.orientation_covariance[0] = -1.0;
  EXPECT_FALSE(imu_orientation_valid(missing));

  auto zero = valid_imu();
  zero.orientation.w = 0.0;
  EXPECT_FALSE(imu_orientation_valid(zero));

  auto oversized = valid_imu();
  oversized.orientation.w = 1.2;
  EXPECT_FALSE(imu_orientation_valid(oversized));

  auto unknown = valid_imu();
  unknown.orientation_covariance[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(imu_orientation_valid(unknown));
}

}  // namespace
}  // namespace dm_swerve_driver
