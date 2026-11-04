#include "lpms_nav3_can/ros_conversion.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(RosConversionTest, MapsAlignedGyroToImuMessage)
{
  lpms_nav3_can::ImuSample sample;
  sample.linear_acceleration_mps2 = {1.0, -2.0, 3.0};
  sample.angular_velocity_radps = {4.0, -5.0, 6.0};
  sample.bias_calibrated_angular_velocity_radps = {7.0, -8.0, 9.0};
  sample.orientation_wxyz = {0.5, 0.1, -0.2, 0.3};

  const auto imu = lpms_nav3_can::make_imu_message(sample, {});

  EXPECT_DOUBLE_EQ(imu.orientation.w, 0.5);
  EXPECT_DOUBLE_EQ(imu.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(imu.orientation.y, -0.2);
  EXPECT_DOUBLE_EQ(imu.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.y, -5.0);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 6.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.y, -2.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 3.0);
}

TEST(RosConversionTest, CopiesConfiguredCovariances)
{
  const lpms_nav3_can::ImuSample sample;
  lpms_nav3_can::ImuCovariances covariances;
  covariances.orientation = {1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0};
  covariances.angular_velocity = {4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0};
  covariances.linear_acceleration = {7.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 9.0};

  const auto imu = lpms_nav3_can::make_imu_message(sample, covariances);

  EXPECT_EQ(imu.orientation_covariance, covariances.orientation);
  EXPECT_EQ(imu.angular_velocity_covariance, covariances.angular_velocity);
  EXPECT_EQ(imu.linear_acceleration_covariance, covariances.linear_acceleration);
}

TEST(RosConversionTest, RotatesSensorDataOrientationAndCovariancesIntoRobotAxes)
{
  constexpr double kPi = 3.14159265358979323846;
  lpms_nav3_can::ImuSample sample;
  sample.linear_acceleration_mps2 = {1.0, 2.0, 3.0};
  sample.angular_velocity_radps = {4.0, 5.0, 6.0};
  sample.orientation_wxyz = {std::cos(kPi / 4.0), 0.0, 0.0, std::sin(kPi / 4.0)};
  lpms_nav3_can::ImuCovariances covariances;
  covariances.orientation = {1.0, 2.0, 3.0, 2.0, 4.0, 5.0, 3.0, 5.0, 6.0};
  covariances.angular_velocity = {1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0};

  const auto imu = lpms_nav3_can::make_imu_message(sample, covariances, kPi / 2.0);

  EXPECT_NEAR(imu.linear_acceleration.x, -2.0, 1.0e-12);
  EXPECT_NEAR(imu.linear_acceleration.y, 1.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 3.0);
  EXPECT_NEAR(imu.angular_velocity.x, -5.0, 1.0e-12);
  EXPECT_NEAR(imu.angular_velocity.y, 4.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 6.0);
  EXPECT_NEAR(imu.orientation.w, 1.0, 1.0e-12);
  EXPECT_NEAR(imu.orientation.x, 0.0, 1.0e-12);
  EXPECT_NEAR(imu.orientation.y, 0.0, 1.0e-12);
  EXPECT_NEAR(imu.orientation.z, 0.0, 1.0e-12);
  const std::array<double, 9> expected_orientation = {4.0, -2.0, -5.0, -2.0, 1.0,
                                                      3.0, -5.0, 3.0,  6.0};
  const std::array<double, 9> expected_angular_velocity = {4.0, 0.0, 0.0, 0.0, 1.0,
                                                           0.0, 0.0, 0.0, 9.0};
  for (std::size_t index = 0; index < 9; ++index) {
    EXPECT_NEAR(imu.orientation_covariance[index], expected_orientation[index], 1.0e-12);
    EXPECT_NEAR(imu.angular_velocity_covariance[index], expected_angular_velocity[index], 1.0e-12);
    EXPECT_DOUBLE_EQ(imu.linear_acceleration_covariance[index], 0.0);
  }
}
