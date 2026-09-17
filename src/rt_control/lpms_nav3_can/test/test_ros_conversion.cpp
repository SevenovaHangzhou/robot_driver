#include "lpms_nav3_can/ros_conversion.hpp"

#include <gtest/gtest.h>

TEST(RosConversionTest, MapsDecodedSampleToSensorMessages)
{
  lpms_nav3_can::ImuSample sample;
  sample.linear_acceleration_mps2 = {1.0, -2.0, 3.0};
  sample.angular_velocity_radps = {4.0, -5.0, 6.0};
  sample.magnetic_field_t = {7.0e-6, -8.0e-6, 9.0e-6};
  sample.orientation_wxyz = {0.5, 0.1, -0.2, 0.3};

  const auto imu = lpms_nav3_can::make_imu_message(sample);
  const auto magnetic_field = lpms_nav3_can::make_magnetic_field_message(sample);

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
  EXPECT_DOUBLE_EQ(magnetic_field.magnetic_field.x, 7.0e-6);
  EXPECT_DOUBLE_EQ(magnetic_field.magnetic_field.y, -8.0e-6);
  EXPECT_DOUBLE_EQ(magnetic_field.magnetic_field.z, 9.0e-6);
}

TEST(RosConversionTest, LeavesUnknownCovariancesAtRosDefault)
{
  const lpms_nav3_can::ImuSample sample;

  const auto imu = lpms_nav3_can::make_imu_message(sample);
  const auto magnetic_field = lpms_nav3_can::make_magnetic_field_message(sample);

  for (const auto value : imu.orientation_covariance) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
  for (const auto value : imu.angular_velocity_covariance) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
  for (const auto value : imu.linear_acceleration_covariance) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
  for (const auto value : magnetic_field.magnetic_field_covariance) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}
