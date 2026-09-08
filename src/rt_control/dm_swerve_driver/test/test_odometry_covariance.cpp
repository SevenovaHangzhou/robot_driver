#include <gtest/gtest.h>

#include <stdexcept>

#include "dm_swerve_driver/odometry_covariance.hpp"

namespace dm_swerve_driver {
namespace {

TEST(OdometryCovarianceTest, PlacesHealthyBaselinesOnSixBySixDiagonal)
{
  OdometryParameters parameters;
  parameters.pose_covariance_diagonal = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  parameters.twist_covariance_diagonal = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};

  const auto covariance = make_odometry_covariances(parameters, false, 4U);

  for (std::size_t index{0U}; index < 6U; ++index) {
    EXPECT_DOUBLE_EQ(covariance.pose[index * 7U], static_cast<double>(index + 1U));
    EXPECT_DOUBLE_EQ(covariance.twist[index * 7U], static_cast<double>(index + 7U));
  }
  EXPECT_DOUBLE_EQ(covariance.pose[1], 0.0);
  EXPECT_DOUBLE_EQ(covariance.twist[34], 0.0);
}

TEST(OdometryCovarianceTest, MultipliesFallbackAndMissingModuleInflation)
{
  OdometryParameters parameters;
  parameters.pose_covariance_diagonal.fill(1.0);
  parameters.twist_covariance_diagonal.fill(2.0);
  parameters.imu_fallback_covariance_scale = 10.0;
  parameters.missing_module_covariance_scale = 4.0;

  const auto covariance = make_odometry_covariances(parameters, true, 3U);

  for (std::size_t index{0U}; index < 6U; ++index) {
    EXPECT_DOUBLE_EQ(covariance.pose[index * 7U], 40.0);
    EXPECT_DOUBLE_EQ(covariance.twist[index * 7U], 80.0);
  }
}

TEST(OdometryCovarianceTest, RejectsInvalidPublicInputs)
{
  OdometryParameters parameters;
  parameters.pose_covariance_diagonal[0] = -1.0;
  EXPECT_THROW(
    static_cast<void>(make_odometry_covariances(parameters, false, 4U)),
    std::invalid_argument);
  parameters = OdometryParameters{};
  EXPECT_THROW(
    static_cast<void>(make_odometry_covariances(parameters, false, kSwerveModuleCount + 1U)),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
