#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "dm_swerve_driver/kinco_units.hpp"

namespace dm_swerve_driver {
namespace {

constexpr std::uint32_t kResolution{10000U};
constexpr double kTau{6.283185307179586476925286766559};

TEST(KincoUnitsTest, ConvertsMotorPositionRadiansToSignedIncrements)
{
  EXPECT_EQ(kinco_position_from_radians(kTau, kResolution), 10000);
  EXPECT_EQ(kinco_position_from_radians(-kTau / 4.0, kResolution), -2500);
  EXPECT_NEAR(kinco_position_to_radians(2500, kResolution), kTau / 4.0, 1e-12);
}

TEST(KincoUnitsTest, AppliesDocumentedVelocityDecFormula)
{
  const auto raw = kinco_velocity_from_radians_per_second(kTau, kResolution);
  EXPECT_EQ(raw, 163840);
  EXPECT_NEAR(kinco_velocity_to_radians_per_second(raw, kResolution), kTau, 1e-12);
}

TEST(KincoUnitsTest, ConvertsCstRatedTorqueRatioToIntegerPercent)
{
  EXPECT_EQ(kinco_torque_percent_from_ratio(1.0), 100);
  EXPECT_EQ(kinco_torque_percent_from_ratio(-0.5), -50);
  EXPECT_DOUBLE_EQ(kinco_torque_ratio_from_percent(250), 2.5);
}

TEST(KincoUnitsTest, RejectsInvalidResolutionNonfiniteAndRawOverflow)
{
  EXPECT_THROW(
    static_cast<void>(kinco_position_from_radians(1.0, 0U)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(kinco_velocity_from_radians_per_second(
        std::numeric_limits<double>::infinity(), kResolution)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(kinco_position_from_radians(1e12, kResolution)),
    std::out_of_range);
  EXPECT_THROW(
    static_cast<void>(kinco_torque_percent_from_ratio(1000.0)),
    std::out_of_range);
}

}  // namespace
}  // namespace dm_swerve_driver
