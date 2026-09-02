#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "dm_swerve_driver/bringup_audit.hpp"

namespace dm_swerve_driver {
namespace {

TEST(BringupAuditTest, DefaultsCoverEightMotorsAndFourSteeringAxes)
{
  const auto options = default_bringup_audit_options();
  ASSERT_EQ(options.motors.size(), 8U);
  EXPECT_EQ(options.interface_name, "can0");
  for (std::size_t index{0U}; index < options.motors.size(); ++index) {
    EXPECT_EQ(options.motors[index].esc_id, index + 1U);
    EXPECT_EQ(options.motors[index].mst_id, 0x11U + index);
    EXPECT_EQ(options.motors[index].steering, index < 4U);
  }
}

TEST(BringupAuditTest, ParsesSingleModuleHexIdsAndExpectedLimits)
{
  const auto options = parse_bringup_audit_arguments({
      "--interface", "vcan0",
      "--steering", "0x1:0x11",
      "--motor", "0x5:0x15",
      "--pmax", "20.0", "--vmax", "50.0", "--tmax", "12.0",
      "--deadline-us", "4000", "--retries", "2"});

  ASSERT_EQ(options.motors.size(), 2U);
  EXPECT_TRUE(options.motors[0].steering);
  EXPECT_FALSE(options.motors[1].steering);
  EXPECT_EQ(options.motors[1].esc_id, 5U);
  EXPECT_DOUBLE_EQ(options.expected_limits.position_max, 20.0);
  EXPECT_EQ(options.feedback_deadline.count(), 4000);
  EXPECT_EQ(options.retries, 2);
}

TEST(BringupAuditTest, RejectsDuplicateAndInvalidIdentifiers)
{
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({
        "--motor", "1:17", "--motor", "2:17"})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--motor", "0x800:1"})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--motor", "bad"})),
    std::invalid_argument);
}

TEST(BringupAuditTest, RejectsInvalidTimingAndUnknownOptions)
{
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--retries", "0"})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--deadline-us", "-1"})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--unknown"})),
    std::invalid_argument);
  EXPECT_FALSE(bringup_audit_usage().empty());
}

TEST(BringupAuditTest, RejectsEscAndMstIdentifierOverlap)
{
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({
        "--motor", "1:17", "--motor", "17:18"})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(parse_bringup_audit_arguments({"--motor", "0x7ff:17"})),
    std::invalid_argument);
}

}  // namespace
}  // namespace dm_swerve_driver
