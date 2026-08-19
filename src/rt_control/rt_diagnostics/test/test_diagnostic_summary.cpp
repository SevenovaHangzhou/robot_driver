#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rt_diagnostics/diagnostic_summary.hpp"

namespace rt_diagnostics
{
namespace
{

struct UnhealthyLevel
{
  std::uint8_t level;
  std::string message;
};

class UnhealthyDiagnosticSummaryTest : public ::testing::TestWithParam<UnhealthyLevel>
{};

TEST_P(UnhealthyDiagnosticSummaryTest, CarriesWorstSourceNameAndOriginalMessage)
{
  DiagnosticSummaryAccumulator summary{"EtherCAT topology healthy"};
  summary.observe(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "/robot/rt_control/ethercat/master", "EtherCAT master healthy");
  summary.observe(
    GetParam().level, "/robot/rt_control/ethercat/slave_7", GetParam().message);

  const auto result = summary.result();

  EXPECT_EQ(result.level, GetParam().level);
  EXPECT_EQ(
    result.message,
    "/robot/rt_control/ethercat/slave_7: " + GetParam().message);
}

INSTANTIATE_TEST_SUITE_P(
  WarnErrorAndStale, UnhealthyDiagnosticSummaryTest,
  ::testing::Values(
    UnhealthyLevel{
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "EtherCAT slave is not in the expected state"},
    UnhealthyLevel{
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "CiA402 fault"},
    UnhealthyLevel{
      diagnostic_msgs::msg::DiagnosticStatus::STALE,
      "EtherCAT slave snapshot is stale"}));

TEST(DiagnosticSummaryTest, PreservesExistingHealthyMessages)
{
  DiagnosticSummaryAccumulator ethercat{"EtherCAT topology healthy"};
  ethercat.observe(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "/robot/rt_control/ethercat/master", "EtherCAT master healthy");
  DiagnosticSummaryAccumulator canopen{"CANopen topology healthy"};
  canopen.observe(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "/robot/rt_control/canopen/node_2", "CANopen node healthy");

  EXPECT_EQ(ethercat.result().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(ethercat.result().message, "EtherCAT topology healthy");
  EXPECT_EQ(canopen.result().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(canopen.result().message, "CANopen topology healthy");
}

TEST(DiagnosticSummaryTest, SelectsTheHighestSeverity)
{
  DiagnosticSummaryAccumulator summary{"CANopen topology healthy"};
  summary.observe(
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    "/robot/rt_control/canopen/node_2", "CANopen node degraded");
  summary.observe(
    diagnostic_msgs::msg::DiagnosticStatus::STALE,
    "/robot/rt_control/canopen/node_3", "ros2_canopen native diagnostics are stale");
  summary.observe(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "/robot/rt_control/canopen/node_4", "CANopen node fault");

  const auto result = summary.result();

  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
  EXPECT_EQ(
    result.message,
    "/robot/rt_control/canopen/node_3: ros2_canopen native diagnostics are stale");
}

TEST(DiagnosticSummaryTest, BreaksEqualSeverityTiesBySourceName)
{
  DiagnosticSummaryAccumulator forward{"CANopen topology healthy"};
  forward.observe(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "/robot/rt_control/canopen/node_9", "later source fault");
  forward.observe(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "/robot/rt_control/canopen/node_2", "earlier source fault");

  DiagnosticSummaryAccumulator reverse{"CANopen topology healthy"};
  reverse.observe(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "/robot/rt_control/canopen/node_2", "earlier source fault");
  reverse.observe(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
    "/robot/rt_control/canopen/node_9", "later source fault");

  EXPECT_EQ(forward.result().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    forward.result().message,
    "/robot/rt_control/canopen/node_2: earlier source fault");
  EXPECT_EQ(reverse.result().message, forward.result().message);
}

}  // namespace
}  // namespace rt_diagnostics
