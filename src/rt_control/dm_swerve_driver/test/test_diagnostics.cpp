#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "dm_swerve_driver/diagnostics.hpp"

namespace dm_swerve_driver {
namespace {

TEST(DiagnosticsTest, PublishesEightMotorsAndHealthySummary)
{
  ControlLoopStatus status{};
  status.initialized = true;
  status.command_timed_out = false;
  for (auto & motor : status.motors) {
    motor.has_feedback = true;
    motor.error = MotorError::enabled;
  }
  const auto diagnostics = build_diagnostic_statuses(status, default_parameters());

  ASSERT_EQ(diagnostics.size(), kMotorCount + 1U);
  EXPECT_EQ(diagnostics.front().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST(DiagnosticsTest, ReportsFaultTemperatureAndSeedSourceWithoutAddingStopState)
{
  ControlLoopStatus status{};
  status.motors[0].has_feedback = true;
  status.motors[0].error = MotorError::mos_over_temperature;
  status.motors[0].mos_temperature_c = 95U;
  status.motors[0].seeded_from_multi_turn = true;
  const auto diagnostics = build_diagnostic_statuses(status, default_parameters());

  ASSERT_GE(diagnostics.size(), 1U);
  EXPECT_EQ(diagnostics[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  const auto has_temperature = std::any_of(
    diagnostics[0].values.begin(), diagnostics[0].values.end(),
    [](const auto & value) {
      return value.key == "mos_temperature_c" && value.value == "95";
    });
  const auto has_seed_source = std::any_of(
    diagnostics[0].values.begin(), diagnostics[0].values.end(),
    [](const auto & value) {
      return value.key == "seeded_from_multi_turn" && value.value == "true";
    });
  EXPECT_TRUE(has_temperature);
  EXPECT_TRUE(has_seed_source);
}

TEST(DiagnosticsTest, SummaryWarnsForRecoverableDegradationAndErrorsForSilentBus)
{
  ControlLoopStatus status{};
  status.command_timed_out = true;
  status.imu_fallback = true;
  auto diagnostics = build_diagnostic_statuses(status, default_parameters());
  ASSERT_FALSE(diagnostics.empty());
  EXPECT_EQ(diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  status.bus_silent = true;
  diagnostics = build_diagnostic_statuses(status, default_parameters());
  EXPECT_EQ(diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

}  // namespace
}  // namespace dm_swerve_driver
