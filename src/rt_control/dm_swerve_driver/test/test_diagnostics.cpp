#include <gtest/gtest.h>

#include <string>

#include "dm_swerve_driver/diagnostics.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] ControlLoopStatus healthy_status()
{
  ControlLoopStatus status;
  status.initialized = true;
  status.command_timed_out = false;
  status.ethercat.domain.link_up = true;
  status.ethercat.domain.all_slaves_operational = true;
  status.ethercat.domain.expected_working_counter = 8U;
  status.ethercat.domain.working_counter = 8U;
  for (std::size_t index{0U}; index < kKincoAxisCount; ++index) {
    auto & axis = status.ethercat.feedback[index];
    axis.online = true;
    axis.status_word = 0x0027U;
    axis.mode_display =
      static_cast<std::int8_t>(index < kSwerveModuleCount ? 8 : 9);
  }
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    status.steering_sources[index].valid = true;
    status.steering_sources[index].source = SteeringAngleSource::external_encoder;
    status.encoder_heartbeat[index] = true;
  }
  return status;
}

TEST(DiagnosticsTest, HealthyEthercatAndExternalEncodersAreOk)
{
  const auto diagnostics = build_diagnostic_statuses(healthy_status());

  ASSERT_EQ(diagnostics.size(), 13U);
  for (const auto & status : diagnostics) {
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  }
}

TEST(DiagnosticsTest, EncoderFallbackIsAWarning)
{
  auto status = healthy_status();
  status.steering_sources[2].source = SteeringAngleSource::motor_backup;
  status.steering_sources[2].degraded = true;
  status.encoder_heartbeat[2] = false;

  const auto diagnostics = build_diagnostic_statuses(status);

  EXPECT_EQ(diagnostics[10].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(DiagnosticsTest, AxisOrLatchedFaultIsAnError)
{
  auto status = healthy_status();
  status.ethercat.feedback[5].error_word = 1U << 7U;
  status.faulted = true;
  status.fault_latched = true;

  const auto diagnostics = build_diagnostic_statuses(status);

  EXPECT_EQ(diagnostics[5].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(diagnostics.back().message.find("manual clear"), std::string::npos);
}

}  // namespace
}  // namespace dm_swerve_driver
