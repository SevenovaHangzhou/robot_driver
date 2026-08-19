#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "rt_diagnostics/ethercat_master_diagnostic.hpp"

namespace rt_diagnostics
{
namespace
{

constexpr std::uint8_t kOk{0U};
constexpr std::uint8_t kWarn{1U};
constexpr std::uint8_t kError{2U};
constexpr std::uint8_t kStale{3U};

EthercatMasterDiagnosticSample makeHealthySample()
{
  return EthercatMasterDiagnosticSample{10.0, 1.0, 16.0, 705.0};
}

TEST(EthercatMasterDiagnosticTest, AcceptsAValidSnapshotAndHistoricalCounterBaseline)
{
  const auto result = evaluateEthercatMasterDiagnostic(
    makeHealthySample(), false, 16U, std::nullopt);

  EXPECT_EQ(result.level, kOk);
  EXPECT_EQ(result.message, "EtherCAT master healthy");
  EXPECT_TRUE(result.values_valid);
  EXPECT_EQ(result.slaves_responding, 16U);
  EXPECT_DOUBLE_EQ(result.working_counter_error_count, 705.0);
}

TEST(EthercatMasterDiagnosticTest, PreservesWarnOnlyWhileWorkingCounterIncreases)
{
  const auto unchanged = evaluateEthercatMasterDiagnostic(
    makeHealthySample(), false, 16U, 705.0);
  auto increased_sample = makeHealthySample();
  increased_sample.working_counter_error_count = 706.0;
  const auto increased = evaluateEthercatMasterDiagnostic(
    increased_sample, false, 16U, 705.0);

  EXPECT_EQ(unchanged.level, kOk);
  EXPECT_EQ(increased.level, kWarn);
  EXPECT_EQ(
    increased.message,
    "EtherCAT responder count or working counter changed");
}

struct InvalidNumericCase
{
  const char * label;
  double EthercatMasterDiagnosticSample::* member;
  double value;
};

class InvalidEthercatMasterNumericTest
  : public ::testing::TestWithParam<InvalidNumericCase>
{};

TEST_P(InvalidEthercatMasterNumericTest, FailsStaleInsteadOfAcceptingInvalidNumbers)
{
  auto sample = makeHealthySample();
  sample.*(GetParam().member) = GetParam().value;

  const auto result = evaluateEthercatMasterDiagnostic(
    sample, false, 16U, std::nullopt);

  EXPECT_EQ(result.level, kStale) << GetParam().label;
  EXPECT_FALSE(result.values_valid) << GetParam().label;
  EXPECT_NE(result.message.find(GetParam().label), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidDomains, InvalidEthercatMasterNumericTest,
  ::testing::Values(
    InvalidNumericCase{"process_data_age_ms", &EthercatMasterDiagnosticSample::process_data_age_ms,
      -1.0},
    InvalidNumericCase{
      "process_data_age_ms", &EthercatMasterDiagnosticSample::process_data_age_ms,
      std::numeric_limits<double>::quiet_NaN()},
    InvalidNumericCase{"link_up", &EthercatMasterDiagnosticSample::link_up, 2.0},
    InvalidNumericCase{"link_up", &EthercatMasterDiagnosticSample::link_up, 0.5},
    InvalidNumericCase{"slaves_responding", &EthercatMasterDiagnosticSample::slaves_responding,
      -1.0},
    InvalidNumericCase{"slaves_responding", &EthercatMasterDiagnosticSample::slaves_responding,
      16.5},
    InvalidNumericCase{"wc_error_count",
      &EthercatMasterDiagnosticSample::working_counter_error_count, -1.0},
    InvalidNumericCase{"wc_error_count",
      &EthercatMasterDiagnosticSample::working_counter_error_count, 1.5}));

TEST(EthercatMasterDiagnosticTest, DistinguishesStaleLinkDownAndResponderMismatch)
{
  const auto topic_stale = evaluateEthercatMasterDiagnostic(
    makeHealthySample(), true, 16U, std::nullopt);
  auto old_sample = makeHealthySample();
  old_sample.process_data_age_ms = 3000.1;
  const auto process_stale = evaluateEthercatMasterDiagnostic(
    old_sample, false, 16U, std::nullopt);
  auto link_down_sample = makeHealthySample();
  link_down_sample.link_up = 0.0;
  const auto link_down = evaluateEthercatMasterDiagnostic(
    link_down_sample, false, 16U, std::nullopt);
  auto responder_sample = makeHealthySample();
  responder_sample.slaves_responding = 15.0;
  const auto responder_mismatch = evaluateEthercatMasterDiagnostic(
    responder_sample, false, 16U, std::nullopt);

  EXPECT_EQ(topic_stale.level, kStale);
  EXPECT_EQ(process_stale.level, kStale);
  EXPECT_EQ(link_down.level, kError);
  EXPECT_EQ(responder_mismatch.level, kWarn);
}

}  // namespace
}  // namespace rt_diagnostics
