#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rt_diagnostics/canopen_native_diagnostic.hpp"

namespace rt_diagnostics
{
namespace
{

constexpr std::uint8_t kOk{0U};
constexpr std::uint8_t kWarn{1U};
constexpr std::uint8_t kError{2U};
constexpr std::uint8_t kStale{3U};

NativeCanopenDiagnostic makeOperationalDiagnostic()
{
  return NativeCanopenDiagnostic{
    kOk,
    "Operation enabled",
    {
      {"device_state", "Added to master."},
      {"nmt_state", "START"},
      {"emcy_state", ""},
      {"cia402_mode", "Mode switched to: 3"},
      {"cia402_state", "State switched to: 5"},
    }};
}

TEST(CanopenNativeDiagnosticTest, AcceptsOnlyThePinnedUpstreamOperationalStrings)
{
  const auto source = makeOperationalDiagnostic();

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kOk);
  EXPECT_EQ(result.message, source.message);
  EXPECT_EQ(source.values[1].value, "START");
  EXPECT_EQ(source.values[2].value, "");
  EXPECT_EQ(source.values[4].value, "State switched to: 5");
}

struct MissingFieldCase
{
  std::string field;
};

class MissingNativeCanopenFieldTest : public ::testing::TestWithParam<MissingFieldCase>
{};

TEST_P(MissingNativeCanopenFieldTest, FailsStaleInsteadOfTrustingAnIncompleteFrame)
{
  auto source = makeOperationalDiagnostic();
  const auto field = GetParam().field;
  source.values.erase(
    std::remove_if(
      source.values.begin(), source.values.end(),
      [&field](const auto & value) {return value.key == field;}),
    source.values.end());

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kStale);
  EXPECT_NE(result.message.find(field), std::string::npos);
  EXPECT_NE(result.message.find(source.message), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  AllPinnedNativeFields, MissingNativeCanopenFieldTest,
  ::testing::Values(
    MissingFieldCase{"device_state"},
    MissingFieldCase{"nmt_state"},
    MissingFieldCase{"emcy_state"},
    MissingFieldCase{"cia402_mode"},
    MissingFieldCase{"cia402_state"}));

class DuplicateNativeCanopenFieldTest : public ::testing::TestWithParam<MissingFieldCase>
{};

TEST_P(DuplicateNativeCanopenFieldTest, FailsStaleInsteadOfSelectingOneAmbiguousValue)
{
  auto source = makeOperationalDiagnostic();
  const auto field = GetParam().field;
  source.values.push_back({field, "conflicting duplicate"});

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kStale);
  EXPECT_NE(result.message.find(field), std::string::npos);
  EXPECT_NE(result.message.find("duplicated"), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  AllPinnedNativeFields, DuplicateNativeCanopenFieldTest,
  ::testing::Values(
    MissingFieldCase{"device_state"},
    MissingFieldCase{"nmt_state"},
    MissingFieldCase{"emcy_state"},
    MissingFieldCase{"cia402_mode"},
    MissingFieldCase{"cia402_state"}));

struct FieldStateCase
{
  std::string value;
  std::uint8_t expected_level;
};

class NonOperationalNmtTest : public ::testing::TestWithParam<FieldStateCase>
{};

TEST_P(NonOperationalNmtTest, NeverTreatsAnUpstreamNonStartStateAsOperational)
{
  auto source = makeOperationalDiagnostic();
  source.values[1].value = GetParam().value;

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, GetParam().expected_level);
  EXPECT_NE(result.message.find(GetParam().value), std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  PinnedUpstreamNmtStrings, NonOperationalNmtTest,
  ::testing::Values(
    FieldStateCase{"BOOTUP", kWarn},
    FieldStateCase{"PREOP", kWarn},
    FieldStateCase{"RESET_COMM", kWarn},
    FieldStateCase{"RESET_NODE", kWarn},
    FieldStateCase{"STOP", kWarn},
    FieldStateCase{"TOGGLE", kWarn},
    FieldStateCase{"ERROR", kError},
    FieldStateCase{"OPERATIONAL", kError},
    FieldStateCase{"", kStale}));

class NonOperationalCia402SummaryTest : public ::testing::TestWithParam<FieldStateCase>
{};

TEST_P(NonOperationalCia402SummaryTest, NeverTrustsOkLevelWithoutTheExactCurrentStateMessage)
{
  auto source = makeOperationalDiagnostic();
  source.message = GetParam().value;

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, GetParam().expected_level);
  if (!GetParam().value.empty()) {
    EXPECT_NE(result.message.find(GetParam().value), std::string::npos);
  }
}

INSTANTIATE_TEST_SUITE_P(
  PinnedUpstreamCia402Messages, NonOperationalCia402SummaryTest,
  ::testing::Values(
    FieldStateCase{"Not ready to switch on", kWarn},
    FieldStateCase{"Switch on disabled", kWarn},
    FieldStateCase{"Ready to switch on", kWarn},
    FieldStateCase{"Switched on", kWarn},
    FieldStateCase{"Quick stop active", kWarn},
    FieldStateCase{"Warning bit is set", kWarn},
    FieldStateCase{"Internal limit active", kWarn},
    FieldStateCase{"Fault", kError},
    FieldStateCase{"Fault reaction active", kError},
    FieldStateCase{"Unknown state", kError},
    FieldStateCase{"OperationEnabled", kError},
    FieldStateCase{"", kStale}));

TEST(CanopenNativeDiagnosticTest, DoesNotTreatTransitionHistoryAsTheCurrentCia402State)
{
  auto no_transition_history = makeOperationalDiagnostic();
  no_transition_history.values[4].value.clear();
  auto old_transition_history = makeOperationalDiagnostic();
  old_transition_history.values[4].value = "State switched to: 4";

  EXPECT_EQ(evaluateNativeCanopenDiagnostic(no_transition_history).level, kOk);
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(old_transition_history).level, kOk);
}

TEST(CanopenNativeDiagnosticTest, TreatsANonzeroEmcyPayloadAsAnError)
{
  auto source = makeOperationalDiagnostic();
  source.values[2].value = "Emergency message: eec: 4096 er: 1 msef: 2 3 4 5 6 ";

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kError);
  EXPECT_NE(result.message.find(source.values[2].value), std::string::npos);
}

TEST(CanopenNativeDiagnosticTest, AcceptsThePinnedUpstreamZeroEmcyResetPayload)
{
  auto source = makeOperationalDiagnostic();
  source.values[2].value = "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 0 ";

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kOk);
  EXPECT_EQ(result.message, source.message);
}

TEST(CanopenNativeDiagnosticTest, RejectsMalformedZeroEmcyPayloads)
{
  const std::vector<std::string> malformed{
    "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 0",
    "Emergency message: eec: 00 er: 0 msef: 0 0 0 0 0 ",
    "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 ",
    "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 0 0 ",
    "Emergency message: eec: 0 er: 1 msef: 0 0 0 0 0 ",
    "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 1 ",
  };

  for (const auto & payload : malformed) {
    auto source = makeOperationalDiagnostic();
    source.values[2].value = payload;

    const auto result = evaluateNativeCanopenDiagnostic(source);

    EXPECT_EQ(result.level, kError) << "payload=" << payload;
  }
}

TEST(CanopenNativeDiagnosticTest, RejectsNonCanonicalDeviceAndModeValues)
{
  auto removed = makeOperationalDiagnostic();
  removed.values[0].value = "Removed from master.";
  auto timed_out = makeOperationalDiagnostic();
  timed_out.values[3].value = "Mode switch timed out: 3";
  auto empty_mode = makeOperationalDiagnostic();
  empty_mode.values[3].value.clear();

  EXPECT_EQ(evaluateNativeCanopenDiagnostic(removed).level, kError);
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(timed_out).level, kError);
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(empty_mode).level, kStale);
}

TEST(CanopenNativeDiagnosticTest, RejectsEveryCanonicalModeOtherThanProfiledVelocityThree)
{
  for (const auto mode : {1, 8, 9, 10}) {
    auto source = makeOperationalDiagnostic();
    source.values[3].value = "Mode switched to: " + std::to_string(mode);

    EXPECT_NE(evaluateNativeCanopenDiagnostic(source).level, kOk) << "mode=" << mode;
  }
}

TEST(CanopenNativeDiagnosticTest, PreservesAUpstreamNonOkLevelAndMessage)
{
  auto source = makeOperationalDiagnostic();
  source.level = kError;
  source.message = "Emergency message received";

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kError);
  EXPECT_EQ(result.message, source.message);
}

TEST(CanopenNativeDiagnosticTest, AnOperationalMessageDoesNotOverrideANonOkNativeLevel)
{
  auto source = makeOperationalDiagnostic();
  source.level = kWarn;

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kWarn);
  EXPECT_EQ(result.message, source.message);
}

TEST(CanopenNativeDiagnosticTest, ExtraNativeValuesRemainUntouched)
{
  auto source = makeOperationalDiagnostic();
  source.values.push_back({"vendor_detail", "raw payload"});
  const auto original_values = source.values;

  const auto result = evaluateNativeCanopenDiagnostic(source);

  EXPECT_EQ(result.level, kOk);
  EXPECT_EQ(source.values, original_values);
}

TEST(CanopenNativeDiagnosticIngestTest, AConfiguredMalformedFrameImmediatelyReplacesOldHealth)
{
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> configured{
    {"2", NativeCanopenDiagnosticSnapshot{}},
    {"3", NativeCanopenDiagnosticSnapshot{}},
  };
  const auto first_time = std::chrono::steady_clock::time_point{std::chrono::seconds{10}};
  const auto second_time = std::chrono::steady_clock::time_point{std::chrono::seconds{11}};
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", makeOperationalDiagnostic(), first_time));
  ASSERT_EQ(evaluateNativeCanopenDiagnostic(configured.at("2").diagnostic).level, kOk);

  auto malformed = makeOperationalDiagnostic();
  malformed.values.erase(malformed.values.begin() + 1);
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", std::move(malformed), second_time));

  EXPECT_EQ(configured.at("2").received, second_time);
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(configured.at("2").diagnostic).level, kStale);
}

TEST(CanopenNativeDiagnosticIngestTest, StopAndDuplicateFramesImmediatelyReplaceOldHealth)
{
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> configured{
    {"2", NativeCanopenDiagnosticSnapshot{}},
  };
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", makeOperationalDiagnostic(), std::chrono::steady_clock::time_point{}));

  auto stopped = makeOperationalDiagnostic();
  stopped.values[1].value = "STOP";
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", std::move(stopped), std::chrono::steady_clock::time_point{}));
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(configured.at("2").diagnostic).level, kWarn);

  auto duplicate = makeOperationalDiagnostic();
  duplicate.values.push_back({"cia402_state", "State switched to: 4"});
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", std::move(duplicate), std::chrono::steady_clock::time_point{}));
  EXPECT_EQ(evaluateNativeCanopenDiagnostic(configured.at("2").diagnostic).level, kStale);
}

TEST(CanopenNativeDiagnosticIngestTest, OnlyUnknownHardwareIdsAreIgnored)
{
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> configured{
    {"2", NativeCanopenDiagnosticSnapshot{}},
  };
  ASSERT_TRUE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "2", makeOperationalDiagnostic(), std::chrono::steady_clock::time_point{}));
  const auto accepted = configured.at("2").diagnostic;

  auto unknown = makeOperationalDiagnostic();
  unknown.values.clear();
  EXPECT_FALSE(
    ingestConfiguredNativeCanopenDiagnostic(
      configured, "99", std::move(unknown), std::chrono::steady_clock::time_point{}));
  EXPECT_EQ(configured.at("2").diagnostic, accepted);
}

}  // namespace
}  // namespace rt_diagnostics
