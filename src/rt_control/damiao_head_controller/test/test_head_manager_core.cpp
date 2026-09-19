#include <gtest/gtest.h>

#include <cstdint>

#include "damiao_head_controller/head_manager_core.hpp"

namespace damiao_head_controller
{
namespace
{

TEST(HeadManagerCoreTest, EnableCompletesOnlyAfterHardwareAllowsMotion)
{
  EXPECT_EQ(
    assess_enable(HeadSnapshot{HeadPhase::enabling, false, false, 0U}),
    OperationResult::pending);
  EXPECT_EQ(
    assess_enable(HeadSnapshot{HeadPhase::enabled, false, true, 0U}),
    OperationResult::success);
  EXPECT_EQ(
    assess_enable(HeadSnapshot{HeadPhase::fault_latched, true, false, 0U}),
    OperationResult::fault);
}

TEST(HeadManagerCoreTest, DisableAndResetPreserveTheFaultLatchContract)
{
  EXPECT_EQ(
    assess_disable(HeadSnapshot{HeadPhase::disabled, false, false, 0U}),
    OperationResult::success);
  EXPECT_EQ(
    assess_disable(HeadSnapshot{HeadPhase::fault_latched, true, false, 0U}),
    OperationResult::fault);
  EXPECT_EQ(
    assess_reset(HeadSnapshot{HeadPhase::resetting, true, false, 0U}, 1U),
    OperationResult::pending);
  EXPECT_EQ(
    assess_reset(HeadSnapshot{HeadPhase::disabled, false, false, 1U}, 1U),
    OperationResult::success);
}

TEST(HeadManagerCoreTest, DiagnosticsExposeTransitionsAndLatchedFaults)
{
  const auto disabled = assess_diagnostics(
    HeadSnapshot{HeadPhase::disabled, false, false, 0U}, false);
  EXPECT_EQ(disabled.level, DiagnosticLevel::ok);
  EXPECT_STREQ(disabled.message, "disabled");

  const auto enabling = assess_diagnostics(
    HeadSnapshot{HeadPhase::enabling, false, false, 0U}, false);
  EXPECT_EQ(enabling.level, DiagnosticLevel::warn);
  EXPECT_STREQ(enabling.message, "enabling");

  const auto fault = assess_diagnostics(
    HeadSnapshot{HeadPhase::fault_latched, true, false, 0U}, false);
  EXPECT_EQ(fault.level, DiagnosticLevel::error);
  EXPECT_STREQ(fault.message, "fault latched");

  const auto stale = assess_diagnostics(
    HeadSnapshot{HeadPhase::enabled, false, true, 0U}, true);
  EXPECT_EQ(stale.level, DiagnosticLevel::stale);
  EXPECT_STREQ(stale.message, "feedback stale");
}

}  // namespace
}  // namespace damiao_head_controller
