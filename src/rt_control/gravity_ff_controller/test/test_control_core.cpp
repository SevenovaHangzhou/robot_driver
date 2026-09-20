#include "gravity_ff_controller/control_core.hpp"
#include <gtest/gtest.h>
#include <limits>

using gravity_ff_controller::ControlCore;
using gravity_ff_controller::Limits;
using gravity_ff_controller::StepStatus;

class CoreTest : public testing::Test
{
protected:
  ControlCore core;
  std::vector<double> gravity{20.0, -30.0};
  std::vector<double> scale{1.0, 0.5};
  std::vector<double> status{64.0, 64.0};
  void SetUp() override
  {
    ASSERT_TRUE(core.configure({{10.0, 12.0}, {5.0, 6.0}, 20.0}));
  }
};

TEST_F(CoreTest, PreloadsWhileDisabledAndContinuesThroughEnableAndDisable) {
  EXPECT_EQ(core.step(gravity, scale, status, true, 1.0), StepStatus::kOk);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 5.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -6.0);
  status = {39.0, 39.0};
  core.step(gravity, scale, status, true, 1.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 10.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -12.0);
  gravity = {5.0, 4.0};
  status = {35.0, 35.0}; // Switched on during a normal disable window.
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 1.0),
    StepStatus::kDisableTransitionHold);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 10.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -12.0);
  status = {64.0, 64.0};
  core.step(gravity, scale, status, true, 1.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 5.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -6.0);
  EXPECT_FALSE(core.fault_latched());
}

TEST_F(CoreTest, PreloadsInEnableTransitionBeforeFirstOperationEnabled)
{
  status = {33.0, 35.0};
  EXPECT_EQ(core.step(gravity, scale, status, true, 1.0), StepStatus::kOk);
  EXPECT_NE(core.output_nm()[0], 0.0);
  EXPECT_FALSE(core.fault_latched());
}

TEST_F(CoreTest, FaultReactionHoldsLastOutputWithoutUpdating) {
  core.step(gravity, scale, status, true, 1.0);
  const auto before = core.output_nm();
  gravity = {-100.0, 100.0};
  status = {15.0, 15.0};
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 1.0),
    StepStatus::kFaultReactionHold);
  EXPECT_EQ(core.output_nm(), before);
  EXPECT_FALSE(core.fault_latched());
}

TEST_F(CoreTest, FaultReactionHoldsEvenWhenModelInputIsInvalid) {
  core.step(gravity, scale, status, true, 1.0);
  const auto before = core.output_nm();
  status = {15.0, 15.0};
  gravity[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    core.step(gravity, scale, status, false, 0.1),
    StepStatus::kFaultReactionHold);
  EXPECT_EQ(core.output_nm(), before);
}

TEST_F(CoreTest, FaultImmediatelyZerosAndLatchesUntilReactivated) {
  core.step(gravity, scale, status, true, 1.0);
  status = {8.0, 8.0};
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 1.0),
    StepStatus::kFaultLatched);
  EXPECT_EQ(core.output_nm(), std::vector<double>({0.0, 0.0}));
  status = {64.0, 64.0};
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 1.0),
    StepStatus::kAlreadyLatched);
  core.activate();
  EXPECT_EQ(core.step(gravity, scale, status, true, 1.0), StepStatus::kOk);
}

TEST_F(CoreTest, InvalidFeedbackWhileEnabledUsesFaultSlewThenStaysLatched) {
  status = {39.0, 39.0};
  core.step(gravity, scale, status, true, 1.0);
  status[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 0.1),
    StepStatus::kInvalidInputLatched);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 3.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -4.0);
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 0.5),
    StepStatus::kAlreadyLatched);
  EXPECT_EQ(core.output_nm(), std::vector<double>({0.0, 0.0}));
}

TEST_F(CoreTest, InvalidFeedbackWhileDisabledImmediatelyZeros) {
  core.step(gravity, scale, status, true, 1.0);
  gravity[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    core.step(gravity, scale, status, true, 0.1),
    StepStatus::kInvalidInputLatched);
  EXPECT_EQ(core.output_nm(), std::vector<double>({0.0, 0.0}));
}

TEST_F(CoreTest, ModelErrorFollowsEnabledFaultSlewRule) {
  status = {39.0, 39.0};
  core.step(gravity, scale, status, true, 1.0);
  core.step(gravity, scale, status, false, 0.25);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 0.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -1.0);
  EXPECT_TRUE(core.fault_latched());
}

TEST_F(CoreTest, LimitsAndNominalSlewAreAppliedPerAxis) {
  EXPECT_EQ(core.step(gravity, scale, status, true, 0.5), StepStatus::kOk);
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 2.5);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -3.0);
  for (int i = 0; i < 10; ++i) {
    core.step(gravity, scale, status, true, 0.5);
  }
  EXPECT_DOUBLE_EQ(core.output_nm()[0], 10.0);
  EXPECT_DOUBLE_EQ(core.output_nm()[1], -12.0);
}

TEST(ControlCoreValidation, RejectsInvalidLimitsAndInputs) {
  ControlCore core;
  EXPECT_FALSE(core.configure({{}, {}, 1.0}));
  EXPECT_FALSE(core.configure({{1.0}, {0.0}, 1.0}));
  EXPECT_FALSE(core.configure({{1.0}, {1.0}, 0.0}));
  ASSERT_TRUE(core.configure({{1.0}, {1.0}, 1.0}));
  EXPECT_EQ(
    core.step({1.0}, {1.1}, {64.0}, true, 0.1),
    StepStatus::kInvalidInputLatched);
}
