#include <gtest/gtest.h>
#include "ctrl_swerve/stop_transitions.hpp"
using namespace ctrl_swerve;
TEST(CommandTiming, CallbackDuringCycleCannotMakeFreshInputAppearFuture)
{
  int64_t clock = 1000;
  const auto old_cycle_time = clock;
  const auto sample = readThenTimestamp(
    [&]() {
      clock += 100;
      return clock;
    },
    [&]() { return clock; });
  EXPECT_GT(sample.first, old_cycle_time);
  EXPECT_LE(sample.first, sample.second);
}
TEST(StopTransitions, OneCycleCauseSurvivesUntilDiagnosticDrain)
{
  StopTransitionQueue q;
  StopTransition e;
  e.sequence = 1;
  e.admission = 5;
  e.ros_stamp = 1.;
  ASSERT_TRUE(q.push(e));
  e.sequence = 2;
  e.admission = 0;
  e.waiting = true;
  ASSERT_TRUE(q.push(e));
  StopTransition out;
  ASSERT_TRUE(q.pop(out));
  EXPECT_EQ(out.admission, 5);
  EXPECT_EQ(out.sequence, 1U);
  ASSERT_TRUE(q.pop(out));
  EXPECT_EQ(out.admission, 0);
  EXPECT_TRUE(out.waiting);
  EXPECT_FALSE(q.pop(out));
}
TEST(StopTransitions, OverflowIsCountedAndDoesNotOverwriteUnreadCause)
{
  StopTransitionQueue q;
  StopTransition e;
  for (uint64_t i = 0; i < 63; ++i) {
    e.sequence = i;
    ASSERT_TRUE(q.push(e));
  }
  EXPECT_FALSE(q.push(e));
  EXPECT_EQ(q.dropped(), 1U);
  for (uint64_t i = 0; i < 63; ++i) {
    ASSERT_TRUE(q.pop(e));
    EXPECT_EQ(e.sequence, i);
  }
  EXPECT_FALSE(q.pop(e));
  EXPECT_TRUE(q.push(e));
}
