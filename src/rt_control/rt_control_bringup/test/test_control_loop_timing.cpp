// Copyright 2026 Sevenova Hangzhou
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <chrono>

#include "rt_control_bringup/control_loop_timing.hpp"

using rt_control_bringup::next_deadline;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using Clock = std::chrono::steady_clock;

namespace
{
const Clock::time_point kStart{std::chrono::seconds(100)};
const std::chrono::nanoseconds kPeriod = milliseconds(1);
}  // namespace

TEST(ControlLoopTiming, OnTimeCycleAdvancesOnePeriod)
{
  const auto step = next_deadline(kStart, kStart + microseconds(300), kPeriod, true);
  EXPECT_EQ(step.deadline, kStart + kPeriod);
  EXPECT_EQ(step.missed_cycles, 0U);
}

TEST(ControlLoopTiming, CycleEndingExactlyAtTheNextTickIsNotAnOverrun)
{
  const auto step = next_deadline(kStart, kStart + kPeriod, kPeriod, true);
  EXPECT_EQ(step.deadline, kStart + kPeriod);
  EXPECT_EQ(step.missed_cycles, 0U);
}

TEST(ControlLoopTiming, LateCycleSkipsToTheNextFutureTickOfTheSameGrid)
{
  // The cycle ended 2.4 periods after its start: ticks +1 and +2 are past, +3 is next.
  const auto step = next_deadline(kStart, kStart + microseconds(2400), kPeriod, true);
  EXPECT_EQ(step.deadline, kStart + 3 * kPeriod);
  EXPECT_EQ(step.missed_cycles, 2U);
}

TEST(ControlLoopTiming, LongBlockedReadIsSkippedNotReplayed)
{
  // Hardware activation held resources_lock_ for 6.6 s (dual-arm bench, 2026-10-22): Humble
  // 2.53.1 ran about 6600 cycles back to back afterwards. Now the loop resumes on the next tick.
  const auto now = kStart + std::chrono::nanoseconds(6'600'250'000);
  const auto step = next_deadline(kStart, now, kPeriod, true);
  EXPECT_GE(step.deadline, now);
  EXPECT_LT(step.deadline - now, kPeriod);
  EXPECT_EQ((step.deadline - kStart) % kPeriod, std::chrono::nanoseconds(0));
  EXPECT_EQ(step.missed_cycles, 6600U);
}

TEST(ControlLoopTiming, UnmanagedOverrunsKeepTheHumbleBehaviour)
{
  const auto step = next_deadline(kStart, kStart + milliseconds(50), kPeriod, false);
  EXPECT_EQ(step.deadline, kStart + kPeriod);
  EXPECT_EQ(step.missed_cycles, 0U);
}

TEST(ControlLoopTiming, NonMillisecondPeriodsStayOnTheirGrid)
{
  const std::chrono::nanoseconds period = microseconds(4000);  // 250 Hz
  // Ticks at +4 ms and +8 ms are past at +9.001 ms; the loop wakes at +12 ms.
  const auto step = next_deadline(kStart, kStart + microseconds(9001), period, true);
  EXPECT_EQ(step.deadline, kStart + 3 * period);
  EXPECT_EQ(step.missed_cycles, 2U);
}
