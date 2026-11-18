#include <gtest/gtest.h>

#include <limits>

#include "ctrl_wheel_observation_broadcaster/channel_tracker.hpp"

namespace ctrl_wheel_observation_broadcaster
{
namespace
{
constexpr double kTimeoutMs = 200.0;
constexpr std::int64_t kMs = 1000000;

ChannelInput fresh(double age_ms = 0.0)
{
  return {age_ms, true, false, true, false};
}

ChannelInput repeated(double age_ms)
{
  return {age_ms, true, false, false, false};
}

TEST(ChannelTrackerTest, NotReadyUntilTheFirstSample)
{
  ChannelTracker tracker;
  const auto & state = tracker.update(
    {std::numeric_limits<double>::infinity(), false, false, false, false}, 10 * kMs, kTimeoutMs);
  EXPECT_FALSE(state.valid);
  EXPECT_EQ(state.reason, Reason::not_ready);
  EXPECT_EQ(state.sample_sequence, 0U);
}

TEST(ChannelTrackerTest, RepeatedSampleKeepsItsTimeAndSequence)
{
  ChannelTracker tracker;
  tracker.update(fresh(2.0), 100 * kMs, kTimeoutMs);
  EXPECT_EQ(tracker.state().sample_ns, 98 * kMs);
  EXPECT_EQ(tracker.state().sample_sequence, 1U);
  const auto & state = tracker.update(repeated(3.0), 101 * kMs, kTimeoutMs);
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.sample_ns, 98 * kMs);
  EXPECT_EQ(state.sample_sequence, 1U);
  tracker.update(fresh(0.5), 102 * kMs, kTimeoutMs);
  EXPECT_EQ(tracker.state().sample_sequence, 2U);
  EXPECT_EQ(tracker.state().epoch, 0U);
}

TEST(ChannelTrackerTest, StaleSampleIsInvalidAndRecoveryStartsANewEpoch)
{
  ChannelTracker tracker;
  tracker.update(fresh(), 0, kTimeoutMs);
  const auto & stale = tracker.update(repeated(250.0), 250 * kMs, kTimeoutMs);
  EXPECT_FALSE(stale.valid);
  EXPECT_EQ(stale.reason, Reason::stale);
  EXPECT_EQ(stale.sample_ns, 0);
  const auto & recovered = tracker.update(fresh(), 260 * kMs, kTimeoutMs);
  EXPECT_TRUE(recovered.valid);
  EXPECT_EQ(recovered.epoch, 1U);
  EXPECT_EQ(recovered.sample_sequence, 2U);
}

TEST(ChannelTrackerTest, ReasonsAndDiscontinuity)
{
  ChannelTracker tracker;
  tracker.update(fresh(), 0, kTimeoutMs);
  EXPECT_EQ(
    tracker.update({0.0, false, false, true, false}, kMs, kTimeoutMs).reason, Reason::nonfinite);
  EXPECT_EQ(
    tracker.update({0.0, true, true, true, false}, 2 * kMs, kTimeoutMs).reason,
    Reason::device_fault);
  EXPECT_EQ(tracker.update(fresh(), 3 * kMs, kTimeoutMs).epoch, 1U);
  // A valid sample that cannot continue the previous one also starts a new epoch.
  EXPECT_EQ(tracker.update({0.0, true, false, true, true}, 4 * kMs, kTimeoutMs).epoch, 2U);
  EXPECT_TRUE(tracker.state().valid);
}

TEST(ChannelTrackerTest, ResetStartsOver)
{
  ChannelTracker tracker;
  tracker.update(fresh(), kMs, kTimeoutMs);
  tracker.reset();
  EXPECT_EQ(tracker.state().sample_sequence, 0U);
  EXPECT_EQ(tracker.update(repeated(1.0), 2 * kMs, kTimeoutMs).reason, Reason::not_ready);
}
}  // namespace
}  // namespace ctrl_wheel_observation_broadcaster
