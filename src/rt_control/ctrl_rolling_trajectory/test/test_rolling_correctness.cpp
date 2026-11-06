#include <gtest/gtest.h>

#include "rolling_buffer_fixture.hpp"

namespace ctrl_rolling_trajectory::test
{

TEST_F(BufferFixture, InteriorSplicePreservesThePrefixAndHasOneContinuousKnot)
{
  ASSERT_EQ(
    admit({point(0U), point(100U, 0.1), point(300U, 0.3), point(500U, 0.5)}), RejectCode::kNone);
  const auto original = buffer.image();
  JointPoint splice;
  ASSERT_TRUE(sampleTrajectoryImage(original, 250U * kMs, splice));
  auto requested = splice;
  requested.positions[0] += 0.005;
  requested.velocities[0] += 0.01;
  AdmissionContext context{0U, 16U * kMs, 0U, 1U, false};
  ASSERT_EQ(admit({requested, point(550U, 0.55)}, context), RejectCode::kNone);
  EXPECT_EQ(buffer.image().point_count, 4U);
  for (std::uint64_t time = 0U; time <= 250U * kMs; time += kMs) {
    JointPoint before, after;
    ASSERT_TRUE(sampleTrajectoryImage(original, time, before));
    ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), time, after));
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_NEAR(before.positions[axis], after.positions[axis], 1.0e-12);
      EXPECT_NEAR(before.velocities[axis], after.velocities[axis], 1.0e-12);
    }
  }
  // Repeated replacements cannot accumulate the permitted first-point error.
  for (std::size_t i = 0; i < 20U; ++i) {
    context.active_generation = buffer.image().generation;
    ASSERT_EQ(admit({requested, point(550U, 0.55)}, context), RejectCode::kNone);
    JointPoint actual;
    ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), splice.time_ns, actual));
    EXPECT_EQ(actual.positions, splice.positions);
    EXPECT_EQ(actual.velocities, splice.velocities);
  }
}

TEST_F(BufferFixture, PrestartBatchesBuildOnTheLatestAcceptedPlan)
{
  AdmissionContext context;
  context.priming = true;
  ASSERT_EQ(admit({point(0U), point(500U, 0.1)}, context), RejectCode::kNone);
  JointPoint first;
  ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), 200U * kMs, first));
  ASSERT_EQ(admit({first, point(500U, 0.2)}, context), RejectCode::kNone);
  JointPoint second;
  ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), 300U * kMs, second));
  ASSERT_EQ(admit({second, point(600U, 0.3)}, context), RejectCode::kNone);
  EXPECT_EQ(buffer.image().generation, 3U);
  EXPECT_EQ(buffer.image().earliest_changed_ns, 0U);
  JointPoint terminal;
  ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), 600U * kMs, terminal));
  EXPECT_DOUBLE_EQ(terminal.positions[0], 0.3);
}

TEST_F(BufferFixture, CompactionKeepsLongStreamsBounded)
{
  ASSERT_EQ(
    admit({point(0U), point(100U), point(200U), point(300U), point(400U), point(500U)}),
    RejectCode::kNone);
  for (std::uint64_t cycle = 1U; cycle <= 500U; ++cycle) {
    const auto elapsed = cycle * 20U;
    AdmissionContext context{
      elapsed * kMs, (elapsed + 16U) * kMs, 0U, buffer.image().generation, false};
    ASSERT_EQ(admit({point(elapsed + 100U), point(elapsed + 500U)}, context), RejectCode::kNone);
    EXPECT_LE(buffer.image().point_count, 8U);
    JointPoint sampled;
    EXPECT_TRUE(sampleTrajectoryImage(buffer.image(), elapsed * kMs, sampled));
    EXPECT_DOUBLE_EQ(sampled.positions[0], 0.0);
  }
}

TEST_F(BufferFixture, HorizonTracksExecutionTimeAndRejectedUpdatesDoNotChangeThePlan)
{
  config.max_horizon_ns = 600U * kMs;
  configure();
  EXPECT_EQ(admit({point(0U), point(601U)}), RejectCode::kHorizonExceeded);
  ASSERT_EQ(admit({point(0U), point(500U)}), RejectCode::kNone);
  AdmissionContext context{200U * kMs, 216U * kMs, 100U * kMs, 1U, false};
  EXPECT_EQ(admit({point(250U), point(300U)}, context), RejectCode::kInsufficientHorizon);
  EXPECT_EQ(admit({point(215U), point(800U)}, context), RejectCode::kLateReplace);
  EXPECT_EQ(admit({point(501U), point(800U)}, context), RejectCode::kTimeGap);
  ASSERT_EQ(admit({point(250U), point(800U)}, context), RejectCode::kNone);
  EXPECT_EQ(buffer.image().earliest_changed_ns, 250U * kMs);
  // Before RT observes that plan, a later update must retain its earliest changed boundary.
  ASSERT_EQ(admit({point(400U), point(800U)}, context), RejectCode::kNone);
  EXPECT_EQ(buffer.image().earliest_changed_ns, 250U * kMs);
}

TEST_F(BufferFixture, MonotonicCursorMatchesSamplingAcrossGenerationAndTimeChanges)
{
  ASSERT_EQ(
    admit({point(0U), point(100U, 0.1), point(300U, 0.3), point(500U, 0.5)}), RejectCode::kNone);
  MonotonicTrajectoryCursor cursor;
  std::size_t advances = 0U;
  for (std::uint64_t time = 0U; time <= 500U * kMs; time += kMs) {
    JointPoint a, b;
    std::size_t step = 0U;
    ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), time, a));
    ASSERT_TRUE(sampleTrajectoryImageMonotonic(buffer.image(), time, cursor, b, &step));
    advances += step;
    EXPECT_EQ(a.positions, b.positions);
    EXPECT_EQ(a.velocities, b.velocities);
  }
  EXPECT_EQ(advances, 3U);
  buffer.clear();
  ASSERT_TRUE(buffer.reset(point(0U)));
  ASSERT_EQ(admit({point(0U), point(500U, -0.2)}), RejectCode::kNone);
  JointPoint actual;
  ASSERT_TRUE(sampleTrajectoryImageMonotonic(buffer.image(), 100U * kMs, cursor, actual));
  EXPECT_LT(actual.positions[0], 0.0);
}

}  // namespace ctrl_rolling_trajectory::test
