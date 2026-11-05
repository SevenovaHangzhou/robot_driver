#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rolling_buffer_fixture.hpp"

namespace ctrl_rolling_trajectory::test
{

TEST_F(BufferFixture, ConfigurationAndHoldRequireUsableLimits)
{
  for (const auto capacity : {0U, 1U, 257U}) {
    config.capacity = capacity;
    EXPECT_FALSE(buffer.configure(config, envelope));
  }
  for (const auto capacity : {2U, 256U}) {
    config.capacity = capacity;
    EXPECT_TRUE(buffer.configure(config, envelope));
  }
  config.max_horizon_ns = 0U;
  EXPECT_FALSE(buffer.configure(config, envelope));
  config.max_horizon_ns = 1000U * kMs;
  envelope.axes[0].velocity_positive = 0.0;
  EXPECT_FALSE(buffer.configure(config, envelope));
  envelope.axes[0].velocity_positive = 1.0;
  ASSERT_TRUE(buffer.configure(config, envelope));
  EXPECT_FALSE(buffer.reset(point(0U, 101.0)));
  EXPECT_FALSE(buffer.reset(point(0U, 0.0, 1.0)));
  EXPECT_FALSE(buffer.reset(point(1U)));
  EXPECT_TRUE(buffer.reset(point(0U)));
}

TEST_F(BufferFixture, RejectsMalformedPointsWithoutReplacingThePlan)
{
  ASSERT_EQ(admit({point(0U), point(1000U)}), RejectCode::kNone);
  TrajectoryImage candidate;
  EXPECT_EQ(buffer.prepare({0U, nullptr, 2U}, {}, candidate), RejectCode::kInvalidShape);
  std::vector<JointPoint> points{point(0U), point(1000U)};
  auto input = views(points);
  EXPECT_EQ(buffer.prepare({0U, input.data(), 1U}, {}, candidate), RejectCode::kInvalidShape);
  EXPECT_EQ(buffer.prepare({0U, input.data(), 65U}, {}, candidate), RejectCode::kCapacityExceeded);
  for (const bool position : {false, true}) {
    input = views(points);
    if (position) {
      input[1].position_count = kAxisCount - 1U;
    } else {
      input[1].velocity_count = kAxisCount - 1U;
    }
    EXPECT_EQ(
      buffer.prepare({0U, input.data(), input.size()}, {}, candidate), RejectCode::kInvalidShape);
    for (const double bad :
         {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
      points[1] = point(1000U);
      (position ? points[1].positions : points[1].velocities)[0] = bad;
      EXPECT_EQ(admit(points), RejectCode::kNonFinite);
    }
    points[1] = point(1000U);
  }
  EXPECT_EQ(admit({point(0U), point(0U)}), RejectCode::kNonMonotonicTime);
  EXPECT_EQ(admit({point(20U), point(10U)}), RejectCode::kNonMonotonicTime);
  EXPECT_EQ(buffer.image().generation, 1U);
  EXPECT_EQ(buffer.image().point_count, 2U);
}

TEST_F(BufferFixture, FirstPointMustMatchTheHoldButCannotMoveIt)
{
  EXPECT_EQ(admit({point(10U), point(500U)}), RejectCode::kTimeGap);
  EXPECT_EQ(admit({point(0U, 0.011), point(500U)}), RejectCode::kPositionDiscontinuity);
  EXPECT_EQ(admit({point(0U, 0.0, 0.031), point(500U)}), RejectCode::kVelocityDiscontinuity);
  EXPECT_EQ(admit({point(0U, 0.005, 0.01), point(500U, 0.02)}), RejectCode::kNone);
  EXPECT_EQ(buffer.image().points[0].positions, JointPoint{}.positions);
  EXPECT_EQ(buffer.image().points[0].velocities, JointPoint{}.velocities);
}

TEST_F(BufferFixture, NoInitialHorizonRequirementSurvivesTheOldProtocol)
{
  AdmissionContext context;
  context.priming = true;
  context.minimum_horizon_ns = 20U * kMs;
  EXPECT_EQ(admit({point(0U), point(5U)}, context), RejectCode::kNone);
}

TEST_F(BufferFixture, CubicInteriorAndStoppingDistanceAreBothChecked)
{
  envelope.axes[0].velocity_positive = 1.4;
  configure();
  EXPECT_EQ(admit({point(0U), point(1000U, 1.0)}), RejectCode::kVelocityLimit);
  envelope.axes[0].velocity_positive = 100.0;
  envelope.axes[0].acceleration_positive = 1.0;
  configure();
  EXPECT_EQ(admit({point(0U), point(1000U, 1.0)}), RejectCode::kAccelerationLimit);
  envelope.axes[0].acceleration_positive = 1.0e6;
  envelope.axes[0].position_upper = 1.0;
  envelope.axes[0].stop_acceleration_positive = 1.0;
  configure();
  EXPECT_EQ(
    admit({point(0U), point(500U, 0.4, 0.8), point(1000U, 0.8, 0.8)}),
    RejectCode::kNotStoppingViable);
  EXPECT_EQ(buffer.image().point_count, 0U);
}

TEST_F(BufferFixture, CapacityIncludesTheRetainedPrefix)
{
  config.capacity = 4U;
  configure();
  ASSERT_EQ(
    admit({point(0U), point(100U, 0.1), point(200U, 0.2), point(300U, 0.3)}), RejectCode::kNone);
  AdmissionContext context;
  context.active_generation = 1U;
  EXPECT_EQ(admit({point(300U, 0.3), point(400U, 0.4)}, context), RejectCode::kCapacityExceeded);
  EXPECT_EQ(buffer.image().generation, 1U);
}

TEST_F(BufferFixture, CommitRechecksTheBoundaryAndHorizonAfterValidation)
{
  ASSERT_EQ(admit({point(0U), point(500U, 0.2)}), RejectCode::kNone);
  JointPoint splice;
  ASSERT_TRUE(sampleTrajectoryImage(buffer.image(), 200U * kMs, splice));
  AdmissionContext context{100U * kMs, 150U * kMs, 0U, 1U, false};
  TrajectoryImage candidate;
  ASSERT_EQ(prepare({splice, point(600U, 0.3)}, context, candidate), RejectCode::kNone);
  context.replaceable_from_ns = 201U * kMs;
  EXPECT_EQ(buffer.commit(candidate, context), RejectCode::kLateReplace);
  context.replaceable_from_ns = 200U * kMs;
  context.minimum_horizon_ns = 500U * kMs;
  EXPECT_EQ(buffer.commit(candidate, context), RejectCode::kInsufficientHorizon);
  EXPECT_EQ(buffer.image().generation, 1U);
  context.minimum_horizon_ns = 0U;
  EXPECT_EQ(buffer.commit(candidate, context), RejectCode::kNone);
  EXPECT_EQ(buffer.image().generation, 2U);
}

}  // namespace ctrl_rolling_trajectory::test
