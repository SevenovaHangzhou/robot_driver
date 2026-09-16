#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "robot_hw_canopen/swerve_encoder_feedback.hpp"

namespace robot_hw_canopen
{
namespace
{

constexpr double kPi{3.14159265358979323846};

[[nodiscard]] std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> valid_config()
{
  return {
    SwerveEncoderAxisConfig{1U, 10000U, 2U, 1U, 1, 0.0},
    SwerveEncoderAxisConfig{2U, 10000U, 2U, 1U, -1, 0.1},
    SwerveEncoderAxisConfig{3U, 10000U, 2U, 1U, 1, 0.2},
    SwerveEncoderAxisConfig{4U, 10000U, 2U, 1U, -1, -0.1},
  };
}

TEST(SwerveEncoderFeedbackTest, RejectsIncompleteOrAmbiguousConfiguration)
{
  auto config = valid_config();
  config[1].node_id = config[0].node_id;
  EXPECT_THROW(SwerveEncoderFeedback{config}, std::invalid_argument);

  config = valid_config();
  config[0].counts_per_revolution = 0U;
  EXPECT_THROW(SwerveEncoderFeedback{config}, std::invalid_argument);

  config = valid_config();
  config[0].ring_gear_teeth = 0U;
  EXPECT_THROW(SwerveEncoderFeedback{config}, std::invalid_argument);

  config = valid_config();
  config[0].direction = 0;
  EXPECT_THROW(SwerveEncoderFeedback{config}, std::invalid_argument);

  config = valid_config();
  config[0].installation_offset_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SwerveEncoderFeedback{config}, std::invalid_argument);
}

TEST(SwerveEncoderFeedbackTest, RoutesPositionObjectAndReportsRealReceiveAge)
{
  SwerveEncoderFeedback feedback{valid_config()};

  EXPECT_TRUE(feedback.observe(3U, 0x6004U, 0U, 1000U, 1000000000LL));
  const auto samples = feedback.sample(1002000000LL);

  EXPECT_FALSE(samples[0].valid);
  EXPECT_FALSE(samples[1].valid);
  EXPECT_TRUE(samples[2].valid);
  EXPECT_NEAR(samples[2].position_rad, 0.1 * kPi - 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(samples[2].feedback_age_ms, 2.0);
}

TEST(SwerveEncoderFeedbackTest, ConfirmedExternalGearingMapsFourEncoderTurnsToOneAxisTurn)
{
  auto config = valid_config();
  for (auto & axis : config) {
    axis.ring_gear_teeth = 108U;
    axis.pinion_gear_teeth = 27U;
    axis.direction = 1;
    axis.installation_offset_rad = 0.0;
  }
  SwerveEncoderFeedback feedback{config};

  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 10000U, 1000000LL));
  const auto sample = feedback.sample(2000000LL)[0];

  ASSERT_TRUE(sample.valid);
  EXPECT_NEAR(sample.position_rad, kPi / 2.0, 1e-12);
}

TEST(SwerveEncoderFeedbackTest, RejectsUnknownNodeWrongObjectAndInvalidTimestamp)
{
  SwerveEncoderFeedback feedback{valid_config()};

  EXPECT_FALSE(feedback.observe(5U, 0x6004U, 0U, 1U, 1LL));
  EXPECT_FALSE(feedback.observe(1U, 0x6501U, 0U, 1U, 1LL));
  EXPECT_FALSE(feedback.observe(1U, 0x6004U, 1U, 1U, 1LL));
  EXPECT_FALSE(feedback.observe(1U, 0x6004U, 0U, 1U, 0LL));
}

TEST(SwerveEncoderFeedbackTest, IdenticalPositionStillRefreshesFeedbackAge)
{
  SwerveEncoderFeedback feedback{valid_config()};
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 500U, 1000000LL));
  EXPECT_DOUBLE_EQ(feedback.sample(4000000LL)[0].feedback_age_ms, 3.0);

  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 500U, 5000000LL));
  const auto refreshed = feedback.sample(6000000LL)[0];

  EXPECT_TRUE(refreshed.valid);
  EXPECT_DOUBLE_EQ(refreshed.feedback_age_ms, 1.0);
  EXPECT_NEAR(refreshed.position_rad, 0.05 * kPi, 1e-12);
}

TEST(SwerveEncoderFeedbackTest, InitialOrFutureDatedFeedbackIsInvalid)
{
  SwerveEncoderFeedback feedback{valid_config()};

  const auto initial = feedback.sample(1000LL)[0];
  EXPECT_FALSE(initial.valid);
  EXPECT_TRUE(std::isnan(initial.position_rad));
  EXPECT_TRUE(std::isinf(initial.feedback_age_ms));

  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 1U, 2000LL));
  const auto future = feedback.sample(1000LL)[0];
  EXPECT_FALSE(future.valid);
  EXPECT_TRUE(std::isinf(future.feedback_age_ms));
}

}  // namespace
}  // namespace robot_hw_canopen
