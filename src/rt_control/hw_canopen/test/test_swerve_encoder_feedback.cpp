#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "hw_canopen/swerve_encoder_feedback.hpp"

namespace hw_canopen
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

TEST(SwerveEncoderFeedbackTest, InitialIsInvalidAndAFrameNewerThanTheReadIsFresh)
{
  SwerveEncoderFeedback feedback{valid_config()};

  const auto initial = feedback.sample(1000LL)[0];
  EXPECT_FALSE(initial.valid);
  EXPECT_TRUE(std::isnan(initial.position_rad));
  EXPECT_TRUE(std::isinf(initial.feedback_age_ms));

  // The reader takes its time before reading the slot; a frame stored in between is newer
  // than that time. It is the freshest sample there is, not an invalid one (ELECTRI-127).
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 1U, 2000LL));
  const auto newer = feedback.sample(1000LL)[0];
  EXPECT_TRUE(newer.valid);
  EXPECT_DOUBLE_EQ(newer.feedback_age_ms, 0.0);
}

TEST(SwerveEncoderFeedbackTest, ReaderKeepsTheLastFrameAndAgesItUntilANewOneArrives)
{
  SwerveEncoderFeedback feedback{valid_config()};
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 100U, 1000000LL));
  EXPECT_DOUBLE_EQ(feedback.sample(2000000LL)[0].feedback_age_ms, 1.0);
  // No new frame: the same sample is returned with a growing age (staleness by age only).
  const auto again = feedback.sample(250000000LL)[0];
  EXPECT_TRUE(again.valid);
  EXPECT_DOUBLE_EQ(again.feedback_age_ms, 249.0);
}

TEST(SwerveEncoderFeedbackTest, ConcurrentWriterNeverYieldsAnInvalidOrTornSample)
{
  // One writer thread and one reader thread, as rtcan-master and the controller update.
  auto config = valid_config();
  for (auto & axis : config) {
    axis.ring_gear_teeth = 1U;
    axis.direction = 1;
    axis.installation_offset_rad = 0.0;
  }
  SwerveEncoderFeedback feedback{config};
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 0U, 1000LL));
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    // raw position and receive time move together: received = 1000 + raw (mod counts).
    for (std::uint32_t raw{1U}; !stop.load(std::memory_order_relaxed); ++raw) {
      const std::uint32_t wrapped{raw % 10000U};
      (void)feedback.observe(1U, 0x6004U, 0U, wrapped, 1000LL + static_cast<std::int64_t>(wrapped));
    }
  });
  std::size_t reads{0U};
  bool all_consistent{true};
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < end) {
    constexpr std::int64_t now{20000LL};
    const auto sample = feedback.sample(now)[0];
    ++reads;
    if (!sample.valid) {
      all_consistent = false;
      break;
    }
    // Recover raw from both fields; a torn read would make them disagree.
    const double raw_from_position{sample.position_rad * 10000.0 / (2.0 * kPi)};
    double raw_wrapped{std::fmod(raw_from_position + 10000.0, 10000.0)};
    const double raw_from_age{static_cast<double>(now - 1000LL) - sample.feedback_age_ms * 1e6};
    if (
      std::abs(raw_wrapped - raw_from_age) > 1e-3 &&
      std::abs(std::abs(raw_wrapped - raw_from_age) - 10000.0) > 1e-3) {
      all_consistent = false;
      break;
    }
  }
  stop = true;
  writer.join();
  EXPECT_GT(reads, 1000U);
  EXPECT_TRUE(all_consistent);
}
TEST(SwerveEncoderFeedbackTest, MultiTurnCounterWrapStaysContinuous)
{
  // Module 4 field calibration: zero sits 48372 counts below the 16-turn counter wrap.
  auto config = valid_config();
  for (auto & axis : config) {
    axis.counts_per_revolution = 65536U;
    axis.ring_gear_teeth = 108U;
    axis.pinion_gear_teeth = 27U;
    axis.direction = -1;
    axis.installation_offset_rad = -23.973339374474527;
    axis.encoder_revolutions = 16U;
  }
  SwerveEncoderFeedback feedback{config};
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 1048500U, 1000000LL));
  const auto before = feedback.sample(2000000LL)[0];
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 100U, 3000000LL));
  const auto after = feedback.sample(4000000LL)[0];
  ASSERT_TRUE(before.valid && after.valid);
  const double per_count{2.0 * kPi * 27.0 / (65536.0 * 108.0)};
  EXPECT_NEAR(after.position_rad - before.position_rad, -176.0 * per_count, 1e-9);
  // Counts outside the declared multi-turn range are not silently folded.
  ASSERT_TRUE(feedback.observe(1U, 0x6004U, 0U, 1048576U, 5000000LL));
  EXPECT_FALSE(feedback.sample(6000000LL)[0].valid);
}

}  // namespace
}  // namespace hw_canopen
