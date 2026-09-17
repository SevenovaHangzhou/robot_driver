#include "lpms_nav3_can/decoder.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-9;

lpms_nav3_can::CanFrame make_frame(
  const std::uint32_t id, const std::array<std::int16_t, 4U> & values)
{
  lpms_nav3_can::CanFrame frame;
  frame.id = id;
  frame.dlc = 8U;
  for (std::size_t index = 0U; index < values.size(); ++index) {
    const auto raw = static_cast<std::uint16_t>(values[index]);
    frame.data[index * 2U] = static_cast<std::uint8_t>(raw & 0x00ffU);
    frame.data[index * 2U + 1U] = static_cast<std::uint8_t>((raw >> 8U) & 0x00ffU);
  }
  return frame;
}

std::array<lpms_nav3_can::CanFrame, 4U> make_default_sample_frames(
  const std::uint8_t node_id = 1U)
{
  const auto node = static_cast<std::uint32_t>(node_id);
  return {
    make_frame(0x180U + node, {1000, -500, 0, 900}),
    make_frame(0x280U + node, {-900, 1800, 100, -200}),
    make_frame(0x380U + node, {300, 100, -200, 9000}),
    make_frame(0x480U + node, {10000, 0, 0, 0}),
  };
}

}  // namespace

TEST(DecoderTest, CompletesOneSampleFromDefaultPdoMapping)
{
  lpms_nav3_can::Decoder decoder{1U};
  const auto frames = make_default_sample_frames();

  EXPECT_FALSE(decoder.consume(frames[0]).sample.has_value());
  EXPECT_FALSE(decoder.consume(frames[1]).sample.has_value());
  EXPECT_FALSE(decoder.consume(frames[2]).sample.has_value());
  const auto result = decoder.consume(frames[3]);

  ASSERT_TRUE(result.sample.has_value());
  const auto & sample = *result.sample;
  EXPECT_NEAR(sample.linear_acceleration_mps2[0], 9.80665, kTolerance);
  EXPECT_NEAR(sample.linear_acceleration_mps2[1], -4.903325, kTolerance);
  EXPECT_NEAR(sample.linear_acceleration_mps2[2], 0.0, kTolerance);
  EXPECT_NEAR(sample.angular_velocity_radps[0], kPi / 2.0, kTolerance);
  EXPECT_NEAR(sample.angular_velocity_radps[1], -kPi / 2.0, kTolerance);
  EXPECT_NEAR(sample.angular_velocity_radps[2], kPi, kTolerance);
  EXPECT_NEAR(sample.magnetic_field_t[0], 1.0e-6, kTolerance);
  EXPECT_NEAR(sample.magnetic_field_t[1], -2.0e-6, kTolerance);
  EXPECT_NEAR(sample.magnetic_field_t[2], 3.0e-6, kTolerance);
  EXPECT_NEAR(sample.euler_rad[0], kPi / 180.0, kTolerance);
  EXPECT_NEAR(sample.euler_rad[1], -2.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(sample.euler_rad[2], kPi / 2.0, kTolerance);
  EXPECT_DOUBLE_EQ(sample.orientation_wxyz[0], 1.0);
  EXPECT_DOUBLE_EQ(sample.orientation_wxyz[1], 0.0);
  EXPECT_DOUBLE_EQ(sample.orientation_wxyz[2], 0.0);
  EXPECT_DOUBLE_EQ(sample.orientation_wxyz[3], 0.0);
  EXPECT_EQ(decoder.pending_mask(), 0U);
}

TEST(DecoderTest, SupportsConfigurableNodeIdAndAnyFrameOrder)
{
  lpms_nav3_can::Decoder decoder{7U};
  const auto frames = make_default_sample_frames(7U);

  EXPECT_FALSE(decoder.consume(frames[3]).sample.has_value());
  EXPECT_FALSE(decoder.consume(frames[1]).sample.has_value());
  EXPECT_FALSE(decoder.consume(frames[0]).sample.has_value());
  const auto result = decoder.consume(frames[2]);

  ASSERT_TRUE(result.sample.has_value());
  EXPECT_EQ(decoder.node_id(), 7U);
  EXPECT_DOUBLE_EQ(result.sample->orientation_wxyz[0], 1.0);
}

TEST(DecoderTest, RejectsMalformedAndUnrelatedFramesWithoutAdvancingAssembly)
{
  lpms_nav3_can::Decoder decoder{1U};
  auto malformed = make_default_sample_frames()[0];
  malformed.dlc = 7U;
  auto unrelated = make_default_sample_frames()[1];
  unrelated.id = 0x123U;
  auto extended = make_default_sample_frames()[2];
  extended.is_extended = true;

  const auto malformed_result = decoder.consume(malformed);
  EXPECT_TRUE(malformed_result.recognized);
  EXPECT_TRUE(malformed_result.malformed);
  EXPECT_FALSE(decoder.consume(unrelated).recognized);
  EXPECT_FALSE(decoder.consume(extended).recognized);
  EXPECT_EQ(decoder.pending_mask(), 0U);
}

TEST(DecoderTest, ReportsHeartbeatWithoutProducingSample)
{
  lpms_nav3_can::Decoder decoder{1U};
  lpms_nav3_can::CanFrame heartbeat;
  heartbeat.id = 0x701U;
  heartbeat.dlc = 1U;
  heartbeat.data[0] = 0x05U;

  const auto result = decoder.consume(heartbeat);

  ASSERT_TRUE(result.heartbeat_state.has_value());
  EXPECT_EQ(*result.heartbeat_state, 0x05U);
  EXPECT_TRUE(result.recognized);
  EXPECT_FALSE(result.malformed);
  EXPECT_FALSE(result.sample.has_value());
  EXPECT_EQ(decoder.pending_mask(), 0U);
}

TEST(DecoderTest, AcceptsNav3HeartbeatPaddedToEightBytes)
{
  lpms_nav3_can::Decoder decoder{1U};
  lpms_nav3_can::CanFrame heartbeat;
  heartbeat.id = 0x701U;
  heartbeat.dlc = 8U;
  heartbeat.data[0] = 0x05U;

  const auto result = decoder.consume(heartbeat);

  ASSERT_TRUE(result.heartbeat_state.has_value());
  EXPECT_EQ(*result.heartbeat_state, 0x05U);
  EXPECT_TRUE(result.recognized);
  EXPECT_FALSE(result.malformed);
}

TEST(DecoderTest, RejectsNonzeroHeartbeatPadding)
{
  lpms_nav3_can::Decoder decoder{1U};
  lpms_nav3_can::CanFrame heartbeat;
  heartbeat.id = 0x701U;
  heartbeat.dlc = 8U;
  heartbeat.data[0] = 0x05U;
  heartbeat.data[1] = 0x01U;

  const auto result = decoder.consume(heartbeat);

  EXPECT_TRUE(result.recognized);
  EXPECT_TRUE(result.malformed);
  EXPECT_FALSE(result.heartbeat_state.has_value());
}

TEST(DecoderTest, ResetsPartialAssemblyAfterCompletedSample)
{
  lpms_nav3_can::Decoder decoder{1U};
  const auto frames = make_default_sample_frames();
  for (const auto & frame : frames) {
    decoder.consume(frame);
  }

  EXPECT_FALSE(decoder.consume(frames[3]).sample.has_value());
  EXPECT_EQ(decoder.pending_mask(), 0x08U);
  decoder.reset();
  EXPECT_EQ(decoder.pending_mask(), 0U);
}

TEST(DecoderTest, ConvertsLpmsCoordinatesToRosConventions)
{
  lpms_nav3_can::ImuSample sample;
  sample.linear_acceleration_mps2 = {1.0, -2.0, 3.0};
  sample.angular_velocity_radps = {4.0, -5.0, 6.0};
  sample.magnetic_field_t = {7.0, -8.0, 9.0};
  sample.orientation_wxyz = {0.5, 0.1, -0.2, 0.3};
  sample.euler_rad = {0.4, -0.5, 0.6};

  const auto converted = lpms_nav3_can::convert_to_ros_convention(sample, true);

  EXPECT_EQ(converted.linear_acceleration_mps2, (std::array<double, 3U>{-1.0, 2.0, -3.0}));
  EXPECT_EQ(converted.angular_velocity_radps, sample.angular_velocity_radps);
  EXPECT_EQ(converted.magnetic_field_t, sample.magnetic_field_t);
  EXPECT_EQ(converted.orientation_wxyz, (std::array<double, 4U>{0.5, -0.1, 0.2, -0.3}));
  EXPECT_EQ(converted.euler_rad, sample.euler_rad);
}

TEST(DecoderTest, LeavesSampleUnchangedWhenConversionIsDisabled)
{
  lpms_nav3_can::ImuSample sample;
  sample.linear_acceleration_mps2 = {1.0, 2.0, 3.0};
  sample.angular_velocity_radps = {4.0, 5.0, 6.0};
  sample.magnetic_field_t = {7.0, 8.0, 9.0};
  sample.orientation_wxyz = {1.0, 0.0, 0.0, 0.0};
  sample.euler_rad = {0.1, 0.2, 0.3};

  const auto converted = lpms_nav3_can::convert_to_ros_convention(sample, false);

  EXPECT_EQ(converted.linear_acceleration_mps2, sample.linear_acceleration_mps2);
  EXPECT_EQ(converted.angular_velocity_radps, sample.angular_velocity_radps);
  EXPECT_EQ(converted.magnetic_field_t, sample.magnetic_field_t);
  EXPECT_EQ(converted.orientation_wxyz, sample.orientation_wxyz);
  EXPECT_EQ(converted.euler_rad, sample.euler_rad);
}

TEST(DecoderTest, RejectsInvalidCanopenNodeIds)
{
  EXPECT_THROW((void)lpms_nav3_can::Decoder{0U}, std::invalid_argument);
  EXPECT_THROW((void)lpms_nav3_can::Decoder{128U}, std::invalid_argument);
}
