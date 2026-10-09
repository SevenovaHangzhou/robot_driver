#include <gtest/gtest.h>

#include "bms_node/protocol.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{

constexpr std::uint32_t kResponseId = 0x18904001U;

TEST(BmsProtocolTest, BuildsGoldenPhoenixExtendedIdentifiers)
{
  EXPECT_EQ(
    bms_node::make_can_id(0x90U, 0x01U, 0x40U),
    0x18900140U);
  EXPECT_EQ(
    bms_node::make_can_id(0x90U, 0x40U, 0x01U),
    kResponseId);
}

TEST(BmsProtocolTest, DecodesBigEndianVoltageAndSoc)
{
  bms_node::GoldenPhoenixDecoder decoder{
    bms_node::ByteOrder::kBigEndian, 0x01U, 0x40U};
  bms_node::BmsSample sample;
  const std::array<std::uint8_t, 8U> payload{
    0x01U, 0xF4U, 0x01U, 0xF4U, 0x75U, 0x30U, 0x03U, 0x20U};

  ASSERT_TRUE(decoder.ingest(sample, kResponseId, payload.data(), payload.size(), 10.0));
  ASSERT_TRUE(sample.voltage_v);
  ASSERT_TRUE(sample.soc_fraction);
  EXPECT_DOUBLE_EQ(*sample.voltage_v, 50.0);
  EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.8);
  EXPECT_EQ(decoder.resolved_order(), bms_node::ByteOrder::kBigEndian);
}

TEST(BmsProtocolTest, DecodesLittleEndianVoltageAndSoc)
{
  bms_node::GoldenPhoenixDecoder decoder{
    bms_node::ByteOrder::kLittleEndian, 0x01U, 0x40U};
  bms_node::BmsSample sample;
  const std::array<std::uint8_t, 8U> payload{
    0xF4U, 0x01U, 0xF4U, 0x01U, 0x30U, 0x75U, 0x20U, 0x03U};

  ASSERT_TRUE(decoder.ingest(sample, kResponseId, payload.data(), payload.size(), 10.0));
  EXPECT_DOUBLE_EQ(*sample.voltage_v, 50.0);
  EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.8);
}

TEST(BmsProtocolTest, AutoDetectsAUniquePlausibleSocByteOrder)
{
  bms_node::GoldenPhoenixDecoder decoder{bms_node::ByteOrder::kAuto, 0x01U, 0x40U};
  bms_node::BmsSample sample;
  const std::array<std::uint8_t, 8U> payload{
    0x01U, 0xF4U, 0x01U, 0xF4U, 0x75U, 0x30U, 0x03U, 0x20U};

  EXPECT_TRUE(decoder.ingest(sample, kResponseId, payload.data(), payload.size(), 10.0));
  EXPECT_EQ(decoder.resolved_order(), bms_node::ByteOrder::kBigEndian);
}

TEST(BmsProtocolTest, AutoModeRejectsAmbiguousByteOrderWithoutMutatingSample)
{
  bms_node::GoldenPhoenixDecoder decoder{bms_node::ByteOrder::kAuto, 0x01U, 0x40U};
  bms_node::BmsSample sample{48.0, 0.5, 2.0};
  const std::array<std::uint8_t, 8U> ambiguous{
    0x01U, 0xF4U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U, 0x02U};

  EXPECT_FALSE(decoder.ingest(sample, kResponseId, ambiguous.data(), ambiguous.size(), 3.0));
  EXPECT_DOUBLE_EQ(*sample.voltage_v, 48.0);
  EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.5);
  EXPECT_DOUBLE_EQ(*sample.last_frame_s, 2.0);
}

TEST(BmsProtocolTest, RejectsWrongIdentifierLengthTimestampAndSoc)
{
  bms_node::GoldenPhoenixDecoder decoder{
    bms_node::ByteOrder::kBigEndian, 0x01U, 0x40U};
  bms_node::BmsSample sample;
  const std::array<std::uint8_t, 8U> payload{
    0x01U, 0xF4U, 0x00U, 0x00U, 0x00U, 0x00U, 0x03U, 0x20U};
  auto invalid_soc = payload;
  invalid_soc[6] = 0x03U;
  invalid_soc[7] = 0xE9U;

  EXPECT_FALSE(decoder.ingest(sample, 0x18904002U, payload.data(), payload.size(), 1.0));
  EXPECT_FALSE(decoder.ingest(sample, kResponseId, payload.data(), 7U, 1.0));
  EXPECT_FALSE(decoder.ingest(
    sample, kResponseId, payload.data(), payload.size(),
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(decoder.ingest(
    sample, kResponseId, invalid_soc.data(), invalid_soc.size(), 1.0));
  EXPECT_FALSE(sample.last_frame_s);
}

TEST(BmsProtocolTest, ProjectsOnlyFreshAtomicSamples)
{
  const bms_node::BmsSample fresh{50.0, 0.8, 8.0};
  const bms_node::BmsSample stale{50.0, 0.8, 5.0};

  const auto fresh_values = bms_node::project_battery_values(fresh, 10.0, 3.0);
  EXPECT_DOUBLE_EQ(fresh_values.voltage_v, 50.0);
  EXPECT_DOUBLE_EQ(fresh_values.soc_fraction, 0.8);
  const auto stale_values = bms_node::project_battery_values(stale, 10.0, 3.0);
  EXPECT_TRUE(std::isnan(stale_values.voltage_v));
  EXPECT_TRUE(std::isnan(stale_values.soc_fraction));
}

TEST(BmsProtocolTest, ValidatesByteOrderParameter)
{
  EXPECT_EQ(bms_node::parse_byte_order("auto"), bms_node::ByteOrder::kAuto);
  EXPECT_EQ(
    bms_node::parse_byte_order("big_endian"), bms_node::ByteOrder::kBigEndian);
  EXPECT_EQ(
    bms_node::parse_byte_order("little_endian"), bms_node::ByteOrder::kLittleEndian);
  EXPECT_THROW(static_cast<void>(bms_node::parse_byte_order("native")), std::invalid_argument);
}

TEST(BmsProtocolTest, EqualSocBytesNeverChooseAnOrderOrMutateTheSample)
{
  for (const std::uint8_t repeated : std::array<std::uint8_t, 4U>{0U, 1U, 2U, 3U}) {
    bms_node::GoldenPhoenixDecoder decoder{bms_node::ByteOrder::kAuto, 0x01U, 0x40U};
    bms_node::BmsSample sample{48.0, 0.5, 2.0};
    const std::array<std::uint8_t, 8U> ambiguous{
      0xF4U, 0x01U, 0x00U, 0x00U, 0x30U, 0x75U, repeated, repeated};
    EXPECT_FALSE(decoder.ingest(sample, kResponseId, ambiguous.data(), ambiguous.size(), 3.0));
    EXPECT_FALSE(decoder.resolved_order());
    EXPECT_DOUBLE_EQ(*sample.voltage_v, 48.0);
    EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.5);
    EXPECT_DOUBLE_EQ(*sample.last_frame_s, 2.0);

    auto unique = ambiguous;
    unique[6] = 0x20U;
    unique[7] = 0x03U;
    ASSERT_TRUE(decoder.ingest(sample, kResponseId, unique.data(), unique.size(), 4.0));
    EXPECT_EQ(decoder.resolved_order(), bms_node::ByteOrder::kLittleEndian);
    EXPECT_DOUBLE_EQ(*sample.voltage_v, 50.0);
    EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.8);

    // Once uniquely resolved, repeated SOC bytes are ordinary valid samples.
    ASSERT_TRUE(decoder.ingest(sample, kResponseId, ambiguous.data(), ambiguous.size(), 5.0));
    EXPECT_DOUBLE_EQ(*sample.voltage_v, 50.0);
    EXPECT_DOUBLE_EQ(*sample.soc_fraction, static_cast<double>(repeated) * 257.0 / 1000.0);
  }
}

TEST(BmsProtocolTest, ExplicitByteOrdersStillAcceptZeroSoc)
{
  for (const auto order : {bms_node::ByteOrder::kBigEndian, bms_node::ByteOrder::kLittleEndian}) {
    bms_node::GoldenPhoenixDecoder decoder{order, 0x01U, 0x40U};
    bms_node::BmsSample sample;
    std::array<std::uint8_t, 8U> payload{0x01U, 0xF4U, 0U, 0U, 0U, 0U, 0U, 0U};
    if (order == bms_node::ByteOrder::kLittleEndian) {
      std::swap(payload[0], payload[1]);
    }
    ASSERT_TRUE(decoder.ingest(sample, kResponseId, payload.data(), payload.size(), 1.0));
    EXPECT_DOUBLE_EQ(*sample.voltage_v, 50.0);
    EXPECT_DOUBLE_EQ(*sample.soc_fraction, 0.0);
  }
}

}  // namespace
