#include <gtest/gtest.h>

#include "bms_node/protocol.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

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

}  // namespace
