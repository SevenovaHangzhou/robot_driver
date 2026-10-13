#include "modbus_tcp_rtu485/protocol/ultrasonic_protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace modbus_tcp_rtu485
{
namespace
{
struct DecodeCase
{
  uint16_t raw;
  float range_m;
  UltrasonicSeverity severity;
  const char * status;
};

class UltrasonicDecoder : public testing::TestWithParam<DecodeCase>
{
};

TEST_P(UltrasonicDecoder, PreservesDistanceAndFailureMeaning)
{
  const auto & expected = GetParam();
  const auto reading = decode_ultrasonic(expected.raw, 0.01F, 3.5F);
  EXPECT_EQ(reading.severity, expected.severity);
  EXPECT_EQ(reading.status, expected.status);
  if (std::isnan(expected.range_m)) {
    EXPECT_TRUE(std::isnan(reading.range_m));
  } else {
    EXPECT_FLOAT_EQ(reading.range_m, expected.range_m);
  }
}

// Existing test_modbus_transport vectors; no new device assumptions.
INSTANTIATE_TEST_SUITE_P(
  ExistingVectors, UltrasonicDecoder,
  testing::Values(
    DecodeCase{25, 0.025F, UltrasonicSeverity::ok, "ok"},
    DecodeCase{0xFFFD, std::numeric_limits<float>::infinity(), UltrasonicSeverity::ok, "no_target"},
    DecodeCase{0xFFFE, NAN, UltrasonicSeverity::warning, "interference"},
    DecodeCase{0xFFFF, NAN, UltrasonicSeverity::error, "sensor_timeout"},
    DecodeCase{0xEEEE, NAN, UltrasonicSeverity::error, "checksum_error"},
    DecodeCase{0, NAN, UltrasonicSeverity::warning, "out_of_range"}));
}  // namespace
}  // namespace modbus_tcp_rtu485
