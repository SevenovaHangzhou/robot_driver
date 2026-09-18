#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace modbus_tcp_rtu485
{
struct UltrasonicReading
{
  float range_m;
  uint8_t diagnostic_level;
  std::string status;
};

inline UltrasonicReading decode_ultrasonic(
  uint16_t raw, float min_range_m, float max_range_m)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  switch (raw) {
    case 0xFFFD:
      return {std::numeric_limits<float>::infinity(), DiagnosticStatus::WARN, "no_target"};
    case 0xFFFE:
      return {std::numeric_limits<float>::quiet_NaN(), DiagnosticStatus::WARN, "interference"};
    case 0xFFFF:
      return {std::numeric_limits<float>::quiet_NaN(), DiagnosticStatus::ERROR, "sensor_timeout"};
    case 0xEEEE:
      return {std::numeric_limits<float>::quiet_NaN(), DiagnosticStatus::ERROR, "checksum_error"};
    default:
      break;
  }

  const float range_m = static_cast<float>(raw) / 1000.0F;
  if (range_m < min_range_m || range_m > max_range_m) {
    return {std::numeric_limits<float>::quiet_NaN(), DiagnosticStatus::WARN, "out_of_range"};
  }
  return {range_m, DiagnosticStatus::OK, "ok"};
}
}  // namespace modbus_tcp_rtu485
