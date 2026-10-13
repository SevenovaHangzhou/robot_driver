#include "modbus_tcp_rtu485/protocol/ultrasonic_protocol.hpp"

namespace modbus_tcp_rtu485
{
UltrasonicReading decode_ultrasonic(uint16_t raw, float min_range_m, float max_range_m)
{
  switch (raw) {
    case 0xFFFD:
      return {std::numeric_limits<float>::infinity(), UltrasonicSeverity::ok, "no_target"};
    case 0xFFFE:
      return {std::numeric_limits<float>::quiet_NaN(), UltrasonicSeverity::warning, "interference"};
    case 0xFFFF:
      return {std::numeric_limits<float>::quiet_NaN(), UltrasonicSeverity::error, "sensor_timeout"};
    case 0xEEEE:
      return {std::numeric_limits<float>::quiet_NaN(), UltrasonicSeverity::error, "checksum_error"};
    default:
      break;
  }

  const float range_m = static_cast<float>(raw) / 1000.0F;
  if (range_m < min_range_m || range_m > max_range_m) {
    return {std::numeric_limits<float>::quiet_NaN(), UltrasonicSeverity::warning, "out_of_range"};
  }
  return {range_m, UltrasonicSeverity::ok, "ok"};
}
}  // namespace modbus_tcp_rtu485
