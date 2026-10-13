#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace modbus_tcp_rtu485
{
enum class UltrasonicSeverity { ok, warning, error };

struct UltrasonicReading
{
  float range_m;
  UltrasonicSeverity severity;
  std::string status;
};

UltrasonicReading decode_ultrasonic(uint16_t raw, float min_range_m, float max_range_m);
}  // namespace modbus_tcp_rtu485
