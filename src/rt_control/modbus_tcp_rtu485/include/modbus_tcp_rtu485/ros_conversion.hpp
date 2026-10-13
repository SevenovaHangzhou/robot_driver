#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "modbus_tcp_rtu485/protocol/ultrasonic_protocol.hpp"

namespace modbus_tcp_rtu485
{
uint8_t diagnostic_level(const UltrasonicReading & reading);

inline constexpr float kA22FieldOfViewRad = 0.6981317008F;
inline constexpr float kA22MaxRangeM = 3.5F;
inline constexpr size_t kUltrasonicChannelCount = 8U;

void validate_e08_unit_ids(const std::vector<int64_t> & unit_ids);

void validate_ultrasonic_config(
  const std::vector<std::string> & frame_ids, float field_of_view_rad, float min_range_m,
  float max_range_m);

sensor_msgs::msg::Range make_range_message(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  const UltrasonicReading & reading, float field_of_view_rad, float min_range_m, float max_range_m);
}  // namespace modbus_tcp_rtu485
