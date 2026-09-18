#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ultrasonic_protocol.hpp"

namespace modbus_tcp_rtu485
{
inline constexpr float kA22FieldOfViewRad = 1.0471975512F;
inline constexpr float kA22MaxRangeM = 3.5F;

inline void validate_ultrasonic_config(
  const std::vector<std::string> & frame_ids, float field_of_view_rad,
  float min_range_m, float max_range_m)
{
  if (frame_ids.size() != 4U) {
    throw std::invalid_argument("frame_ids requires four values");
  }
  for (const auto & frame_id : frame_ids) {
    if (frame_id.empty() || frame_id.front() == '/') {
      throw std::invalid_argument("frame_ids must be non-empty relative frame names");
    }
  }
  auto sorted_frame_ids = frame_ids;
  std::sort(sorted_frame_ids.begin(), sorted_frame_ids.end());
  if (std::adjacent_find(sorted_frame_ids.begin(), sorted_frame_ids.end()) !=
    sorted_frame_ids.end())
  {
    throw std::invalid_argument("frame_ids must be unique");
  }
  if (!std::isfinite(field_of_view_rad) ||
    std::fabs(field_of_view_rad - kA22FieldOfViewRad) > 1.0e-6F)
  {
    throw std::invalid_argument("field_of_view_rad must be 1.0471975512 (60 degrees)");
  }
  if (!std::isfinite(min_range_m) || min_range_m <= 0.0F) {
    throw std::invalid_argument("min_range_m must be finite and greater than zero");
  }
  if (!std::isfinite(max_range_m) || std::fabs(max_range_m - kA22MaxRangeM) > 1.0e-6F ||
    max_range_m <= min_range_m)
  {
    throw std::invalid_argument("max_range_m must be 3.5 and greater than min_range_m");
  }
}

inline sensor_msgs::msg::Range make_range_message(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  const UltrasonicReading & reading, float field_of_view_rad,
  float min_range_m, float max_range_m)
{
  sensor_msgs::msg::Range message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  message.field_of_view = field_of_view_rad;
  message.min_range = min_range_m;
  message.max_range = max_range_m;
  message.range = reading.range_m;
  return message;
}
}  // namespace modbus_tcp_rtu485
