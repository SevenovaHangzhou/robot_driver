#include "lpms_nav3_can/decoder.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace lpms_nav3_can
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMicroteslaToTesla = 1.0e-6;
constexpr std::uint8_t kAllPdosReceived = 0x0fU;

[[nodiscard]] std::int32_t read_signed_le16(
  const std::array<std::uint8_t, 8U> & data, const std::size_t offset) noexcept
{
  const auto raw = static_cast<std::uint32_t>(data[offset]) |
    (static_cast<std::uint32_t>(data[offset + 1U]) << 8U);
  return (raw & 0x8000U) == 0U ?
         static_cast<std::int32_t>(raw) :
         static_cast<std::int32_t>(raw) - 0x10000;
}

[[nodiscard]] double scaled_value(
  const std::array<std::uint8_t, 8U> & data, const std::size_t value_index,
  const double scale) noexcept
{
  return static_cast<double>(read_signed_le16(data, value_index * 2U)) * scale;
}

}  // namespace

Decoder::Decoder(const std::uint8_t node_id)
: node_id_(node_id)
{
  if (node_id_ == 0U || node_id_ > 127U) {
    throw std::invalid_argument{"CANopen node_id must be in [1, 127]"};
  }
}

DecodeResult Decoder::consume(const CanFrame & frame)
{
  DecodeResult result;
  if (frame.is_extended || frame.is_remote || frame.is_error) {
    return result;
  }

  const auto node = static_cast<std::uint32_t>(node_id_);
  if (frame.id == 0x700U + node) {
    result.recognized = true;
    const auto payload_size = static_cast<std::size_t>(frame.dlc);
    if (payload_size < 1U || payload_size > frame.data.size()) {
      result.malformed = true;
      return result;
    }
    for (std::size_t index = 1U; index < payload_size; ++index) {
      if (frame.data[index] != 0U) {
        result.malformed = true;
        return result;
      }
    }
    result.heartbeat_state = frame.data[0];
    return result;
  }

  std::uint8_t pdo_bit{0U};
  if (frame.id == 0x180U + node) {
    pdo_bit = 0x01U;
  } else if (frame.id == 0x280U + node) {
    pdo_bit = 0x02U;
  } else if (frame.id == 0x380U + node) {
    pdo_bit = 0x04U;
  } else if (frame.id == 0x480U + node) {
    pdo_bit = 0x08U;
  } else {
    return result;
  }

  result.recognized = true;
  if (frame.dlc != 8U) {
    result.malformed = true;
    return result;
  }

  if (pdo_bit == 0x01U) {
    constexpr double acceleration_scale = 0.001 * kStandardGravityMps2;
    constexpr double angular_velocity_scale = 0.1 * kDegreesToRadians;
    sample_.linear_acceleration_mps2[0] = scaled_value(frame.data, 0U, acceleration_scale);
    sample_.linear_acceleration_mps2[1] = scaled_value(frame.data, 1U, acceleration_scale);
    sample_.linear_acceleration_mps2[2] = scaled_value(frame.data, 2U, acceleration_scale);
    sample_.angular_velocity_radps[0] = scaled_value(frame.data, 3U, angular_velocity_scale);
  } else if (pdo_bit == 0x02U) {
    constexpr double angular_velocity_scale = 0.1 * kDegreesToRadians;
    constexpr double magnetic_field_scale = 0.01 * kMicroteslaToTesla;
    sample_.angular_velocity_radps[1] = scaled_value(frame.data, 0U, angular_velocity_scale);
    sample_.angular_velocity_radps[2] = scaled_value(frame.data, 1U, angular_velocity_scale);
    sample_.magnetic_field_t[0] = scaled_value(frame.data, 2U, magnetic_field_scale);
    sample_.magnetic_field_t[1] = scaled_value(frame.data, 3U, magnetic_field_scale);
  } else if (pdo_bit == 0x04U) {
    constexpr double magnetic_field_scale = 0.01 * kMicroteslaToTesla;
    constexpr double euler_scale = 0.01 * kDegreesToRadians;
    sample_.magnetic_field_t[2] = scaled_value(frame.data, 0U, magnetic_field_scale);
    sample_.euler_rad[0] = scaled_value(frame.data, 1U, euler_scale);
    sample_.euler_rad[1] = scaled_value(frame.data, 2U, euler_scale);
    sample_.euler_rad[2] = scaled_value(frame.data, 3U, euler_scale);
  } else {
    constexpr double quaternion_scale = 0.0001;
    sample_.orientation_wxyz[0] = scaled_value(frame.data, 0U, quaternion_scale);
    sample_.orientation_wxyz[1] = scaled_value(frame.data, 1U, quaternion_scale);
    sample_.orientation_wxyz[2] = scaled_value(frame.data, 2U, quaternion_scale);
    sample_.orientation_wxyz[3] = scaled_value(frame.data, 3U, quaternion_scale);
  }

  pending_mask_ = static_cast<std::uint8_t>(pending_mask_ | pdo_bit);
  if (pending_mask_ == kAllPdosReceived) {
    result.sample = sample_;
    pending_mask_ = 0U;
  }
  return result;
}

void Decoder::reset() noexcept
{
  pending_mask_ = 0U;
  sample_ = ImuSample{};
}

std::uint8_t Decoder::node_id() const noexcept
{
  return node_id_;
}

std::uint8_t Decoder::pending_mask() const noexcept
{
  return pending_mask_;
}

ImuSample convert_to_ros_convention(const ImuSample & sample, const bool convert_to_ros)
{
  if (!convert_to_ros) {
    return sample;
  }

  auto converted = sample;
  for (auto & acceleration : converted.linear_acceleration_mps2) {
    acceleration = -acceleration;
  }
  converted.orientation_wxyz[1] = -converted.orientation_wxyz[1];
  converted.orientation_wxyz[2] = -converted.orientation_wxyz[2];
  converted.orientation_wxyz[3] = -converted.orientation_wxyz[3];
  return converted;
}

}  // namespace lpms_nav3_can
