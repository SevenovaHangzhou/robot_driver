#include "bms_node/protocol.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace bms_node
{
namespace
{

constexpr std::uint32_t kCanIdPrefix = 0x18000000U;
constexpr std::uint16_t kMaximumSocRaw = 1000U;

std::uint16_t decode_u16(const std::uint8_t * data, const ByteOrder order) noexcept
{
  if (order == ByteOrder::kLittleEndian) {
    return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(data[0]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U));
  }
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[0]) << 8U) |
    static_cast<std::uint16_t>(data[1]));
}

std::optional<ByteOrder> detect_order(const std::uint8_t * data) noexcept
{
  const auto big_soc = decode_u16(data + 6U, ByteOrder::kBigEndian);
  const auto little_soc = decode_u16(data + 6U, ByteOrder::kLittleEndian);
  const bool big_valid = big_soc <= kMaximumSocRaw;
  const bool little_valid = little_soc <= kMaximumSocRaw;
  if (big_valid != little_valid) {
    return big_valid ? ByteOrder::kBigEndian : ByteOrder::kLittleEndian;
  }
  if (big_valid && big_soc == little_soc) {
    return ByteOrder::kBigEndian;
  }
  return std::nullopt;
}

}  // namespace

bool BmsSample::is_fresh(const double now_s, const double timeout_s) const noexcept
{
  if (!last_frame_s || !std::isfinite(*last_frame_s) || !std::isfinite(now_s) ||
    !std::isfinite(timeout_s) || timeout_s <= 0.0)
  {
    return false;
  }
  const double age_s = now_s - *last_frame_s;
  return age_s >= 0.0 && age_s <= timeout_s;
}

std::uint32_t make_can_id(
  const std::uint8_t data_id, const std::uint8_t destination,
  const std::uint8_t source) noexcept
{
  return kCanIdPrefix | (static_cast<std::uint32_t>(data_id) << 16U) |
         (static_cast<std::uint32_t>(destination) << 8U) |
         static_cast<std::uint32_t>(source);
}

ByteOrder parse_byte_order(const std::string_view value)
{
  if (value == "auto") {
    return ByteOrder::kAuto;
  }
  if (value == "big_endian") {
    return ByteOrder::kBigEndian;
  }
  if (value == "little_endian") {
    return ByteOrder::kLittleEndian;
  }
  throw std::invalid_argument{
          "multi_byte_order must be auto, big_endian, or little_endian; got " +
          std::string{value}};
}

GoldenPhoenixDecoder::GoldenPhoenixDecoder(
  const ByteOrder configured_order, const std::uint8_t bms_address,
  const std::uint8_t host_address)
: configured_order_{configured_order},
  response_id_{make_can_id(kTotalStatusDataId, host_address, bms_address)}
{
  if (configured_order_ != ByteOrder::kAuto) {
    resolved_order_ = configured_order_;
  }
}

bool GoldenPhoenixDecoder::ingest(
  BmsSample & sample, const std::uint32_t can_id, const std::uint8_t * data,
  const std::size_t size, const double now_s)
{
  if (can_id != response_id_ || data == nullptr || size != 8U || !std::isfinite(now_s)) {
    return false;
  }

  auto order = resolved_order_;
  if (!order) {
    order = detect_order(data);
    if (!order) {
      return false;
    }
  }

  const std::uint16_t voltage_raw = decode_u16(data, *order);
  const std::uint16_t soc_raw = decode_u16(data + 6U, *order);
  if (soc_raw > kMaximumSocRaw) {
    return false;
  }

  sample.voltage_v = static_cast<double>(voltage_raw) * 0.1;
  sample.soc_fraction = static_cast<double>(soc_raw) / 1000.0;
  sample.last_frame_s = now_s;
  resolved_order_ = order;
  return true;
}

std::optional<ByteOrder> GoldenPhoenixDecoder::resolved_order() const noexcept
{
  return resolved_order_;
}

BatteryValues project_battery_values(
  const BmsSample & sample, const double now_s, const double timeout_s) noexcept
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!sample.is_fresh(now_s, timeout_s) || !sample.voltage_v || !sample.soc_fraction ||
    !std::isfinite(*sample.voltage_v) || !std::isfinite(*sample.soc_fraction) ||
    *sample.soc_fraction < 0.0 || *sample.soc_fraction > 1.0)
  {
    return {nan, nan};
  }
  return {*sample.voltage_v, *sample.soc_fraction};
}

}  // namespace bms_node
