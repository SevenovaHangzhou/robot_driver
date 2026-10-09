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

std::array<std::uint32_t, 14U> make_dual_poll_ids(
  const std::uint8_t primary_address, const std::uint8_t secondary_address,
  const std::uint8_t host_address) noexcept
{
  constexpr std::array<std::uint8_t, 7U> data_ids{
    kTotalStatusDataId, kCellExtremesDataId, kCellTemperatureDataId, kMosStatusDataId,
    kPackInfoDataId, kCellVoltagesDataId, kFaultDataId};
  std::array<std::uint32_t, 14U> ids{};
  for (std::size_t index = 0U; index < data_ids.size(); ++index) {
    ids[index] = make_can_id(data_ids[index], primary_address, host_address);
    ids[index + data_ids.size()] =
      make_can_id(data_ids[index], secondary_address, host_address);
  }
  return ids;
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
  sample.current_a =
    (static_cast<double>(decode_u16(data + 4U, *order)) - 30000.0) * 0.1;
  sample.last_frame_s = now_s;
  resolved_order_ = order;
  return true;
}

std::optional<ByteOrder> GoldenPhoenixDecoder::resolved_order() const noexcept
{
  return resolved_order_;
}

namespace
{
bool fresh_time(const std::optional<double> & stamp, const double now, const double timeout) noexcept
{
  if (!stamp || !std::isfinite(*stamp) || !std::isfinite(now) ||
    !std::isfinite(timeout) || timeout <= 0.0)
  {
    return false;
  }
  const double age = now - *stamp;
  return age >= 0.0 && age <= timeout;
}
}  // namespace

bool PackState::has_fault() const noexcept
{
  if (!fault_frame_s) {
    return false;
  }
  for (std::size_t index = 0U; index < fault_bytes.size(); ++index) {
    const std::uint8_t mask = index == 3U ? 0x0FU : (index == 6U ? 0x7FU : 0xFFU);
    if ((fault_bytes[index] & mask) != 0U) {
      return true;
    }
  }
  return false;
}

bool PackState::status_fresh(const double now_s, const double timeout_s) const noexcept
{
  return sample.is_fresh(now_s, timeout_s) &&
         fresh_time(fault_frame_s, now_s, timeout_s) &&
         fresh_time(mos_frame_s, now_s, timeout_s);
}

bool PackState::cells_fresh(const double now_s, const double timeout_s) const noexcept
{
  if (!cell_count || *cell_count == 0U || *cell_count > 48U) {
    return false;
  }
  for (std::size_t index = 0U; index < *cell_count; ++index) {
    if (!cell_voltage_v[index] || !fresh_time(cell_frame_s[index / 3U], now_s, timeout_s)) {
      return false;
    }
  }
  return true;
}

GoldenPhoenixPack::GoldenPhoenixPack(
  const ByteOrder order, const std::uint8_t bms_address,
  const std::uint8_t host_address)
: total_decoder_{order, bms_address, host_address},
  bms_address_{bms_address}, host_address_{host_address}
{
}

void GoldenPhoenixPack::begin_cell_scan(PackState & state) const noexcept
{
  state.cell_voltage_v.fill(std::nullopt);
  state.cell_frame_s.fill(std::nullopt);
}

bool GoldenPhoenixPack::ingest(
  PackState & state, const std::uint32_t can_id, const std::uint8_t * data,
  const std::size_t size, const double now_s)
{
  if (data == nullptr || size != 8U || !std::isfinite(now_s)) {
    return false;
  }
  if (can_id == make_can_id(kTotalStatusDataId, host_address_, bms_address_)) {
    return total_decoder_.ingest(state.sample, can_id, data, size, now_s);
  }
  if (can_id == make_can_id(kFaultDataId, host_address_, bms_address_)) {
    for (std::size_t index = 0U; index < state.fault_bytes.size(); ++index) {
      state.fault_bytes[index] = data[index];
    }
    state.fault_frame_s = now_s;
    return true;
  }
  if (can_id == make_can_id(kMosStatusDataId, host_address_, bms_address_)) {
    state.operating_state_raw = data[0U];
    state.charge_mos_raw = data[1U];
    state.discharge_mos_raw = data[2U];
    state.mos_frame_s = now_s;
    return true;
  }
  if (can_id == make_can_id(kCellTemperatureDataId, host_address_, bms_address_)) {
    state.max_cell_temperature_c = static_cast<double>(data[0U]) - 40.0;
    state.min_cell_temperature_c = static_cast<double>(data[2U]) - 40.0;
    state.temperature_frame_s = now_s;
    return true;
  }
  if (can_id == make_can_id(kDischargeMosDataId, host_address_, bms_address_)) {
    if (data[0U] > 1U) {
      return false;
    }
    state.discharge_ack_value = data[0U];
    state.discharge_ack_s = now_s;
    return true;
  }
  if (can_id == make_can_id(kPackInfoDataId, host_address_, bms_address_)) {
    if (data[0U] == 0U || data[0U] > 48U) {
      return false;
    }
    if (state.cell_count != data[0U]) {
      begin_cell_scan(state);
    }
    state.cell_count = data[0U];
    state.charger_connected_raw = data[2U];
    state.pack_info_s = now_s;
    return true;
  }
  const auto order = total_decoder_.resolved_order();
  if (!order) {
    return false;
  }
  if (can_id == make_can_id(kCellExtremesDataId, host_address_, bms_address_)) {
    state.max_cell_voltage_v = static_cast<double>(decode_u16(data, *order)) / 1000.0;
    state.min_cell_voltage_v = static_cast<double>(decode_u16(data + 3U, *order)) / 1000.0;
    state.cell_extremes_s = now_s;
    return true;
  }
  if (can_id == make_can_id(kCellVoltagesDataId, host_address_, bms_address_)) {
    const auto frame_index = data[0U];
    if (frame_index >= state.cell_frame_s.size()) {
      return false;
    }
    for (std::size_t offset = 0U; offset < 3U; ++offset) {
      const std::size_t cell_index = static_cast<std::size_t>(frame_index) * 3U + offset;
      state.cell_voltage_v[cell_index] =
        static_cast<double>(decode_u16(data + 1U + offset * 2U, *order)) / 1000.0;
    }
    state.cell_frame_s[frame_index] = now_s;
    return true;
  }
  return false;
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
