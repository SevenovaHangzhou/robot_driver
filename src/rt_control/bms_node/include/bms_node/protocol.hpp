#ifndef BMS_NODE__PROTOCOL_HPP_
#define BMS_NODE__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <string_view>

namespace bms_node
{

constexpr std::uint8_t kTotalStatusDataId = 0x90U;
constexpr std::uint8_t kCellExtremesDataId = 0x91U;
constexpr std::uint8_t kCellTemperatureDataId = 0x92U;
constexpr std::uint8_t kMosStatusDataId = 0x93U;
constexpr std::uint8_t kPackInfoDataId = 0x94U;
constexpr std::uint8_t kCellVoltagesDataId = 0x95U;
constexpr std::uint8_t kFaultDataId = 0x98U;
constexpr std::uint8_t kDischargeMosDataId = 0xD9U;
constexpr std::uint8_t kDefaultBmsAddress = 0x01U;
constexpr std::uint8_t kSecondaryBmsAddress = 0x02U;
constexpr std::uint8_t kDefaultHostAddress = 0x40U;

enum class ByteOrder
{
  kAuto,
  kBigEndian,
  kLittleEndian,
};

struct BmsSample
{
  std::optional<double> voltage_v;
  std::optional<double> soc_fraction;
  std::optional<double> last_frame_s;
  std::optional<double> current_a{std::nullopt};

  [[nodiscard]] bool is_fresh(double now_s, double timeout_s) const noexcept;
};

struct BatteryValues
{
  double voltage_v;
  double soc_fraction;
};

[[nodiscard]] std::uint32_t make_can_id(
  std::uint8_t data_id, std::uint8_t destination, std::uint8_t source) noexcept;

[[nodiscard]] std::array<std::uint32_t, 14U> make_dual_poll_ids(
  std::uint8_t primary_address, std::uint8_t secondary_address,
  std::uint8_t host_address) noexcept;

[[nodiscard]] ByteOrder parse_byte_order(std::string_view value);

class GoldenPhoenixDecoder
{
public:
  GoldenPhoenixDecoder(
    ByteOrder configured_order, std::uint8_t bms_address, std::uint8_t host_address);

  [[nodiscard]] bool ingest(
    BmsSample & sample, std::uint32_t can_id, const std::uint8_t * data,
    std::size_t size, double now_s);

  [[nodiscard]] std::optional<ByteOrder> resolved_order() const noexcept;

private:
  ByteOrder configured_order_;
  std::uint32_t response_id_;
  std::optional<ByteOrder> resolved_order_;
};

struct PackState
{
  BmsSample sample;
  std::optional<double> fault_frame_s;
  std::array<std::uint8_t, 8U> fault_bytes{};
  std::optional<double> mos_frame_s;
  std::optional<std::uint8_t> operating_state_raw;
  std::optional<std::uint8_t> charge_mos_raw;
  std::optional<std::uint8_t> discharge_mos_raw;
  std::optional<double> pack_info_s;
  std::optional<std::uint8_t> charger_connected_raw;
  std::optional<double> discharge_ack_s;
  std::optional<std::uint8_t> discharge_ack_value;
  std::optional<std::uint8_t> cell_count;
  std::optional<double> cell_extremes_s;
  std::optional<double> max_cell_voltage_v;
  std::optional<double> min_cell_voltage_v;
  std::optional<double> temperature_frame_s;
  std::optional<double> max_cell_temperature_c;
  std::optional<double> min_cell_temperature_c;
  std::array<std::optional<double>, 48U> cell_voltage_v{};
  std::array<std::optional<double>, 16U> cell_frame_s{};

  [[nodiscard]] bool has_fault() const noexcept;
  [[nodiscard]] bool status_fresh(double now_s, double timeout_s) const noexcept;
  [[nodiscard]] bool cells_fresh(double now_s, double timeout_s) const noexcept;
};

class GoldenPhoenixPack
{
public:
  GoldenPhoenixPack(ByteOrder order, std::uint8_t bms_address, std::uint8_t host_address);

  [[nodiscard]] bool ingest(
    PackState & state, std::uint32_t can_id, const std::uint8_t * data,
    std::size_t size, double now_s);
  void begin_cell_scan(PackState & state) const noexcept;

private:
  GoldenPhoenixDecoder total_decoder_;
  std::uint8_t bms_address_;
  std::uint8_t host_address_;
};

[[nodiscard]] BatteryValues project_battery_values(
  const BmsSample & sample, double now_s, double timeout_s) noexcept;

}  // namespace bms_node

#endif  // BMS_NODE__PROTOCOL_HPP_
