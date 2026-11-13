#ifndef PERIPH_BMS__PROTOCOL_HPP_
#define PERIPH_BMS__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <array>
#include <optional>
#include <string_view>

namespace periph_bms
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

enum class ByteOrder {
  kAuto,
  kBigEndian,
  kLittleEndian,
};

// Severity the BMS itself assigns in 0x98. The vendor document names two warning levels but does not
// state which is more severe; "first" is treated as the less severe one.
enum class AlarmLevel : std::uint8_t {
  kFirst,
  kSecond,
  kFault,
};

// What the alarm is about. Temperature alarms do not invalidate the published measurements; the other
// kinds still do, because they may accompany an unreliable pack.
enum class AlarmKind : std::uint8_t {
  kTemperatureHigh,
  kTemperatureLow,
  kTemperatureDelta,
  kOther,
};

// One bit of the 0x98 fault frame, transcribed from the vendor document (V3-DOC-2026-005, pages 6-8).
// `code` is the stable identifier; display wording belongs to the consumer.
struct AlarmSpec
{
  std::uint8_t byte;
  std::uint8_t bit;
  AlarmLevel level;
  AlarmKind kind;
  std::string_view code;
};

inline constexpr std::array<AlarmSpec, 51U> kAlarmSpecs{{
  // Byte 0: cell and pack voltage, SOC is not here but byte 2 carries it.
  {0U, 0U, AlarmLevel::kFirst, AlarmKind::kOther, "cell_over_voltage_first"},
  {0U, 1U, AlarmLevel::kSecond, AlarmKind::kOther, "cell_over_voltage_second"},
  {0U, 2U, AlarmLevel::kFirst, AlarmKind::kOther, "cell_under_voltage_first"},
  {0U, 3U, AlarmLevel::kSecond, AlarmKind::kOther, "cell_under_voltage_second"},
  {0U, 4U, AlarmLevel::kFirst, AlarmKind::kOther, "pack_over_voltage_first"},
  {0U, 5U, AlarmLevel::kSecond, AlarmKind::kOther, "pack_over_voltage_second"},
  {0U, 6U, AlarmLevel::kFirst, AlarmKind::kOther, "pack_under_voltage_first"},
  {0U, 7U, AlarmLevel::kSecond, AlarmKind::kOther, "pack_under_voltage_second"},
  // Byte 1: charge and discharge temperature.
  {1U, 0U, AlarmLevel::kFirst, AlarmKind::kTemperatureHigh, "charge_over_temperature_first"},
  {1U, 1U, AlarmLevel::kSecond, AlarmKind::kTemperatureHigh, "charge_over_temperature_second"},
  {1U, 2U, AlarmLevel::kFirst, AlarmKind::kTemperatureLow, "charge_under_temperature_first"},
  {1U, 3U, AlarmLevel::kSecond, AlarmKind::kTemperatureLow, "charge_under_temperature_second"},
  {1U, 4U, AlarmLevel::kFirst, AlarmKind::kTemperatureHigh, "discharge_over_temperature_first"},
  {1U, 5U, AlarmLevel::kSecond, AlarmKind::kTemperatureHigh, "discharge_over_temperature_second"},
  {1U, 6U, AlarmLevel::kFirst, AlarmKind::kTemperatureLow, "discharge_under_temperature_first"},
  {1U, 7U, AlarmLevel::kSecond, AlarmKind::kTemperatureLow, "discharge_under_temperature_second"},
  // Byte 2: current and SOC.
  {2U, 0U, AlarmLevel::kFirst, AlarmKind::kOther, "charge_over_current_first"},
  {2U, 1U, AlarmLevel::kSecond, AlarmKind::kOther, "charge_over_current_second"},
  {2U, 2U, AlarmLevel::kFirst, AlarmKind::kOther, "discharge_over_current_first"},
  {2U, 3U, AlarmLevel::kSecond, AlarmKind::kOther, "discharge_over_current_second"},
  {2U, 4U, AlarmLevel::kFirst, AlarmKind::kOther, "soc_over_first"},
  {2U, 5U, AlarmLevel::kSecond, AlarmKind::kOther, "soc_over_second"},
  {2U, 6U, AlarmLevel::kFirst, AlarmKind::kOther, "soc_under_first"},
  {2U, 7U, AlarmLevel::kSecond, AlarmKind::kOther, "soc_under_second"},
  // Byte 3: voltage and temperature spread; bits 4-7 are reserved.
  {3U, 0U, AlarmLevel::kFirst, AlarmKind::kOther, "voltage_delta_first"},
  {3U, 1U, AlarmLevel::kSecond, AlarmKind::kOther, "voltage_delta_second"},
  {3U, 2U, AlarmLevel::kFirst, AlarmKind::kTemperatureDelta, "temperature_delta_first"},
  {3U, 3U, AlarmLevel::kSecond, AlarmKind::kTemperatureDelta, "temperature_delta_second"},
  // Byte 4: MOS temperature, sensors and contacts.
  {4U, 0U, AlarmLevel::kFirst, AlarmKind::kTemperatureHigh, "charge_mos_over_temperature"},
  {4U, 1U, AlarmLevel::kFirst, AlarmKind::kTemperatureHigh, "discharge_mos_over_temperature"},
  {4U, 2U, AlarmLevel::kFault, AlarmKind::kOther, "charge_mos_temperature_sensor_fault"},
  {4U, 3U, AlarmLevel::kFault, AlarmKind::kOther, "discharge_mos_temperature_sensor_fault"},
  {4U, 4U, AlarmLevel::kFault, AlarmKind::kOther, "charge_mos_welded"},
  {4U, 5U, AlarmLevel::kFault, AlarmKind::kOther, "discharge_mos_welded"},
  {4U, 6U, AlarmLevel::kFault, AlarmKind::kOther, "charge_mos_open"},
  {4U, 7U, AlarmLevel::kFault, AlarmKind::kOther, "discharge_mos_open"},
  // Byte 5: acquisition and internal modules.
  {5U, 0U, AlarmLevel::kFault, AlarmKind::kOther, "afe_fault"},
  {5U, 1U, AlarmLevel::kFault, AlarmKind::kOther, "cell_acquisition_offline"},
  {5U, 2U, AlarmLevel::kFault, AlarmKind::kOther, "cell_temperature_sensor_fault"},
  {5U, 3U, AlarmLevel::kFault, AlarmKind::kOther, "eeprom_fault"},
  {5U, 4U, AlarmLevel::kFault, AlarmKind::kOther, "rtc_fault"},
  {5U, 5U, AlarmLevel::kFault, AlarmKind::kOther, "precharge_failed"},
  {5U, 6U, AlarmLevel::kFault, AlarmKind::kOther, "vehicle_communication_fault"},
  {5U, 7U, AlarmLevel::kFault, AlarmKind::kOther, "internal_network_fault"},
  // Byte 6: power path; bit 7 is reserved.
  {6U, 0U, AlarmLevel::kFault, AlarmKind::kOther, "current_module_fault"},
  {6U, 1U, AlarmLevel::kFault, AlarmKind::kOther, "pack_voltage_module_fault"},
  {6U, 2U, AlarmLevel::kFault, AlarmKind::kOther, "short_circuit_fault"},
  {6U, 3U, AlarmLevel::kFault, AlarmKind::kOther, "low_voltage_charge_inhibited"},
  {6U, 4U, AlarmLevel::kFault, AlarmKind::kOther, "mos_opened_by_gps_or_soft_switch"},
  {6U, 5U, AlarmLevel::kFault, AlarmKind::kOther, "charge_cabinet_offline"},
  {6U, 6U, AlarmLevel::kFault, AlarmKind::kOther, "thermal_runaway"},
}};

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
  std::uint8_t primary_address, std::uint8_t secondary_address, std::uint8_t host_address) noexcept;

[[nodiscard]] ByteOrder parse_byte_order(std::string_view value);

class GoldenPhoenixDecoder
{
public:
  GoldenPhoenixDecoder(
    ByteOrder configured_order, std::uint8_t bms_address, std::uint8_t host_address);

  [[nodiscard]] bool ingest(
    BmsSample & sample, std::uint32_t can_id, const std::uint8_t * data, std::size_t size,
    double now_s);

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
  std::optional<double> mos_temperature_c;
  std::optional<double> board_temperature_c;
  std::array<std::optional<double>, 48U> cell_voltage_v{};
  std::array<std::optional<double>, 16U> cell_frame_s{};

  // Fills `out` with the alarms currently set in the last 0x98 frame and returns how many there are.
  [[nodiscard]] std::size_t active_alarms(
    std::array<const AlarmSpec *, kAlarmSpecs.size()> & out) const noexcept;
  [[nodiscard]] bool has_alarm_of_kind(AlarmKind kind) const noexcept;
  [[nodiscard]] bool has_warning_level(AlarmLevel level) const noexcept;
  // Vendor fault number in 0x98 byte 7; the vendor document does not define the code list.
  [[nodiscard]] std::uint8_t vendor_fault_code() const noexcept;
  // Temperature alarms do not invalidate the measurements; every other alarm still does.
  [[nodiscard]] bool has_temperature_alarm() const noexcept;
  [[nodiscard]] bool has_non_temperature_alarm() const noexcept;

  [[nodiscard]] bool has_fault() const noexcept;
  [[nodiscard]] bool status_fresh(double now_s, double timeout_s) const noexcept;
  [[nodiscard]] bool cells_fresh(double now_s, double timeout_s) const noexcept;
};

class GoldenPhoenixPack
{
public:
  GoldenPhoenixPack(ByteOrder order, std::uint8_t bms_address, std::uint8_t host_address);

  [[nodiscard]] bool ingest(
    PackState & state, std::uint32_t can_id, const std::uint8_t * data, std::size_t size,
    double now_s);
  void begin_cell_scan(PackState & state) const noexcept;

private:
  GoldenPhoenixDecoder total_decoder_;
  std::uint8_t bms_address_;
  std::uint8_t host_address_;
};

[[nodiscard]] BatteryValues project_battery_values(
  const BmsSample & sample, double now_s, double timeout_s) noexcept;

}  // namespace periph_bms

#endif  // PERIPH_BMS__PROTOCOL_HPP_
