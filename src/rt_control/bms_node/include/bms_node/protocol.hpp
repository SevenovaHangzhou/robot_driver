#ifndef BMS_NODE__PROTOCOL_HPP_
#define BMS_NODE__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace bms_node
{

constexpr std::uint8_t kTotalStatusDataId = 0x90U;
constexpr std::uint8_t kDefaultBmsAddress = 0x01U;
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

  [[nodiscard]] bool is_fresh(double now_s, double timeout_s) const noexcept;
};

struct BatteryValues
{
  double voltage_v;
  double soc_fraction;
};

[[nodiscard]] std::uint32_t make_can_id(
  std::uint8_t data_id, std::uint8_t destination, std::uint8_t source) noexcept;

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

[[nodiscard]] BatteryValues project_battery_values(
  const BmsSample & sample, double now_s, double timeout_s) noexcept;

}  // namespace bms_node

#endif  // BMS_NODE__PROTOCOL_HPP_
