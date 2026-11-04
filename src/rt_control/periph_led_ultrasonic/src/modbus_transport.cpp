#include "periph_led_ultrasonic/protocol/modbus_transport.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace periph_led_ultrasonic
{
void validate_endpoint(int64_t port, int64_t unit, int64_t timeout_ms)
{
  if (port < 1 || port > 65535) {
    throw std::invalid_argument("gateway port must be 1..65535");
  }
  if (unit < 1 || unit > 247) {
    throw std::invalid_argument("Modbus unit address must be 1..247");
  }
  if (timeout_ms < 1 || timeout_ms > 60000) {
    throw std::invalid_argument("response_timeout_ms must be 1..60000");
  }
}

void validate_led_config(
  const std::vector<int64_t> & ports, const std::vector<int64_t> & addresses, int64_t timeout_ms)
{
  if (ports.size() != kLedControllerCount || addresses.size() != kLedControllerCount) {
    throw std::invalid_argument("gateway_ports and controller_addresses require six values");
  }
  for (size_t i = 0; i < kLedControllerCount; ++i) {
    validate_endpoint(ports[i], addresses[i], timeout_ms);
  }
}

uint8_t brightness(float value)
{
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0F, 1.0F) * 255.0F));
}

std::vector<uint8_t> color_pdu(const std::array<uint8_t, 4> & values)
{
  std::vector<uint8_t> pdu{0x10, 0, 0, 0, 4, 8};
  for (const auto value : values) {
    pdu.insert(pdu.end(), {0, value});
  }
  return pdu;
}

void check_color_ack(const std::vector<uint8_t> & reply)
{
  if (reply != std::vector<uint8_t>{0x10, 0, 0, 0, 4}) {
    throw std::runtime_error("Invalid Modbus write acknowledgement");
  }
}

std::vector<uint8_t> read_holding_pdu(uint16_t first_register, uint16_t count)
{
  if (count == 0 || count > 125) {
    throw std::invalid_argument("holding register count must be 1..125");
  }
  return {
    0x03, static_cast<uint8_t>(first_register >> 8U), static_cast<uint8_t>(first_register & 0xFFU),
    static_cast<uint8_t>(count >> 8U), static_cast<uint8_t>(count & 0xFFU)};
}

std::vector<uint16_t> parse_holding_reply(const std::vector<uint8_t> & reply, uint16_t count)
{
  const size_t byte_count = static_cast<size_t>(count) * 2U;
  if (
    reply.size() != 2U + byte_count || reply[0] != 0x03 ||
    reply[1] != static_cast<uint8_t>(byte_count)) {
    throw std::runtime_error("Invalid Modbus holding register response");
  }
  std::vector<uint16_t> values;
  values.reserve(count);
  for (size_t offset = 2; offset < reply.size(); offset += 2) {
    values.push_back(
      static_cast<uint16_t>((static_cast<uint16_t>(reply[offset]) << 8U) | reply[offset + 1]));
  }
  return values;
}
}  // namespace periph_led_ultrasonic
