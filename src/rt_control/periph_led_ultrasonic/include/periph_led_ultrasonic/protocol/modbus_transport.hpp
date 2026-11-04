#pragma once

// Request/response PDUs for the LED controllers and E08 ultrasonic modules behind the TCP to
// RS-485 gateway. Connections, framing and timeouts belong to infra_modbus::Client.
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace periph_led_ultrasonic
{
inline constexpr size_t kLedControllerCount{6U};

// The gateway address itself is validated by infra_modbus::Client (numeric only).
void validate_endpoint(int64_t port, int64_t unit, int64_t timeout_ms);
void validate_led_config(
  const std::vector<int64_t> & ports, const std::vector<int64_t> & addresses, int64_t timeout_ms);
uint8_t brightness(float value);
inline constexpr std::array<uint8_t, 4> led_exit_color(bool abnormal)
{
  return abnormal ? std::array<uint8_t, 4>{255U, 0U, 0U, 0U}
                  : std::array<uint8_t, 4>{0U, 0U, 0U, 0U};
}

// FC16: write the four RGBW registers starting at 0.
std::vector<uint8_t> color_pdu(const std::array<uint8_t, 4> & values);
// Throws unless the reply echoes FC16, start register 0 and quantity 4.
void check_color_ack(const std::vector<uint8_t> & reply);
// FC03 for `count` (1..125) registers.
std::vector<uint8_t> read_holding_pdu(uint16_t first_register, uint16_t count);
std::vector<uint16_t> parse_holding_reply(const std::vector<uint8_t> & reply, uint16_t count);
}  // namespace periph_led_ultrasonic
