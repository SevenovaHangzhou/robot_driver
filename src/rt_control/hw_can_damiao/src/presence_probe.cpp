// Startup-only discovery. Sends register reads; never enables, resets, or moves a motor.
#include <charconv>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include "hw_can_damiao/socketcan.hpp"

namespace
{
unsigned number(const char * text, unsigned maximum)
{
  const std::string value{text};
  unsigned result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (
    parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || result == 0U ||
    result > maximum) {
    throw std::invalid_argument{"invalid probe argument"};
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 5) {
      throw std::invalid_argument{
        "usage: damiao_presence_probe interface motor_id master_id timeout_ms"};
    }
    const auto motor = static_cast<std::uint16_t>(number(argv[2], 15U));
    const auto master = static_cast<std::uint16_t>(number(argv[3], 0x7FFU));
    const auto timeout = std::chrono::milliseconds{number(argv[4], 60000U)};
    hw_can_damiao::SocketCan socket;
    socket.open(argv[1], {master});
    socket.send(hw_can_damiao::make_register_read(motor, hw_can_damiao::Register::can_id));
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!socket.wait_readable(1)) {
        continue;
      }
      hw_can_damiao::CanFrame frame{};
      if (!socket.receive(frame) || frame.length != 8U || frame.id != master) {
        continue;
      }
      if (
        frame.data[0] != motor || frame.data[1] != 0U || frame.data[2] != 0x33U ||
        frame.data[3] != static_cast<std::uint8_t>(hw_can_damiao::Register::can_id)) {
        continue;
      }
      const auto reply = hw_can_damiao::decode_register_reply(frame);
      if (reply.raw_value != motor) {
        throw std::runtime_error{"head motor identity mismatch"};
      }
      std::cout << "present\n";
      return 0;
    }
    std::cout << "absent\n";
    return 3;
  } catch (const std::exception & error) {
    std::cerr << "head discovery failed: " << error.what() << '\n';
    return 1;
  }
}
