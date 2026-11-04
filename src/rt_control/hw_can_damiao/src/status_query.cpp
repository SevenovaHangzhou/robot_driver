#include "hw_can_damiao/status_query.hpp"

#include <algorithm>

namespace hw_can_damiao
{

StatusReply query_status(
  const SocketCan & socket, std::uint16_t motor_id, std::uint16_t master_id,
  std::chrono::milliseconds timeout)
{
  socket.send(make_special_command(motor_id, SpecialCommand::disable));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (!socket.wait_readable(static_cast<int>(std::max<std::int64_t>(1, remaining.count())))) {
      continue;
    }
    CanFrame frame{};
    while (socket.receive(frame)) {
      // Register replies (0x33 read, 0x55 write, 0xAA save) share the master ID; skip them.
      if (
        frame.id == master_id && frame.data[2] != 0x33U && frame.data[2] != 0x55U &&
        frame.data[2] != 0xAAU && (frame.data[0] & 0x0FU) == static_cast<std::uint8_t>(motor_id)) {
        return {true, static_cast<std::uint8_t>(frame.data[0] >> 4U)};
      }
    }
  }
  return {};
}

}  // namespace hw_can_damiao
