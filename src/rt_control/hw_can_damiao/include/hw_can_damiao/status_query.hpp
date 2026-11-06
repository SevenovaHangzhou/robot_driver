#ifndef HW_CAN_DAMIAO__STATUS_QUERY_HPP_
#define HW_CAN_DAMIAO__STATUS_QUERY_HPP_

#include <chrono>
#include <cstdint>

#include "hw_can_damiao/socketcan.hpp"

namespace hw_can_damiao
{

struct StatusReply
{
  bool answered{false};
  // Feedback ERR nibble: 0 disabled, 1 enabled, 3..14 vendor faults.
  std::uint8_t status{0U};
};

// Sends one disable command and waits for the feedback frame it triggers. The motor never
// enables, so this is the read of its ERR state that the bus allows.
[[nodiscard]] StatusReply query_status(
  const SocketCan & socket, std::uint16_t motor_id, std::uint16_t master_id,
  std::chrono::milliseconds timeout);

}  // namespace hw_can_damiao

#endif  // HW_CAN_DAMIAO__STATUS_QUERY_HPP_
