#include "bms_node/protocol/can_transport.hpp"

#include <linux/can.h>

namespace bms_node
{

can_frame make_query_frame(const std::uint32_t request_id) noexcept
{
  can_frame frame{};
  frame.can_id = static_cast<canid_t>(request_id) | CAN_EFF_FLAG;
  frame.can_dlc = CAN_MAX_DLEN;
  return frame;
}

bool decode_response_frame(
  const can_frame & frame, const std::uint32_t response_id,
  const std::uint8_t *& data, std::size_t & size) noexcept
{
  data = nullptr;
  size = 0U;
  if ((frame.can_id & CAN_EFF_FLAG) == 0U ||
    (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
    (frame.can_id & CAN_EFF_MASK) != response_id || frame.can_dlc != CAN_MAX_DLEN)
  {
    return false;
  }
  data = frame.data;
  size = CAN_MAX_DLEN;
  return true;
}

}  // namespace bms_node
