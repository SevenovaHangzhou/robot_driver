#include "bms_node/can_transport.hpp"

#include <linux/can.h>

namespace bms_node
{

can_filter make_response_filter(const std::uint32_t response_id) noexcept
{
  constexpr canid_t exact_extended_data_mask =
    CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG;
  return can_filter{
    static_cast<canid_t>(response_id) | CAN_EFF_FLAG,
    exact_extended_data_mask};
}

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
