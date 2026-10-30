#ifndef PERIPH_BMS__CAN_TRANSPORT_HPP_
#define PERIPH_BMS__CAN_TRANSPORT_HPP_

#include <linux/can.h>

#include <cstddef>
#include <cstdint>

namespace periph_bms
{

[[nodiscard]] can_filter make_response_filter(std::uint32_t response_id) noexcept;
[[nodiscard]] can_frame make_query_frame(std::uint32_t request_id) noexcept;
[[nodiscard]] bool decode_response_frame(
  const can_frame & frame, std::uint32_t response_id, const std::uint8_t *& data,
  std::size_t & size) noexcept;

}  // namespace periph_bms

#endif  // PERIPH_BMS__CAN_TRANSPORT_HPP_
