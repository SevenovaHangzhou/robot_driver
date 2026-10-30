#ifndef LPMS_NAV3_CAN__SOCKET_CAN_HPP_
#define LPMS_NAV3_CAN__SOCKET_CAN_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>

namespace lpms_nav3_can
{

[[nodiscard]] std::array<can_filter, 5U> make_can_filters(std::uint8_t node_id);

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__SOCKET_CAN_HPP_
