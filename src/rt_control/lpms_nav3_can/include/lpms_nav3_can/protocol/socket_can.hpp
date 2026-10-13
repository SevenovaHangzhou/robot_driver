#ifndef LPMS_NAV3_CAN__SOCKET_CAN_HPP_
#define LPMS_NAV3_CAN__SOCKET_CAN_HPP_

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <string_view>

namespace lpms_nav3_can
{

[[nodiscard]] std::array<can_filter, 5U> make_can_filters(std::uint8_t node_id);

[[nodiscard]] bool is_reserved_robot_interface(std::string_view interface_name) noexcept;

}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__SOCKET_CAN_HPP_
