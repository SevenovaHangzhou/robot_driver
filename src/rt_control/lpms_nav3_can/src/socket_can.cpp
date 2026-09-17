#include "lpms_nav3_can/socket_can.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace lpms_nav3_can
{
namespace
{

void validate_node_id(const std::uint8_t node_id)
{
  if (node_id == 0U || node_id > 127U) {
    throw std::invalid_argument{"CANopen node_id must be in [1, 127]"};
  }
}

}  // namespace

std::array<can_filter, 5U> make_can_filters(const std::uint8_t node_id)
{
  validate_node_id(node_id);
  const auto node = static_cast<canid_t>(node_id);
  constexpr canid_t exact_standard_data_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  return {
    can_filter{0x180U + node, exact_standard_data_mask},
    can_filter{0x280U + node, exact_standard_data_mask},
    can_filter{0x380U + node, exact_standard_data_mask},
    can_filter{0x480U + node, exact_standard_data_mask},
    can_filter{0x700U + node, exact_standard_data_mask},
  };
}

bool is_reserved_robot_interface(const std::string_view interface_name) noexcept
{
  return interface_name == "can0" || interface_name == "can1";
}

}  // namespace lpms_nav3_can
