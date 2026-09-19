#ifndef ROBOT_HW_CAN__PARAMETER_TOOL_HPP_
#define ROBOT_HW_CAN__PARAMETER_TOOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "robot_hw_can/protocol.hpp"

namespace robot_hw_can
{

struct ParameterWriteRequest
{
  std::string can_interface;
  std::uint16_t motor_id{0U};
  std::uint16_t master_id{0U};
  TrapezoidalProfile profile{};
  bool save_to_flash{false};
};

[[nodiscard]] ParameterWriteRequest parse_parameter_request(
  const std::vector<std::string> & arguments);
void execute_parameter_write(const ParameterWriteRequest & request);

}  // namespace robot_hw_can

#endif  // ROBOT_HW_CAN__PARAMETER_TOOL_HPP_
