#ifndef HW_CAN_DAMIAO__PARAMETER_TOOL_HPP_
#define HW_CAN_DAMIAO__PARAMETER_TOOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hw_can_damiao/protocol.hpp"

namespace hw_can_damiao
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

}  // namespace hw_can_damiao

#endif  // HW_CAN_DAMIAO__PARAMETER_TOOL_HPP_
