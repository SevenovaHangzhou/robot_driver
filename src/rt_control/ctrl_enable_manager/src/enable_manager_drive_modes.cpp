// Drive operation-mode checks: the mode_of_operation_display readback of each managed axis.
#include "ctrl_enable_manager/enable_manager_controller.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ctrl_enable_manager
{

bool EnableManagerController::modeConfirmed(std::size_t axis) const noexcept
{
  return expected_modes_.empty() || actual_modes_[axis] == expected_modes_[axis];
}

bool EnableManagerController::modePending(std::size_t axis) const noexcept
{
  return !expected_modes_.empty() && actual_modes_[axis] == 0;
}

void EnableManagerController::refreshModes()
{
  for (std::size_t axis = 0; axis < expected_modes_.size(); ++axis) {
    const double value = state_interfaces_[mode_state_indices_[axis]].get_value();
    actual_modes_[axis] =
      std::isfinite(value) && value >= -128.0 && value <= 127.0 && std::floor(value) == value
        ? static_cast<std::int16_t>(value)
        : 256;
  }
}

}  // namespace ctrl_enable_manager
