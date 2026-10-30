#ifndef HW_SEMANTIC_COMPONENTS__CIA402_STATE_HPP_
#define HW_SEMANTIC_COMPONENTS__CIA402_STATE_HPP_

#include <cstdint>

// The one CiA402 status-word decoder for hardware plugins and controllers.
namespace hw_semantic_components
{

enum class Cia402State : std::uint8_t {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
  kUnknown
};

constexpr Cia402State decode_cia402_state(std::uint16_t status_word) noexcept
{
  if ((status_word & 0x004FU) == 0x0000U) {
    return Cia402State::kNotReadyToSwitchOn;
  }
  if ((status_word & 0x004FU) == 0x0040U) {
    return Cia402State::kSwitchOnDisabled;
  }
  if ((status_word & 0x006FU) == 0x0021U) {
    return Cia402State::kReadyToSwitchOn;
  }
  if ((status_word & 0x006FU) == 0x0023U) {
    return Cia402State::kSwitchedOn;
  }
  if ((status_word & 0x006FU) == 0x0027U) {
    return Cia402State::kOperationEnabled;
  }
  if ((status_word & 0x006FU) == 0x0007U) {
    return Cia402State::kQuickStopActive;
  }
  if ((status_word & 0x004FU) == 0x000FU) {
    return Cia402State::kFaultReactionActive;
  }
  if ((status_word & 0x004FU) == 0x0008U) {
    return Cia402State::kFault;
  }
  return Cia402State::kUnknown;
}

}  // namespace hw_semantic_components

#endif  // HW_SEMANTIC_COMPONENTS__CIA402_STATE_HPP_
