#ifndef HW_SEMANTIC_COMPONENTS__HANDSHAKE_PHASE_HPP_
#define HW_SEMANTIC_COMPONENTS__HANDSHAKE_PHASE_HPP_

#include <cmath>
#include <cstdint>
#include <optional>

namespace hw_semantic_components
{
// Enable phase of a device whose hardware plugin runs its own enable sequence (DaMiao head).
// The numeric values are the `<gpio>/phase` state interface shared by plugin and controller.
enum class HandshakePhase : std::uint8_t {
  disabled = 0U,
  enabling = 1U,
  enabled = 2U,
  disabling = 3U,
  reset_disabling = 4U,
  resetting = 5U,
  fault_latched = 6U,
};

[[nodiscard]] inline std::optional<HandshakePhase> decode_handshake_phase(double value) noexcept
{
  if (!std::isfinite(value) || value < 0.0 || value > 6.0 || std::trunc(value) != value) {
    return std::nullopt;
  }
  return static_cast<HandshakePhase>(static_cast<std::uint8_t>(value));
}
}  // namespace hw_semantic_components

#endif  // HW_SEMANTIC_COMPONENTS__HANDSHAKE_PHASE_HPP_
