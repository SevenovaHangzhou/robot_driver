#ifndef RT_CONTROL_SEMANTIC_COMPONENTS__HEAD_PHASE_HPP_
#define RT_CONTROL_SEMANTIC_COMPONENTS__HEAD_PHASE_HPP_

#include <cmath>
#include <cstdint>
#include <optional>

namespace rt_control_semantic_components
{
enum class HeadPhase : std::uint8_t {
  disabled = 0U,
  enabling = 1U,
  enabled = 2U,
  disabling = 3U,
  reset_disabling = 4U,
  resetting = 5U,
  fault_latched = 6U,
};

[[nodiscard]] inline std::optional<HeadPhase> decode_head_phase(double value) noexcept
{
  if (!std::isfinite(value) || value < 0.0 || value > 6.0 || std::trunc(value) != value) {
    return std::nullopt;
  }
  return static_cast<HeadPhase>(static_cast<std::uint8_t>(value));
}
}  // namespace rt_control_semantic_components

#endif  // RT_CONTROL_SEMANTIC_COMPONENTS__HEAD_PHASE_HPP_
