#ifndef DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CORE_HPP_
#define DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CORE_HPP_

#include <cstdint>

#include "robot_hw_can/head_state_machine.hpp"

namespace damiao_head_controller
{

using robot_hw_can::HeadPhase;

struct HeadSnapshot
{
  HeadPhase phase{HeadPhase::disabled};
  bool fault_latched{false};
  bool motion_allowed{false};
  std::uint64_t handled_reset_generation{0U};
};

enum class OperationResult : std::uint8_t
{
  pending,
  success,
  fault,
};

enum class DiagnosticLevel : std::uint8_t
{
  ok = 0U,
  warn = 1U,
  error = 2U,
  stale = 3U,
};

struct DiagnosticAssessment
{
  DiagnosticLevel level{DiagnosticLevel::stale};
  const char * message{"feedback stale"};
};

[[nodiscard]] OperationResult assess_enable(const HeadSnapshot & snapshot) noexcept;
[[nodiscard]] OperationResult assess_disable(const HeadSnapshot & snapshot) noexcept;
[[nodiscard]] OperationResult assess_reset(
  const HeadSnapshot & snapshot, std::uint64_t requested_generation) noexcept;
[[nodiscard]] DiagnosticAssessment assess_diagnostics(
  const HeadSnapshot & snapshot, bool feedback_stale) noexcept;
[[nodiscard]] const char * phase_name(HeadPhase phase) noexcept;

}  // namespace damiao_head_controller

#endif  // DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CORE_HPP_
