#include "damiao_head_controller/head_manager_core.hpp"

namespace damiao_head_controller
{

OperationResult assess_enable(const HeadSnapshot & snapshot) noexcept
{
  if (snapshot.fault_latched || snapshot.phase == HeadPhase::fault_latched) {
    return OperationResult::fault;
  }
  if (snapshot.phase == HeadPhase::enabled && snapshot.motion_allowed) {
    return OperationResult::success;
  }
  return OperationResult::pending;
}

OperationResult assess_disable(const HeadSnapshot & snapshot) noexcept
{
  if (snapshot.fault_latched || snapshot.phase == HeadPhase::fault_latched) {
    return OperationResult::fault;
  }
  if (snapshot.phase == HeadPhase::disabled && !snapshot.motion_allowed) {
    return OperationResult::success;
  }
  return OperationResult::pending;
}

OperationResult assess_reset(
  const HeadSnapshot & snapshot, std::uint64_t requested_generation) noexcept
{
  if (snapshot.phase == HeadPhase::disabled && !snapshot.fault_latched &&
    snapshot.handled_reset_generation >= requested_generation)
  {
    return OperationResult::success;
  }
  return OperationResult::pending;
}

DiagnosticAssessment assess_diagnostics(
  const HeadSnapshot & snapshot, bool feedback_stale) noexcept
{
  if (feedback_stale) {
    return {DiagnosticLevel::stale, "feedback stale"};
  }
  if (snapshot.fault_latched || snapshot.phase == HeadPhase::fault_latched) {
    return {DiagnosticLevel::error, "fault latched"};
  }
  if (snapshot.phase == HeadPhase::enabled && snapshot.motion_allowed) {
    return {DiagnosticLevel::ok, "enabled"};
  }
  if (snapshot.phase == HeadPhase::disabled) {
    return {DiagnosticLevel::ok, "disabled"};
  }
  return {DiagnosticLevel::warn, phase_name(snapshot.phase)};
}

const char * phase_name(HeadPhase phase) noexcept
{
  switch (phase) {
    case HeadPhase::disabled: return "disabled";
    case HeadPhase::enabling: return "enabling";
    case HeadPhase::enabled: return "enabled";
    case HeadPhase::disabling: return "disabling";
    case HeadPhase::reset_disabling: return "reset disabling";
    case HeadPhase::resetting: return "resetting";
    case HeadPhase::fault_latched: return "fault latched";
  }
  return "unknown";
}

}  // namespace damiao_head_controller
