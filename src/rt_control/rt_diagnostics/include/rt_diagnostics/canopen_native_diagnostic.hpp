#ifndef RT_DIAGNOSTICS__CANOPEN_NATIVE_DIAGNOSTIC_HPP_
#define RT_DIAGNOSTICS__CANOPEN_NATIVE_DIAGNOSTIC_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rt_diagnostics
{

struct NativeCanopenDiagnosticValue
{
  std::string key;
  std::string value;
};

bool operator==(
  const NativeCanopenDiagnosticValue & lhs,
  const NativeCanopenDiagnosticValue & rhs) noexcept;

struct NativeCanopenDiagnostic
{
  std::uint8_t level{0U};
  std::string message;
  std::vector<NativeCanopenDiagnosticValue> values;
};

bool operator==(
  const NativeCanopenDiagnostic & lhs,
  const NativeCanopenDiagnostic & rhs) noexcept;

struct NativeCanopenDiagnosticAssessment
{
  std::uint8_t level{0U};
  std::string message;
};

struct NativeCanopenDiagnosticSnapshot
{
  NativeCanopenDiagnostic diagnostic;
  std::chrono::steady_clock::time_point received{};
  bool valid{false};
};

[[nodiscard]] NativeCanopenDiagnosticAssessment evaluateNativeCanopenDiagnostic(
  const NativeCanopenDiagnostic & diagnostic);

[[nodiscard]] bool ingestConfiguredNativeCanopenDiagnostic(
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> & configured_snapshots,
  const std::string & hardware_id,
  NativeCanopenDiagnostic diagnostic,
  std::chrono::steady_clock::time_point received);

}  // namespace rt_diagnostics

#endif  // RT_DIAGNOSTICS__CANOPEN_NATIVE_DIAGNOSTIC_HPP_
