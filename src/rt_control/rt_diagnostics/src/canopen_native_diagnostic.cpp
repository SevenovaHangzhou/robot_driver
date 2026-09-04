#include "rt_diagnostics/canopen_native_diagnostic.hpp"

#include <array>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace rt_diagnostics
{
namespace
{

constexpr std::uint8_t kOk{0U};
constexpr std::uint8_t kWarn{1U};
constexpr std::uint8_t kError{2U};
constexpr std::uint8_t kStale{3U};
constexpr std::array<std::string_view, 5U> kRequiredFields{
  "device_state", "nmt_state", "emcy_state", "cia402_mode", "cia402_state"};
// These literals are emitted by the pinned ros2_canopen NodeCanopen402Driver.
// Synonyms must not silently become another hardware-state authority.
constexpr std::string_view kAttachedDevice{"Added to master."};
constexpr std::string_view kOperationalNmt{"START"};
constexpr std::string_view kOperationEnabledSummary{"Operation enabled"};
constexpr std::string_view kProfiledVelocityMode{"Mode switched to: 3"};
// ros2_canopen retains this exact CANopen error-reset frame in its EMCY field.
// Its safety callback also treats eec == 0 as cleared rather than an active fault.
constexpr std::string_view kClearedEmcy{
  "Emergency message: eec: 0 er: 0 msef: 0 0 0 0 0 "};

std::string withNativeMessage(
  std::string message, const std::string & native_message)
{
  if (!native_message.empty()) {
    message.append("; native message: ");
    message.append(native_message);
  }
  return message;
}

bool isKnownNonOperationalNmt(std::string_view value) noexcept
{
  return value == "BOOTUP" || value == "PREOP" || value == "RESET_COMM" ||
         value == "RESET_NODE" || value == "STOP" || value == "TOGGLE";
}

bool isKnownNonOperationalSummary(std::string_view message) noexcept
{
  return message == "Not ready to switch on" || message == "Switch on disabled" ||
         message == "Ready to switch on" || message == "Switched on" ||
         message == "Quick stop active" || message == "Warning bit is set" ||
         message == "Internal limit active";
}

void consider(
  NativeCanopenDiagnosticAssessment & assessment, std::uint8_t level,
  std::string message)
{
  if (level > assessment.level) {
    assessment.level = level;
    assessment.message = std::move(message);
  }
}

}  // namespace

bool operator==(
  const NativeCanopenDiagnosticValue & lhs,
  const NativeCanopenDiagnosticValue & rhs) noexcept
{
  return lhs.key == rhs.key && lhs.value == rhs.value;
}

bool operator==(
  const NativeCanopenDiagnostic & lhs,
  const NativeCanopenDiagnostic & rhs) noexcept
{
  return lhs.level == rhs.level && lhs.message == rhs.message && lhs.values == rhs.values;
}

NativeCanopenDiagnosticAssessment evaluateNativeCanopenDiagnostic(
  const NativeCanopenDiagnostic & diagnostic)
{
  std::unordered_map<std::string_view, const std::string *> fields;
  fields.reserve(kRequiredFields.size());
  for (const auto & field : diagnostic.values) {
    bool required = false;
    for (const auto required_field : kRequiredFields) {
      if (field.key == required_field) {
        required = true;
        break;
      }
    }
    if (!required) {
      continue;
    }
    const auto inserted = fields.emplace(field.key, &field.value);
    if (!inserted.second) {
      return NativeCanopenDiagnosticAssessment{
        kStale,
        withNativeMessage(
          "ros2_canopen native diagnostic field is duplicated: " + field.key,
          diagnostic.message)};
    }
  }

  for (const auto required_field : kRequiredFields) {
    if (fields.find(required_field) == fields.end()) {
      return NativeCanopenDiagnosticAssessment{
        kStale,
        withNativeMessage(
          "ros2_canopen native diagnostic field is missing: " + std::string(required_field),
          diagnostic.message)};
    }
  }

  NativeCanopenDiagnosticAssessment assessment{kOk, diagnostic.message};
  if (diagnostic.level > kStale) {
    consider(
      assessment, kError,
      withNativeMessage(
        "ros2_canopen native diagnostic has an unknown level: " +
        std::to_string(diagnostic.level), diagnostic.message));
  } else if (diagnostic.level != kOk) {
    assessment.level = diagnostic.level;
    if (assessment.message.empty()) {
      assessment.message =
        "ros2_canopen native diagnostic reported level " + std::to_string(diagnostic.level);
    }
  }

  if (diagnostic.message.empty()) {
    consider(
      assessment, kStale,
      "ros2_canopen native diagnostic message is empty");
  } else if (diagnostic.message != kOperationEnabledSummary) {
    const auto level = isKnownNonOperationalSummary(diagnostic.message) ? kWarn : kError;
    consider(
      assessment, level,
      withNativeMessage(
        "ros2_canopen current CiA402 state is not OperationEnabled",
        diagnostic.message));
  }

  const auto & device_state = *fields.at("device_state");
  if (device_state.empty()) {
    consider(
      assessment, kStale,
      withNativeMessage("ros2_canopen device_state is empty", diagnostic.message));
  } else if (device_state != kAttachedDevice) {
    consider(
      assessment, kError,
      withNativeMessage(
        "ros2_canopen device is not attached to the master: " + device_state,
        diagnostic.message));
  }

  const auto & nmt_state = *fields.at("nmt_state");
  if (nmt_state.empty()) {
    consider(
      assessment, kStale,
      withNativeMessage("ros2_canopen nmt_state is empty", diagnostic.message));
  } else if (nmt_state != kOperationalNmt) {
    const auto level = isKnownNonOperationalNmt(nmt_state) ? kWarn : kError;
    consider(
      assessment, level,
      withNativeMessage(
        "ros2_canopen NMT is not operational: " + nmt_state, diagnostic.message));
  }

  const auto & emcy_state = *fields.at("emcy_state");
  if (!emcy_state.empty() && emcy_state != kClearedEmcy) {
    consider(
      assessment, kError,
      withNativeMessage(
        "ros2_canopen EMCY is active: " + emcy_state, diagnostic.message));
  }

  const auto & cia402_mode = *fields.at("cia402_mode");
  if (cia402_mode.empty()) {
    consider(
      assessment, kStale,
      withNativeMessage("ros2_canopen cia402_mode is empty", diagnostic.message));
  } else if (cia402_mode != kProfiledVelocityMode) {
    consider(
      assessment, kError,
      withNativeMessage(
        "ros2_canopen CiA402 mode is not selected: " + cia402_mode,
        diagnostic.message));
  }

  if (assessment.level == kOk && assessment.message.empty()) {
    assessment.message = "ros2_canopen node operational";
  }
  return assessment;
}

bool ingestConfiguredNativeCanopenDiagnostic(
  std::unordered_map<std::string, NativeCanopenDiagnosticSnapshot> & configured_snapshots,
  const std::string & hardware_id,
  NativeCanopenDiagnostic diagnostic,
  std::chrono::steady_clock::time_point received)
{
  const auto configured = configured_snapshots.find(hardware_id);
  if (configured == configured_snapshots.end()) {
    return false;
  }
  configured->second.diagnostic = std::move(diagnostic);
  configured->second.received = received;
  configured->second.valid = true;
  return true;
}

}  // namespace rt_diagnostics
