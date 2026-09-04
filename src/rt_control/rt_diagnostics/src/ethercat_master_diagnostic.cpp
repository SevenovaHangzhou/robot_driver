#include "rt_diagnostics/ethercat_master_diagnostic.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace rt_diagnostics
{
namespace
{

constexpr std::uint8_t kOk{0U};
constexpr std::uint8_t kWarn{1U};
constexpr std::uint8_t kError{2U};
constexpr std::uint8_t kStale{3U};
constexpr double kMaximumProcessDataAgeMs{3000.0};

bool isExactNonnegativeInteger(double value) noexcept
{
  return std::isfinite(value) && value >= 0.0 && std::trunc(value) == value;
}

EthercatMasterDiagnosticAssessment invalidAssessment(const std::string & field)
{
  return EthercatMasterDiagnosticAssessment{
    kStale,
    "EtherCAT master snapshot has invalid " + field,
    false,
    0U,
    0.0};
}

}  // namespace

EthercatMasterDiagnosticAssessment evaluateEthercatMasterDiagnostic(
  const EthercatMasterDiagnosticSample & sample, bool topic_stale,
  std::uint16_t expected_responders,
  std::optional<double> previous_working_counter_error_count)
{
  if (!sample.all_values_present) {
    return invalidAssessment("required values");
  }
  if (!std::isfinite(sample.process_data_age_ms) || sample.process_data_age_ms < 0.0) {
    return invalidAssessment("process_data_age_ms");
  }
  if (!std::isfinite(sample.link_up) || (sample.link_up != 0.0 && sample.link_up != 1.0)) {
    return invalidAssessment("link_up");
  }
  if (
    !isExactNonnegativeInteger(sample.slaves_responding) ||
    sample.slaves_responding >
    static_cast<double>(std::numeric_limits<std::uint16_t>::max()))
  {
    return invalidAssessment("slaves_responding");
  }
  if (!isExactNonnegativeInteger(sample.working_counter_error_count)) {
    return invalidAssessment("wc_error_count");
  }
  if (
    previous_working_counter_error_count.has_value() &&
    !isExactNonnegativeInteger(*previous_working_counter_error_count))
  {
    return invalidAssessment("previous_wc_error_count");
  }

  const auto responders = static_cast<std::uint16_t>(sample.slaves_responding);
  EthercatMasterDiagnosticAssessment assessment{
    kOk,
    "EtherCAT master healthy",
    true,
    responders,
    sample.working_counter_error_count};
  if (topic_stale || sample.process_data_age_ms > kMaximumProcessDataAgeMs) {
    assessment.level = kStale;
    assessment.message = "EtherCAT process-data snapshot is stale";
  } else if (sample.link_up == 0.0) {
    assessment.level = kError;
    assessment.message = "EtherCAT link is down";
  } else if (
    responders != expected_responders ||
    (previous_working_counter_error_count.has_value() &&
    sample.working_counter_error_count > *previous_working_counter_error_count))
  {
    assessment.level = kWarn;
    assessment.message = "EtherCAT responder count or working counter changed";
  }
  return assessment;
}

}  // namespace rt_diagnostics
