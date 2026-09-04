#ifndef RT_DIAGNOSTICS__ETHERCAT_MASTER_DIAGNOSTIC_HPP_
#define RT_DIAGNOSTICS__ETHERCAT_MASTER_DIAGNOSTIC_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace rt_diagnostics
{

struct EthercatMasterDiagnosticSample
{
  double process_data_age_ms{0.0};
  double link_up{0.0};
  double slaves_responding{0.0};
  double working_counter_error_count{0.0};
  bool all_values_present{true};
};

struct EthercatMasterDiagnosticAssessment
{
  std::uint8_t level{0U};
  std::string message;
  bool values_valid{false};
  std::uint16_t slaves_responding{0U};
  double working_counter_error_count{0.0};
};

[[nodiscard]] EthercatMasterDiagnosticAssessment evaluateEthercatMasterDiagnostic(
  const EthercatMasterDiagnosticSample & sample, bool topic_stale,
  std::uint16_t expected_responders,
  std::optional<double> previous_working_counter_error_count);

}  // namespace rt_diagnostics

#endif  // RT_DIAGNOSTICS__ETHERCAT_MASTER_DIAGNOSTIC_HPP_
