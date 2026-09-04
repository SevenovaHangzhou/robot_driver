#include "rt_diagnostics/diagnostic_summary.hpp"

#include <tuple>
#include <utility>

namespace rt_diagnostics
{

DiagnosticSummaryAccumulator::DiagnosticSummaryAccumulator(std::string healthy_message)
: healthy_message_(std::move(healthy_message))
{}

void DiagnosticSummaryAccumulator::observe(
  std::uint8_t level, std::string_view source_name,
  std::string_view source_message)
{
  if (level == kOkLevel) {
    return;
  }

  const bool higher_severity = !has_unhealthy_source_ || level > worst_level_;
  const bool deterministic_tie =
    has_unhealthy_source_ && level == worst_level_ &&
    std::tie(source_name, source_message) <
    std::tie(worst_source_name_, worst_source_message_);
  if (!higher_severity && !deterministic_tie) {
    return;
  }

  worst_level_ = level;
  worst_source_name_ = source_name;
  worst_source_message_ = source_message;
  has_unhealthy_source_ = true;
}

DiagnosticSummary DiagnosticSummaryAccumulator::result() const
{
  if (!has_unhealthy_source_) {
    return DiagnosticSummary{kOkLevel, healthy_message_};
  }

  std::string message;
  message.reserve(worst_source_name_.size() + worst_source_message_.size() + 2U);
  message.append(worst_source_name_);
  message.append(": ");
  message.append(worst_source_message_);
  return DiagnosticSummary{worst_level_, std::move(message)};
}

}  // namespace rt_diagnostics
