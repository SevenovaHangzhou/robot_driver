#ifndef RT_DIAGNOSTICS__DIAGNOSTIC_SUMMARY_HPP_
#define RT_DIAGNOSTICS__DIAGNOSTIC_SUMMARY_HPP_

#include <cstdint>
#include <string>
#include <string_view>

namespace rt_diagnostics
{

struct DiagnosticSummary
{
  std::uint8_t level;
  std::string message;
};

class DiagnosticSummaryAccumulator final
{
public:
  explicit DiagnosticSummaryAccumulator(std::string healthy_message);

  void observe(
    std::uint8_t level, std::string_view source_name,
    std::string_view source_message);

  [[nodiscard]] DiagnosticSummary result() const;

private:
  static constexpr std::uint8_t kOkLevel{0U};

  std::string healthy_message_;
  std::string worst_source_name_;
  std::string worst_source_message_;
  std::uint8_t worst_level_{kOkLevel};
  bool has_unhealthy_source_{false};
};

}  // namespace rt_diagnostics

#endif  // RT_DIAGNOSTICS__DIAGNOSTIC_SUMMARY_HPP_
