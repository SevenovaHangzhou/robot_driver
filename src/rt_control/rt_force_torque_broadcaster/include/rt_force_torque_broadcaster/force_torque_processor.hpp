#ifndef RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_PROCESSOR_HPP_
#define RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_PROCESSOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

#include "rt_control_semantic_components/force_torque_sensor.hpp"

namespace rt_force_torque_broadcaster
{

enum class AuxiliaryPolicy : std::uint8_t
{
  kNone,
  kAllExactIntegersInRange
};

struct ForceTorqueProcessorConfig
{
  std::array<double, 6> scale_factors{};
  AuxiliaryPolicy auxiliary_policy{AuxiliaryPolicy::kNone};
  std::size_t auxiliary_count{0U};
  std::int32_t minimum_auxiliary_value{0};
  std::int32_t maximum_auxiliary_value{0};
  bool calibration_valid{false};
};

struct ForceTorqueOutcome
{
  std::array<std::int32_t, 6> raw_values{};
  std::array<double, 6> wrench_values{};
  bool raw_valid{false};
  bool wrench_valid{false};
  bool runtime_invalidated{false};
  bool invalidated_this_cycle{false};
};

class ForceTorqueProcessor final
{
public:
  explicit ForceTorqueProcessor(ForceTorqueProcessorConfig config)
  : config_(config)
  {
    if (
      config_.auxiliary_count >
      rt_control_semantic_components::ForceTorqueSensor::kMaximumAuxiliaryCount)
    {
      throw std::invalid_argument("invalid force/torque auxiliary interface count");
    }
    if (!std::all_of(
        config_.scale_factors.begin(), config_.scale_factors.end(),
        [](const double value) {return std::isfinite(value) && value != 0.0;}))
    {
      throw std::invalid_argument("force/torque scale factors must be finite and nonzero");
    }
    if (
      config_.auxiliary_policy == AuxiliaryPolicy::kAllExactIntegersInRange &&
      config_.minimum_auxiliary_value > config_.maximum_auxiliary_value)
    {
      throw std::invalid_argument("invalid force/torque auxiliary range");
    }
  }

  ForceTorqueOutcome process(
    const rt_control_semantic_components::ForceTorqueSample & sample,
    const double link_up, const double al_state) noexcept
  {
    ForceTorqueOutcome outcome;
    const bool operation = link_up == 1.0 && al_state == 8.0;
    if (operation) {
      seen_operation_ = true;
    } else if (seen_operation_ && !runtime_invalidated_) {
      runtime_invalidated_ = true;
      outcome.invalidated_this_cycle = true;
    }
    outcome.runtime_invalidated = runtime_invalidated_;

    if (sample.auxiliary_count != config_.auxiliary_count) {
      return outcome;
    }

    for (std::size_t index = 0U; index < sample.values.size(); ++index) {
      const auto raw = exact_int32(sample.values[index]);
      if (!raw.has_value()) {
        return outcome;
      }
      outcome.raw_values[index] = *raw;
    }

    bool auxiliary_eligible{true};
    for (std::size_t index = 0U; index < sample.auxiliary_count; ++index) {
      const double value = sample.auxiliary[index];
      if (!std::isfinite(value)) {
        return outcome;
      }
      if (config_.auxiliary_policy == AuxiliaryPolicy::kAllExactIntegersInRange) {
        const auto integer = exact_int32(value);
        if (!integer.has_value()) {
          return outcome;
        }
        auxiliary_eligible = auxiliary_eligible &&
          *integer >= config_.minimum_auxiliary_value &&
          *integer <= config_.maximum_auxiliary_value;
      }
    }
    outcome.raw_valid = true;

    if (
      !operation || runtime_invalidated_ || !config_.calibration_valid ||
      !auxiliary_eligible)
    {
      return outcome;
    }

    for (std::size_t index = 0U; index < outcome.raw_values.size(); ++index) {
      outcome.wrench_values[index] =
        static_cast<double>(outcome.raw_values[index]) * config_.scale_factors[index];
      if (!std::isfinite(outcome.wrench_values[index])) {
        outcome.wrench_values.fill(0.0);
        return outcome;
      }
    }
    outcome.wrench_valid = true;
    return outcome;
  }

  bool runtime_invalidated() const noexcept
  {
    return runtime_invalidated_;
  }

private:
  static std::optional<std::int32_t> exact_int32(const double value) noexcept
  {
    constexpr double kMinimum =
      static_cast<double>(std::numeric_limits<std::int32_t>::min());
    constexpr double kMaximum =
      static_cast<double>(std::numeric_limits<std::int32_t>::max());
    if (
      !std::isfinite(value) || value < kMinimum || value > kMaximum ||
      std::trunc(value) != value)
    {
      return std::nullopt;
    }
    return static_cast<std::int32_t>(value);
  }

  ForceTorqueProcessorConfig config_;
  bool seen_operation_{false};
  bool runtime_invalidated_{false};
};

}  // namespace rt_force_torque_broadcaster

#endif  // RT_FORCE_TORQUE_BROADCASTER__FORCE_TORQUE_PROCESSOR_HPP_
