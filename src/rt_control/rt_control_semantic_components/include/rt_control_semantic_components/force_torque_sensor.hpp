#ifndef RT_CONTROL_SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_
#define RT_CONTROL_SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"

namespace rt_control_semantic_components
{

struct ForceTorqueSample
{
  std::array<double, 6> values{};
  std::array<double, 6> auxiliary{};
  std::size_t auxiliary_count{0U};
};

class ForceTorqueSensor final
{
public:
  static constexpr std::size_t kValueCount{6U};
  static constexpr std::size_t kMaximumAuxiliaryCount{6U};
  static constexpr std::size_t kMaximumInterfaceCount{
    kValueCount + kMaximumAuxiliaryCount};

  ForceTorqueSensor(
    std::array<std::string, kValueCount> value_interface_names,
    std::vector<std::string> auxiliary_interface_names)
  : value_interface_names_(std::move(value_interface_names)),
    auxiliary_interface_names_(std::move(auxiliary_interface_names))
  {
    if (auxiliary_interface_names_.size() > kMaximumAuxiliaryCount) {
      throw std::invalid_argument(
              "force/torque sensor supports at most six auxiliary interfaces");
    }

    state_interface_names_.reserve(
      value_interface_names_.size() + auxiliary_interface_names_.size());
    state_interface_names_.insert(
      state_interface_names_.end(), value_interface_names_.begin(),
      value_interface_names_.end());
    state_interface_names_.insert(
      state_interface_names_.end(), auxiliary_interface_names_.begin(),
      auxiliary_interface_names_.end());

    std::unordered_set<std::string> unique_names;
    for (const auto & name : state_interface_names_) {
      const auto separator = name.find_last_of('/');
      if (
        name.empty() || separator == std::string::npos || separator == 0U ||
        separator + 1U >= name.size() || !unique_names.insert(name).second)
      {
        throw std::invalid_argument(
                "force/torque state interfaces must be unique full names");
      }
    }
  }

  const std::vector<std::string> & get_state_interface_names() const noexcept
  {
    return state_interface_names_;
  }

  bool assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface> & interfaces) noexcept
  {
    if (is_bound()) {
      return false;
    }

    std::array<hardware_interface::LoanedStateInterface *, kMaximumInterfaceCount>
    candidates{};
    for (std::size_t index = 0U; index < state_interface_names_.size(); ++index) {
      for (auto & interface : interfaces) {
        if (interface.get_name() != state_interface_names_[index]) {
          continue;
        }
        if (candidates[index] != nullptr) {
          return false;
        }
        candidates[index] = &interface;
      }
      if (candidates[index] == nullptr) {
        return false;
      }
    }

    interfaces_ = candidates;
    return true;
  }

  void release_interfaces() noexcept
  {
    interfaces_.fill(nullptr);
  }

  bool is_bound() const noexcept
  {
    return std::all_of(
      interfaces_.begin(),
      interfaces_.begin() + static_cast<std::ptrdiff_t>(state_interface_names_.size()),
      [](const auto * interface) {return interface != nullptr;});
  }

  std::optional<ForceTorqueSample> read() const noexcept
  {
    if (!is_bound()) {
      return std::nullopt;
    }

    ForceTorqueSample sample;
    for (std::size_t index = 0U; index < kValueCount; ++index) {
      sample.values[index] = interfaces_[index]->get_value();
    }
    sample.auxiliary_count = auxiliary_interface_names_.size();
    for (std::size_t index = 0U; index < sample.auxiliary_count; ++index) {
      sample.auxiliary[index] = interfaces_[kValueCount + index]->get_value();
    }
    return sample;
  }

private:
  std::array<std::string, kValueCount> value_interface_names_;
  std::vector<std::string> auxiliary_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::array<hardware_interface::LoanedStateInterface *, kMaximumInterfaceCount>
  interfaces_{};
};

}  // namespace rt_control_semantic_components

#endif  // RT_CONTROL_SEMANTIC_COMPONENTS__FORCE_TORQUE_SENSOR_HPP_
