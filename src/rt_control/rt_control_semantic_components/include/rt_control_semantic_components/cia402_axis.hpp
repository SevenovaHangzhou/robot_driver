#ifndef RT_CONTROL_SEMANTIC_COMPONENTS__CIA402_AXIS_HPP_
#define RT_CONTROL_SEMANTIC_COMPONENTS__CIA402_AXIS_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace rt_control_semantic_components
{

enum class Cia402State : std::uint8_t
{
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
  kUnknown
};

class Cia402Axis final
{
public:
  explicit Cia402Axis(std::string joint_name)
  : joint_name_(std::move(joint_name)),
    command_interface_name_(joint_name_ + "/control_word"),
    state_interface_name_(joint_name_ + "/status_word"),
    command_interface_names_{command_interface_name_},
    state_interface_names_{state_interface_name_}
  {}

  const std::string & get_name() const noexcept {return joint_name_;}

  const std::vector<std::string> & get_command_interface_names() const noexcept
  {
    return command_interface_names_;
  }

  const std::vector<std::string> & get_state_interface_names() const noexcept
  {
    return state_interface_names_;
  }

  bool assign_loaned_command_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> & interfaces)
  {
    if (command_interface_.has_value()) {
      return false;
    }
    return bind_exactly_once(
      interfaces, joint_name_, "control_word", command_interface_);
  }

  bool assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface> & interfaces)
  {
    if (state_interface_.has_value()) {
      return false;
    }
    return bind_exactly_once(
      interfaces, joint_name_, "status_word", state_interface_);
  }

  void release_interfaces() noexcept
  {
    command_interface_.reset();
    state_interface_.reset();
  }

  bool has_command_interface() const noexcept {return command_interface_.has_value();}
  bool has_state_interface() const noexcept {return state_interface_.has_value();}
  bool is_bound() const noexcept
  {
    return has_command_interface() && has_state_interface();
  }

  bool set_control_word(std::uint16_t control_word)
  {
    if (!command_interface_.has_value()) {
      return false;
    }
    command_interface_->get().set_value(static_cast<double>(control_word));
    return true;
  }

  std::optional<std::uint16_t> get_status_word() const
  {
    if (!state_interface_.has_value()) {
      return std::nullopt;
    }
    const double value = state_interface_->get().get_value();
    constexpr double kMaximumStatusWord =
      static_cast<double>(std::numeric_limits<std::uint16_t>::max());
    if (
      !std::isfinite(value) || value < 0.0 || value > kMaximumStatusWord ||
      std::trunc(value) != value)
    {
      return std::nullopt;
    }
    return static_cast<std::uint16_t>(value);
  }

  Cia402State get_state() const
  {
    const auto status_word = get_status_word();
    return status_word.has_value() ? decode_state(*status_word) : Cia402State::kUnknown;
  }

  static constexpr Cia402State decode_state(std::uint16_t status_word) noexcept
  {
    if ((status_word & 0x004FU) == 0x0000U) {
      return Cia402State::kNotReadyToSwitchOn;
    }
    if ((status_word & 0x004FU) == 0x0040U) {
      return Cia402State::kSwitchOnDisabled;
    }
    if ((status_word & 0x006FU) == 0x0021U) {
      return Cia402State::kReadyToSwitchOn;
    }
    if ((status_word & 0x006FU) == 0x0023U) {
      return Cia402State::kSwitchedOn;
    }
    if ((status_word & 0x006FU) == 0x0027U) {
      return Cia402State::kOperationEnabled;
    }
    if ((status_word & 0x006FU) == 0x0007U) {
      return Cia402State::kQuickStopActive;
    }
    if ((status_word & 0x004FU) == 0x000FU) {
      return Cia402State::kFaultReactionActive;
    }
    if ((status_word & 0x004FU) == 0x0008U) {
      return Cia402State::kFault;
    }
    return Cia402State::kUnknown;
  }

private:
  template<typename LoanedInterface>
  static bool bind_exactly_once(
    std::vector<LoanedInterface> & interfaces, const std::string & expected_prefix,
    const std::string & expected_interface,
    std::optional<std::reference_wrapper<LoanedInterface>> & target)
  {
    LoanedInterface * match = nullptr;
    for (auto & interface : interfaces) {
      if (
        interface.get_prefix_name() != expected_prefix ||
        interface.get_interface_name() != expected_interface)
      {
        continue;
      }
      if (match != nullptr) {
        return false;
      }
      match = &interface;
    }
    if (match == nullptr) {
      return false;
    }
    target = std::ref(*match);
    return true;
  }

  std::string joint_name_;
  std::string command_interface_name_;
  std::string state_interface_name_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    state_interface_;
};

}  // namespace rt_control_semantic_components

#endif  // RT_CONTROL_SEMANTIC_COMPONENTS__CIA402_AXIS_HPP_
