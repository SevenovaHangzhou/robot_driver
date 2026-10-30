#include "enable_manager/handshake_device.hpp"

#include <cmath>
#include <optional>
#include <utility>

namespace enable_manager
{
namespace
{

constexpr std::size_t kPhase = 0U;
constexpr std::size_t kFault = 1U;
constexpr std::size_t kHandledGeneration = 2U;
constexpr std::size_t kMotionAllowed = 3U;

std::optional<bool> exact_flag(double value) noexcept
{
  if (value == 0.0) {
    return false;
  }
  if (value == 1.0) {
    return true;
  }
  return std::nullopt;
}

std::optional<std::uint64_t> exact_generation(double value) noexcept
{
  constexpr double kMaximumExactInteger{9007199254740991.0};
  if (
    !std::isfinite(value) || value < 0.0 || value > kMaximumExactInteger ||
    std::trunc(value) != value) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(value);
}

// valid | fault << 1 | motion << 2 | phase << 3
std::uint32_t pack(bool valid, bool fault, bool motion, HandshakeDevice::Phase phase) noexcept
{
  return static_cast<std::uint32_t>(valid) | static_cast<std::uint32_t>(fault) << 1U |
         static_cast<std::uint32_t>(motion) << 2U | static_cast<std::uint32_t>(phase) << 3U;
}

}  // namespace

HandshakeDevice::HandshakeDevice(std::string gpio) : gpio_(std::move(gpio)) {}

std::vector<std::string> HandshakeDevice::command_interface_names() const
{
  return {gpio_ + "/enable_request", gpio_ + "/reset_generation"};
}

std::vector<std::string> HandshakeDevice::state_interface_names() const
{
  return {
    gpio_ + "/phase", gpio_ + "/fault_latched", gpio_ + "/handled_reset_generation",
    gpio_ + "/motion_allowed"};
}

bool HandshakeDevice::bind(
  std::vector<hardware_interface::LoanedCommandInterface> & commands,
  std::vector<hardware_interface::LoanedStateInterface> & states)
{
  release();
  const auto command_names = command_interface_names();
  const auto state_names = state_interface_names();
  for (auto & interface : commands) {
    if (interface.get_name() == command_names[0]) {
      enable_command_ = &interface;
    } else if (interface.get_name() == command_names[1]) {
      reset_command_ = &interface;
    }
  }
  for (auto & interface : states) {
    for (std::size_t index = 0U; index < state_names.size(); ++index) {
      if (interface.get_name() == state_names[index]) {
        states_[index] = &interface;
      }
    }
  }
  for (const auto * state : states_) {
    if (state == nullptr) {
      release();
      return false;
    }
  }
  if (enable_command_ == nullptr || reset_command_ == nullptr) {
    release();
    return false;
  }
  return true;
}

void HandshakeDevice::release() noexcept
{
  enable_command_ = nullptr;
  reset_command_ = nullptr;
  states_.fill(nullptr);
}

bool HandshakeDevice::baseline() noexcept
{
  read();
  if (!valid_) {
    return false;
  }
  enable_requested_ = false;
  reset_generation_ = handled_generation_;
  reset_pending_ = false;
  write();
  return true;
}

void HandshakeDevice::read() noexcept
{
  const auto phase =
    rt_control_semantic_components::decode_handshake_phase(states_[kPhase]->get_value());
  const auto fault = exact_flag(states_[kFault]->get_value());
  const auto motion = exact_flag(states_[kMotionAllowed]->get_value());
  const auto generation = exact_generation(states_[kHandledGeneration]->get_value());
  valid_ = phase && fault && motion && generation;
  if (valid_) {
    phase_ = *phase;
    fault_latched_ = *fault;
    motion_allowed_ = *motion;
    handled_generation_ = *generation;
  }
  published_status_.store(
    pack(valid_, fault_latched_, motion_allowed_, phase_), std::memory_order_release);
}

void HandshakeDevice::write() noexcept
{
  (void)enable_command_->set_value(enable_requested_ ? 1.0 : 0.0);
  (void)reset_command_->set_value(static_cast<double>(reset_generation_));
}

void HandshakeDevice::start_reset() noexcept
{
  enable_requested_ = false;
  reset_target_ = ++reset_generation_;
  reset_pending_ = true;
}

bool HandshakeDevice::faulted() const noexcept
{
  return !valid_ || fault_latched_ || phase_ == Phase::fault_latched;
}

bool HandshakeDevice::enabled() const noexcept
{
  return !faulted() && phase_ == Phase::enabled && motion_allowed_;
}

bool HandshakeDevice::disabled() const noexcept
{
  return !faulted() && phase_ == Phase::disabled && !motion_allowed_;
}

bool HandshakeDevice::reset_complete() const noexcept
{
  return disabled() && (!reset_pending_ || handled_generation_ >= reset_target_);
}

HandshakeDevice::Status HandshakeDevice::status() const noexcept
{
  const std::uint32_t bits = published_status_.load(std::memory_order_acquire);
  return Status{
    (bits & 1U) != 0U, static_cast<Phase>(bits >> 3U), (bits & 2U) != 0U, (bits & 4U) != 0U};
}

}  // namespace enable_manager
