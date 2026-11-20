#include "hw_ethercat/csp_torque_offset_slave.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace hw_ethercat
{
bool CspTorqueOffsetSlave::setupSlave(
  std::unordered_map<std::string, std::string> parameters, std::vector<double> * states,
  std::vector<double> * commands)
{
  offset_command_index_ = control_command_index_ = raw_state_index_ = epoch_state_index_ = -1;
  command_epoch_ = 0.0;
  complete_ = false;
  const auto raw_state = parameters.find("state_interface/torque_offset_permille_sent");
  const auto epoch_state = parameters.find("state_interface/gravity_command_epoch");
  if (raw_state == parameters.end() || epoch_state == parameters.end() || states == nullptr) {
    return false;
  }
  try {
    raw_state_index_ = std::stoi(raw_state->second);
    epoch_state_index_ = std::stoi(epoch_state->second);
  } catch (const std::exception &) {
    return false;
  }
  if (
    raw_state_index_ < 0 || epoch_state_index_ < 0 ||
    static_cast<size_t>(std::max(raw_state_index_, epoch_state_index_)) >= states->size()) {
    return false;
  }
  (*states)[static_cast<size_t>(raw_state_index_)] = 0.0;
  (*states)[static_cast<size_t>(epoch_state_index_)] = 0.0;
  if (
    !EcCiA402Drive::setupSlave(std::move(parameters), states, commands) ||
    mode_of_operation_ != 8 || auto_state_transitions_ || commands == nullptr) {
    return false;
  }
  unsigned int offsets = 0;
  for (const auto & channel : pdo_channels_info_) {
    if (channel.index == 0x6040) {
      control_command_index_ = channel.interface_index;
    }
    if (channel.index == 0x60B2) {
      ++offsets;
      if (
        channel.pdo_type != ethercat_interface::RPDO || channel.sub_index != 0 ||
        channel.data_type != "int16" || channel.interface_name != "effort" ||
        !std::isfinite(channel.factor) || channel.factor == 0.0 || channel.offset != 0.0 ||
        channel.default_value != 0.0) {
        return false;
      }
      offset_command_index_ = channel.interface_index;
    }
  }
  if (
    offsets != 1 || offset_command_index_ < 0 || control_command_index_ < 0 ||
    static_cast<size_t>(std::max(offset_command_index_, control_command_index_)) >=
      commands->size()) {
    return false;
  }
  clear_offset();
  return true;
}

void CspTorqueOffsetSlave::clear_offset() noexcept
{
  if (offset_command_index_ >= 0 && command_interface_ptr_ != nullptr) {
    (*command_interface_ptr_)[static_cast<size_t>(offset_command_index_)] = 0.0;
  }
}

void CspTorqueOffsetSlave::onPdoCycleStart(bool complete)
{
  EcCiA402Drive::onPdoCycleStart(complete);
  complete_ = complete;
  if (!complete) {
    clear_offset();
  }
}

void CspTorqueOffsetSlave::set_state_is_operational(bool operational)
{
  EcCiA402Drive::set_state_is_operational(operational);
  if (!operational) {
    complete_ = false;
    clear_offset();
  }
}

void CspTorqueOffsetSlave::processData(size_t index, uint8_t * address)
{
  auto & channel = pdo_channels_info_[index];
  bool accepted = false;
  if (channel.index == 0x60B2) {
    // The generic writer otherwise casts Inf/out-of-range doubles to int16.
    // Override only this channel with a finite, representable wire value.
    channel.default_value = 0.0;
    channel.override_command = true;
    const double control = (*command_interface_ptr_)[static_cast<size_t>(control_command_index_)];
    const double effort = (*command_interface_ptr_)[static_cast<size_t>(offset_command_index_)];
    const bool enable_requested = std::isfinite(control) && control >= 0.0 && control <= 65535.0 &&
                                  std::trunc(control) == control &&
                                  (static_cast<uint16_t>(control) & 0x008fU) == 0x000fU;
    if (
      complete_ && is_operational_ && enable_requested && state_ == STATE_OPERATION_ENABLED &&
      position_command_takeover_allowed_ && std::isfinite(effort)) {
      const double raw = channel.factor * effort;
      if (std::isfinite(raw)) {
        channel.default_value = std::clamp(raw, -32768.0, 32767.0);
        accepted = true;
      }
    }
  }
  EcCiA402Drive::processData(index, address);
  if (channel.index == 0x60B2 && channel.allow_ec_write) {
    (*state_interface_ptr_)[static_cast<size_t>(raw_state_index_)] = EC_READ_S16(address);
    if (!accepted) {
      // The controller resets its fractional slew accumulator after a forced
      // zero. Using the integer PDO mirror as that accumulator would stall any
      // ramp slower than one permille per update.
      (*state_interface_ptr_)[static_cast<size_t>(epoch_state_index_)] = ++command_epoch_;
    }
  }
  if (state_ != STATE_OPERATION_ENABLED) {
    clear_offset();
  }
}
}  // namespace hw_ethercat

PLUGINLIB_EXPORT_CLASS(hw_ethercat::CspTorqueOffsetSlave, ethercat_interface::EcSlave)
