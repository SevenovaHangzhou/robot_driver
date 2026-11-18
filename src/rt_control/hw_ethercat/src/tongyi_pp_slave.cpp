#include "hw_ethercat/tongyi_pp_slave.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace hw_ethercat
{
namespace
{
bool unsigned_integer(double value, double max) noexcept
{
  return std::isfinite(value) && value >= 0.0 && value <= max && std::floor(value) == value;
}

using Layout = std::vector<std::pair<unsigned int, const char *>>;

// The saved, fixed layout of ESI XMC_ESC_250325 (0x1600 / 0x1A00), in wire order.
const Layout kOutputs{{0x6040, "uint16"}, {0x6071, "int16"}, {0x60ff, "int32"}, {0x607a, "int32"},
                      {0x6081, "uint32"}, {0x6060, "int8"},  {0x607f, "uint32"}};
const Layout kInputs{{0x6041, "uint16"}, {0x603f, "uint16"}, {0x6078, "int16"}, {0x606c, "int32"},
                     {0x6064, "int32"},  {0x6061, "int8"},   {0x6000, "uint8"}, {0x2300, "uint16"},
                     {0x2301, "uint16"}, {0x60fd, "uint32"}, {0x6077, "int16"}};

bool matches(const YAML::Node & pdos, unsigned int pdo_index, const Layout & expected)
{
  if (!pdos.IsSequence() || pdos.size() != 1 || pdos[0]["index"].as<unsigned int>() != pdo_index) {
    return false;
  }
  const auto channels = pdos[0]["channels"];
  if (!channels.IsSequence() || channels.size() != expected.size()) {
    return false;
  }
  for (size_t i = 0; i < expected.size(); ++i) {
    const auto channel = channels[i];
    // Generic interface bindings would become a second writer or an unscaled reader.
    if (
      channel["index"].as<unsigned int>() != expected[i].first ||
      channel["sub_index"].as<unsigned int>() != 0 ||
      channel["type"].as<std::string>() != expected[i].second || channel["command_interface"] ||
      channel["state_interface"] || channel["factor"] || channel["offset"] || channel["default"]) {
      return false;
    }
  }
  return true;
}

double seconds_now() noexcept
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

bool TongyiPpSlave::setupSlave(
  std::unordered_map<std::string, std::string> parameters, std::vector<double> * states,
  std::vector<double> * commands)
{
  if (axis_ || !states || !commands) {
    return false;
  }
  try {
    const auto config = YAML::LoadFile(parameters.at("slave_config"));
    const auto pp = config["pp"];
    if (!pp || !pp["verified"].as<bool>() || !config["use_slave_pdo_defaults"].as<bool>()) {
      return false;
    }
    const PpAxisConfig limits{
      pp["counts_per_metre"].as<double>(),    pp["zero_counts"].as<double>(),
      pp["min_position"].as<double>(),        pp["max_position"].as<double>(),
      pp["position_tolerance"].as<double>(),  pp["stopped_velocity"].as<double>(),
      pp["ack_timeout"].as<double>(),         pp["halt_timeout"].as<double>(),
      pp["stopped_cycles"].as<unsigned int>()};
    max_velocity_ = pp["max_velocity"].as<double>();
    profile_velocity_per_mps_ = pp["profile_velocity_per_metre_per_second"].as<double>();
    max_profile_velocity_ = pp["max_profile_velocity"].as<uint32_t>();
    if (
      !PpAxis::valid_config(limits) || !PpAxis::positive(max_velocity_) ||
      !PpAxis::positive(profile_velocity_per_mps_) || max_profile_velocity_ == 0 ||
      std::floor(max_velocity_ * profile_velocity_per_mps_) > max_profile_velocity_ ||
      std::floor(max_velocity_ * profile_velocity_per_mps_) < 1.0) {
      return false;
    }
    auto bind = [&parameters](
                  const auto & names, auto & indices, const char * prefix, size_t size) {
      std::set<size_t> used;
      for (size_t i = 0; i < names.size(); ++i) {
        const auto text = parameters.at(std::string(prefix) + names[i]);
        size_t consumed = 0;
        const auto value = std::stoul(text, &consumed);
        if (consumed != text.size() || value >= size || !used.insert(value).second) {
          throw std::invalid_argument("Invalid Tongyi PP interface index");
        }
        indices[i] = value;
      }
    };
    bind(kCommands, commands_, "command_interface/", commands->size());
    bind(kStates, states_, "state_interface/", states->size());
    if (!matches(config["rpdo"], 0x1600, kOutputs) || !matches(config["tpdo"], 0x1a00, kInputs)) {
      return false;
    }
    // The drive keeps no parameters written over EtherCAT across power cycles (vendor,
    // 2026-11-17): the master writes exactly the profile acceleration and deceleration.
    std::set<unsigned int> written;
    for (const auto & sdo : config["sdo"]) {
      const auto index = sdo["index"].as<unsigned int>();
      if (
        (index != 0x6083 && index != 0x6084) || !written.insert(index).second ||
        sdo["sub_index"].as<unsigned int>() != 0 || sdo["type"].as<std::string>() != "uint32" ||
        sdo["value"].as<uint32_t>() == 0) {
        return false;
      }
    }
    if (written.size() != 2 || !GenericEcSlave::setupSlave(parameters, states, commands)) {
      return false;
    }
    axis_ = std::make_unique<PpAxis>(limits);
    for (auto index : states_) {
      (*states)[index] = std::numeric_limits<double>::quiet_NaN();
    }
    (*states)[states_[kStateSequence]] = 0.0;
    (*states)[states_[kPpState]] = 0.0;
    return true;
  } catch (const std::exception & error) {
    std::cerr << "Tongyi PP configuration rejected: " << error.what() << '\n';
    return false;
  }
}

uint16_t TongyiPpSlave::base_control_word() const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[kControlWord]];
  return unsigned_integer(value, 65535.0) ? static_cast<uint16_t>(value) : 0;
}

void TongyiPpSlave::processData(size_t index, uint8_t * address)
{
  const auto & channel = pdo_channels_info_[domain_map_[index]];
  if (channel.pdo_type == ethercat_interface::RPDO) {
    switch (channel.index) {
      case 0x6040: {
        const auto base = base_control_word();
        EC_WRITE_U16(address, (base & 0x008fU) == 0x000fU ? output_.control_word : base);
        written_ |= 1U;
        break;
      }
      case 0x6071:
        EC_WRITE_S16(address, 0);
        written_ |= 2U;
        break;
      case 0x60ff:
        EC_WRITE_S32(address, 0);
        written_ |= 4U;
        break;
      case 0x607a:
        EC_WRITE_S32(address, output_.target);
        written_ |= 8U;
        break;
      case 0x6081:
        EC_WRITE_U32(address, profile_velocity_);
        written_ |= 16U;
        break;
      case 0x6060:
        EC_WRITE_S8(address, 1);
        written_ |= 32U;
        break;
      case 0x607f:
        EC_WRITE_U32(address, max_profile_velocity_);
        written_ |= 64U;
        break;
      default:
        break;
    }
    return;
  }
  switch (channel.index) {
    case 0x6041:
      feedback_.status_word = EC_READ_U16(address);
      state(kStatusWord) = feedback_.status_word;
      break;
    case 0x603f:
      state(kErrorCode) = EC_READ_U16(address);
      break;
    case 0x6078:
      state(kCurrentRaw) = EC_READ_S16(address);
      break;
    case 0x606c:
      state(kVelocityRaw) = EC_READ_S32(address);
      break;
    case 0x6064:
      feedback_.actual_position = EC_READ_S32(address);
      have_position_ = true;
      state(kStatePosition) = axis_->position(feedback_.actual_position);
      break;
    case 0x6061:
      feedback_.mode = EC_READ_S8(address);
      state(kModeDisplay) = feedback_.mode;
      break;
    case 0x2300:
      state(kBridgeTemperature) = EC_READ_U16(address);
      break;
    case 0x2301:
      state(kMotorTemperature) = EC_READ_U16(address);
      break;
    case 0x60fd:
      state(kDigitalInputs) = EC_READ_U32(address);
      break;
    case 0x6077:
      state(kTorqueRaw) = EC_READ_S16(address);
      break;
    default:
      break;
  }
}

void TongyiPpSlave::onPdoCycleRead(bool complete)
{
  if (!complete) {
    preloaded_ = false;
    have_previous_ = false;
    state(kPpState) = 0.0;
  }
}

void TongyiPpSlave::onPdoCycleStart(bool complete)
{
  complete_ = complete;
  if (!complete) {
    preloaded_ = false;
  }
  written_ = 0;
  const double now = seconds_now();
  // Velocity is the position derivative between consecutive complete samples.
  feedback_.velocity = std::numeric_limits<double>::quiet_NaN();
  if (complete && have_position_) {
    if (have_previous_ && now > previous_time_) {
      feedback_.velocity =
        (axis_->position(feedback_.actual_position) - axis_->position(previous_position_)) /
        (now - previous_time_);
    }
    previous_position_ = feedback_.actual_position;
    previous_time_ = now;
    have_previous_ = true;
  }
  state(kVelocity) = feedback_.velocity;

  const auto & command = *command_interface_ptr_;
  const double sequence = command[commands_[kSequence]];
  const double velocity = command[commands_[kProfileVelocity]];
  const PpAxisRequest request{
    unsigned_integer(sequence, 9007199254740991.0) ? static_cast<uint64_t>(sequence) : 0,
    command[commands_[kPosition]], command[commands_[kHalt]] != 0.0};
  const bool velocity_valid = PpAxis::positive(velocity) && velocity <= max_velocity_ &&
                              velocity * profile_velocity_per_mps_ >= 1.0;
  feedback_.operational = complete && is_operational_ && have_position_;
  feedback_.control_word = base_control_word();
  if (!preloaded_ && (feedback_.control_word & 0x008fU) == 0x000fU) {
    feedback_.control_word = 0;
  }
  output_ = axis_->update(request, feedback_, now, velocity_valid);
  if (axis_->accepted()) {
    // Round downward: the conversion must not exceed the requested velocity.
    profile_velocity_ = static_cast<uint32_t>(std::floor(velocity * profile_velocity_per_mps_));
  }
  state(kStateSequence) = static_cast<double>(output_.sequence);
  state(kPpState) = static_cast<double>(output_.state);
}

void TongyiPpSlave::onPdoCycleSent()
{
  if (written_ == 0x7fU && complete_ && is_operational_ && have_position_) {
    axis_->sent();
    if (output_.target == feedback_.actual_position && feedback_.mode == 1) {
      preloaded_ = true;
    }
  }
}

void TongyiPpSlave::set_state_is_operational(bool operational)
{
  is_operational_ = operational;
  if (!operational) {
    preloaded_ = false;
    have_previous_ = false;
    if (axis_) {
      state(kPpState) = 0.0;
    }
  }
}
}  // namespace hw_ethercat

PLUGINLIB_EXPORT_CLASS(hw_ethercat::TongyiPpSlave, ethercat_interface::EcSlave)
