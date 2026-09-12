#include "robot_hw_ethercat/zeroerr_pp_slave.hpp"

#include <chrono>
#include <set>

#include "pluginlib/class_list_macros.hpp"

namespace robot_hw_ethercat
{
namespace
{
bool unsigned_integer(double value, double max) noexcept
{
  return std::isfinite(value) && value >= 0.0 && value <= max && std::floor(value) == value;
}
}  // namespace

bool ZeroErrPpSlave::setupSlave(std::unordered_map<std::string, std::string> parameters,
  std::vector<double> * states, std::vector<double> * commands)
{
  if (gripper_ || !states || !commands) {return false;}
  try {
    const auto file = parameters.at("slave_config");
    const auto config = YAML::LoadFile(file);
    const auto pp = config["pp"];
    if (!pp || !pp["verified"].as<bool>()) {return false;}
    PpConfig limits{
      pp["counts_per_metre"].as<double>(), pp["zero_counts"].as<double>(),
      pp["min_position"].as<double>(), pp["max_position"].as<double>(),
      pp["max_force"].as<double>(), pp["permille_per_newton"].as<double>(),
      pp["max_torque_permille"].as<uint16_t>(), pp["position_tolerance"].as<double>(),
      pp["stopped_velocity"].as<double>(), pp["ack_timeout"].as<double>(),
      pp["halt_timeout"].as<double>(), pp["stopped_cycles"].as<unsigned int>()};
    if (!PpGripper::valid_config(limits) ||
      limits.max_torque_permille > std::numeric_limits<int16_t>::max())
    {
      return false;
    }
    velocity_factor_ = pp["velocity_metres_per_unit"].as<double>();
    effort_factor_ = pp["effort_newtons_per_unit"].as<double>();
    if (!std::isfinite(velocity_factor_) || velocity_factor_ == 0.0 ||
      !std::isfinite(effort_factor_) || effort_factor_ == 0.0)
    {
      return false;
    }
    const std::array<std::string, 5> command_names{
      "position", "max_effort", "pp_sequence", "pp_halt", "control_word"};
    const std::array<std::string, 6> state_names{
      "position", "velocity", "effort", "status_word", "pp_sequence", "pp_state"};
    auto bind = [&parameters](const auto & names, auto & indices, const char * prefix, size_t size)
      {
        std::set<size_t> used;
        for (size_t i = 0; i < names.size(); ++i) {
          const auto text = parameters.at(std::string(prefix) + names[i]);
          size_t consumed = 0;
          const auto value = std::stoul(text, &consumed);
          if (consumed != text.size() || value >= size || !used.insert(value).second) {
            throw std::invalid_argument("Invalid PP interface index");
          }
          indices[i] = value;
        }
      };
    bind(command_names, commands_, "command_interface/", commands->size());
    bind(state_names, states_, "state_interface/", states->size());

    // This adapter owns these raw objects. Reject alternate units, duplicates and
    // generic command mappings that could become a second writer.
    const std::map<unsigned int, std::string> outputs{
      {0x6040, "uint16"}, {0x6072, "int16"}, {0x607a, "int32"}};
    const std::map<unsigned int, std::string> inputs{
      {0x6041, "uint16"}, {0x6061, "int8"}, {0x6064, "int32"},
      {0x606c, "int32"}, {0x6077, "int16"}};
    auto validate = [](const YAML::Node & pdos, const auto & expected, bool padding)
      {
        // ZeroErr's variable PDO maps are mutually exclusive. The 13-byte PP
        // feedback requires one trailing dummy byte for ESC word alignment.
        if (!pdos.IsSequence() || pdos.size() != 1) {return false;}
        const auto channels = pdos[0]["channels"];
        if (!channels.IsSequence() || channels.size() != expected.size() + (padding ? 1U : 0U)) {
          return false;
        }
        std::set<unsigned int> found;
        for (size_t i = 0; i < channels.size(); ++i) {
          const auto channel = channels[i];
          const auto index = channel["index"].as<unsigned int>();
          if (padding && i + 1 == channels.size()) {
            if (channel.size() != 3 || index != 0 ||
              channel["sub_index"].as<unsigned int>() != 0 ||
              channel["type"].as<std::string>() != "uint8")
            {
              return false;
            }
            continue;
          }
          if (expected.count(index) != 1 || !found.insert(index).second ||
            channel["sub_index"].as<unsigned int>() != 0 ||
            channel["type"].as<std::string>() != expected.at(index) ||
            channel["command_interface"] || channel["state_interface"] ||
            channel["factor"] || channel["offset"])
          {
            return false;
          }
        }
        return found.size() == expected.size();
      };
    if (!validate(config["rpdo"], outputs, false) ||
      !validate(config["tpdo"], inputs, true)) {return false;}
    bool mode_set = false;
    std::set<unsigned int> profile_objects;
    for (const auto & sdo : config["sdo"]) {
      const auto index = sdo["index"].as<unsigned int>();
      if (index == 0x6040 || index == 0x607a || index == 0x6072) {return false;}
      if (index == 0x6081 || index == 0x6083 || index == 0x6084 || index == 0x605d) {
        if (!profile_objects.insert(index).second || sdo["sub_index"].as<unsigned int>() != 0) {
          return false;
        }
        if (index == 0x605d) {
          if (sdo["type"].as<std::string>() != "int16") {return false;}
          sdo["value"].as<int16_t>();
        } else {
          if (sdo["type"].as<std::string>() != "uint32" || sdo["value"].as<uint32_t>() == 0) {
            return false;
          }
        }
      }
      if (index == 0x6060) {
        if (mode_set || sdo["sub_index"].as<unsigned int>() != 0 ||
          sdo["type"].as<std::string>() != "int8" || sdo["value"].as<int>() != 1)
        {
          return false;
        }
        mode_set = true;
      }
    }
    if (!mode_set || profile_objects.size() != 4 ||
      !GenericEcSlave::setupSlave(parameters, states, commands)) {return false;}
    gripper_ = std::make_unique<PpGripper>(limits);
    for (auto index : states_) {(*states)[index] = std::numeric_limits<double>::quiet_NaN();}
    (*states)[states_[4]] = 0.0;
    (*states)[states_[5]] = 0.0;
    return true;
  } catch (const std::exception & error) {
    std::cerr << "ZeroErr PP configuration rejected: " << error.what() << '\n';
    return false;
  }
}

uint16_t ZeroErrPpSlave::base_control_word() const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[4]];
  return unsigned_integer(value, 65535.0) ? static_cast<uint16_t>(value) : 0;
}

void ZeroErrPpSlave::processData(size_t index, uint8_t * address)
{
  auto & channel = pdo_channels_info_[domain_map_[index]];
  if (channel.pdo_type == ethercat_interface::RPDO) {
    switch (channel.index) {
      case 0x6040: {
        const auto base = base_control_word();
        const auto word = (base & 0x008fU) == 0x000fU ? output_.control_word : base;
        EC_WRITE_U16(address, word);
        written_ |= 1;
        break;
      }
      case 0x6072:
        EC_WRITE_S16(address, static_cast<int16_t>(output_.max_torque));
        written_ |= 2;
        break;
      case 0x607a: EC_WRITE_S32(address, output_.target); written_ |= 4; break;
      default: break;
    }
    return;
  }
  switch (channel.index) {
    case 0x6041:
      feedback_.status_word = EC_READ_U16(address);
      (*state_interface_ptr_)[states_[3]] = feedback_.status_word;
      break;
    case 0x6061: feedback_.mode = EC_READ_S8(address); break;
    case 0x6064:
      feedback_.actual_position = EC_READ_S32(address);
      have_position_ = true;
      (*state_interface_ptr_)[states_[0]] = gripper_->position(feedback_.actual_position);
      break;
    case 0x606c:
      feedback_.velocity = EC_READ_S32(address) * velocity_factor_;
      (*state_interface_ptr_)[states_[1]] = feedback_.velocity;
      break;
    case 0x6077:
      (*state_interface_ptr_)[states_[2]] = EC_READ_S16(address) * effort_factor_;
      break;
    default: break;
  }
}

void ZeroErrPpSlave::onPdoCycleRead(bool complete)
{
  if (!complete) {
    preloaded_ = false;
    (*state_interface_ptr_)[states_[5]] = 0.0;
  }
}

void ZeroErrPpSlave::onPdoCycleStart(bool complete)
{
  complete_ = complete;
  if (!complete) {preloaded_ = false;}
  written_ = 0;
  const double sequence = (*command_interface_ptr_)[commands_[2]];
  const double halt = (*command_interface_ptr_)[commands_[3]];
  PpRequest request{
    unsigned_integer(sequence, 9007199254740991.0) ? static_cast<uint64_t>(sequence) : 0,
    (*command_interface_ptr_)[commands_[0]], (*command_interface_ptr_)[commands_[1]], halt != 0.0};
  feedback_.operational = complete && is_operational_ && have_position_;
  feedback_.control_word = base_control_word();
  if (!preloaded_ && (feedback_.control_word & 0x008fU) == 0x000fU) {
    feedback_.control_word = 0;
  }
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  output_ = gripper_->update(request, feedback_, std::chrono::duration<double>(now).count());
  (*state_interface_ptr_)[states_[4]] = static_cast<double>(output_.sequence);
  (*state_interface_ptr_)[states_[5]] = static_cast<double>(output_.state);
}

void ZeroErrPpSlave::onPdoCycleSent()
{
  if (written_ == 7 && complete_ && is_operational_ && have_position_) {
    gripper_->sent();
    if (output_.target == feedback_.actual_position && feedback_.mode == 1) {preloaded_ = true;}
  }
}

void ZeroErrPpSlave::set_state_is_operational(bool operational)
{
  is_operational_ = operational;
  if (!operational) {
    preloaded_ = false;
    if (gripper_) {(*state_interface_ptr_)[states_[5]] = 0.0;}
  }
}
}  // namespace robot_hw_ethercat

PLUGINLIB_EXPORT_CLASS(robot_hw_ethercat::ZeroErrPpSlave, ethercat_interface::EcSlave)
