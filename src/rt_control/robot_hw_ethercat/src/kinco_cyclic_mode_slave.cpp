#include "robot_hw_ethercat/kinco_cyclic_mode_slave.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace robot_hw_ethercat
{
namespace
{
bool exact_integer(double value, double maximum) noexcept
{
  return std::isfinite(value) && value >= 0.0 && value <= maximum && std::floor(value) == value;
}

bool finite_positive(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

bool valid_raw(double value) noexcept
{
  return std::isfinite(value) &&
         value >= static_cast<double>(std::numeric_limits<int32_t>::min()) &&
         value <= static_cast<double>(std::numeric_limits<int32_t>::max());
}
}  // namespace

bool KincoCyclicModeSlave::setupSlave(
  std::unordered_map<std::string, std::string> parameters,
  std::vector<double> * states, std::vector<double> * commands)
{
  if (!states || !commands || startup_mode_ != 0) {
    return false;
  }
  try {
    const auto config = YAML::LoadFile(parameters.at("slave_config"));
    const auto kinco = config["kinco_cyclic_mode"];
    if (!kinco || !kinco["verified"].as<bool>() ||
      !config["use_slave_pdo_defaults"].as<bool>())
    {
      return false;
    }
    const bool mock_only = kinco["mock_only"].as<bool>();
    const bool allow_mock = parameters.count("allow_mock_profile") != 0 &&
      parameters.at("allow_mock_profile") == "true";
    if (mock_only && !allow_mock) {
      return false;
    }

    position_counts_per_unit_ = kinco["position_counts_per_unit"].as<double>();
    velocity_counts_per_unit_ = kinco["velocity_counts_per_unit"].as<double>();
    position_offset_counts_ = kinco["position_offset_counts"].as<double>();
    min_position_ = kinco["min_position"].as<double>();
    max_position_ = kinco["max_position"].as<double>();
    max_abs_velocity_ = kinco["max_abs_velocity"].as<double>();
    stationary_velocity_ = kinco["stationary_velocity"].as<double>();
    feedback_timeout_seconds_ = kinco["feedback_timeout_seconds"].as<double>();
    mode_ack_timeout_seconds_ = kinco["mode_ack_timeout_seconds"].as<double>();
    if (!finite_positive(std::abs(position_counts_per_unit_)) ||
      !finite_positive(std::abs(velocity_counts_per_unit_)) ||
      !std::isfinite(position_offset_counts_) || !std::isfinite(min_position_) ||
      !std::isfinite(max_position_) || min_position_ >= max_position_ ||
      !finite_positive(max_abs_velocity_) || !finite_positive(stationary_velocity_) ||
      stationary_velocity_ > max_abs_velocity_ || !finite_positive(feedback_timeout_seconds_) ||
      !finite_positive(mode_ack_timeout_seconds_))
    {
      return false;
    }
    const double raw_min = min_position_ * position_counts_per_unit_ + position_offset_counts_;
    const double raw_max = max_position_ * position_counts_per_unit_ + position_offset_counts_;
    if (!valid_raw(raw_min) || !valid_raw(raw_max) ||
      !valid_raw(max_abs_velocity_ * std::abs(velocity_counts_per_unit_)))
    {
      return false;
    }

    size_t consumed = 0;
    const std::string startup_text = parameters.at("mode_of_operation");
    const int startup = std::stoi(startup_text, &consumed);
    if (consumed != startup_text.size() || (startup != kCsp && startup != kCsv)) {
      return false;
    }
    startup_mode_ = static_cast<int8_t>(startup);

    const std::array<std::string, kCommandCount> command_names{
      "position", "velocity", "mode_of_operation", "control_word", "write_sequence", "write_mask"};
    const std::array<std::string, kStateCount> state_names{
      "position", "velocity", "mode_of_operation_display", "status_word", "feedback_age_ms",
      "mode_switch_ready", "mode_request_ack", "command_fresh", "mode_request_error",
      "feedback_sequence", "sent_sequence", "feedback_sequence_at_send", "sent_velocity"};
    auto bind =
      [&parameters](const auto & names, auto & indices, const char * prefix, size_t size) {
        std::set<size_t> used;
        for (size_t index = 0; index < names.size(); ++index) {
          const std::string text = parameters.at(std::string(prefix) + names[index]);
          size_t parsed = 0;
          const size_t value = std::stoul(text, &parsed);
          if (parsed != text.size() || value >= size || !used.insert(value).second) {
            throw std::invalid_argument("invalid Kinco interface index");
          }
          indices[index] = value;
        }
      };
    bind(command_names, commands_, "command_interface/", commands->size());
    bind(state_names, states_, "state_interface/", states->size());

    const std::map<unsigned int, std::string> outputs{
      {0x6040, "uint16"}, {0x607a, "int32"}, {0x60ff, "int32"}, {0x6060, "int8"}};
    const std::map<unsigned int, std::string> inputs{
      {0x6041, "uint16"}, {0x6064, "int32"}, {0x606c, "int32"}, {0x6061, "int8"}};
    auto validate_pdo = [](const YAML::Node & pdos, const auto & expected) {
        if (!pdos.IsSequence() || pdos.size() != 1) {
          return false;
        }
        const auto channels = pdos[0]["channels"];
        if (!channels.IsSequence() || channels.size() != expected.size()) {
          return false;
        }
        std::set<unsigned int> found;
        for (const auto & channel : channels) {
          const auto object = channel["index"].as<unsigned int>();
          if (expected.count(object) != 1 || !found.insert(object).second ||
            channel["sub_index"].as<unsigned int>() != 0 ||
            channel["type"].as<std::string>() != expected.at(object) ||
            channel["command_interface"] || channel["state_interface"] || channel["factor"] ||
            channel["offset"] || channel["default"])
          {
            return false;
          }
        }
        return found.size() == expected.size();
      };
    if (!validate_pdo(config["rpdo"], outputs) || !validate_pdo(config["tpdo"], inputs)) {
      return false;
    }
    if (config["sdo"]) {
      for (const auto & sdo : config["sdo"]) {
        if (sdo["index"].as<unsigned int>() == 0x6060U) {
          return false;
        }
      }
    }
    if (!GenericEcSlave::setupSlave(parameters, states, commands)) {
      return false;
    }
    for (const size_t index : states_) {
      (*states)[index] = std::numeric_limits<double>::quiet_NaN();
    }
    set_state(kFeedbackAge, std::numeric_limits<double>::infinity());
    set_state(kModeSwitchReady, 0.0);
    set_state(kModeRequestAck, 0.0);
    set_state(kCommandFresh, 0.0);
    set_state(kModeRequestError, 0.0);
    set_state(kFeedbackSequence, 0.0);
    set_state(kSentSequence, 0.0);
    set_state(kFeedbackSequenceAtSend, 0.0);
    set_state(kSentVelocity, std::numeric_limits<double>::quiet_NaN());
    invalidate_velocity_command();
    return true;
  } catch (const std::exception & error) {
    std::cerr << "Kinco cyclic-mode configuration rejected: " << error.what() << '\n';
    startup_mode_ = 0;
    return false;
  }
}

void KincoCyclicModeSlave::processData(size_t index, uint8_t * address)
{
  auto & channel = pdo_channels_info_[domain_map_[index]];
  if (channel.pdo_type == ethercat_interface::RPDO) {
    switch (channel.index) {
      case 0x6040: EC_WRITE_U16(address, output_control_word_); written_outputs_ |= 0x01U; break;
      case 0x607a: EC_WRITE_S32(address, output_position_raw_); written_outputs_ |= 0x02U; break;
      case 0x60ff: EC_WRITE_S32(address, output_velocity_raw_); written_outputs_ |= 0x04U; break;
      case 0x6060: EC_WRITE_S8(address, output_mode_); written_outputs_ |= 0x08U; break;
      default: break;
    }
    return;
  }
  if (!cycle_complete_) {
    return;
  }
  switch (channel.index) {
    case 0x6041:
      status_word_ = EC_READ_U16(address);
      have_status_ = true;
      set_state(kStatusWord, status_word_);
      break;
    case 0x6064:
      actual_position_raw_ = EC_READ_S32(address);
      have_position_ = true;
      set_state(
        kActualPosition,
        (static_cast<double>(actual_position_raw_) - position_offset_counts_) /
        position_counts_per_unit_);
      break;
    case 0x606c:
      actual_velocity_raw_ = EC_READ_S32(address);
      have_velocity_ = true;
      set_state(kActualVelocity, actual_velocity_raw_ / velocity_counts_per_unit_);
      break;
    case 0x6061:
      mode_display_ = EC_READ_S8(address);
      have_mode_ = true;
      set_state(kModeDisplay, mode_display_);
      break;
    default: break;
  }
  if (have_position_ && have_velocity_ && have_status_ && have_mode_) {
    if (feedback_sequence_ >= 9007199254740991ULL) {clear_handoff(true); return;}
    set_state(kFeedbackSequence, static_cast<double>(++feedback_sequence_));
  }
}

void KincoCyclicModeSlave::onPdoCycleRead(bool complete)
{
  have_position_ = have_velocity_ = have_status_ = have_mode_ = false;
  read_hook_seen_ = true;
  cycle_complete_ = complete;
  if (complete) {
    last_complete_feedback_ = std::chrono::steady_clock::now();
    set_state(kFeedbackAge, 0.0);
  } else {
    startup_seed_sent_ = false;
    clear_handoff(true);
    set_state(kFeedbackAge, std::numeric_limits<double>::infinity());
  }
}

void KincoCyclicModeSlave::onPdoCycleStart(bool complete)
{
  const auto now = std::chrono::steady_clock::now();
  cycle_complete_ = complete;
  if (complete && !read_hook_seen_) {
    last_complete_feedback_ = now;
  }
  written_outputs_ = 0U;
  const double sequence = (*command_interface_ptr_)[commands_[kWriteSequence]];
  const double mask = (*command_interface_ptr_)[commands_[kWriteMask]];
  pending_valid_ = exact_integer(sequence, 9007199254740991.0) && sequence > 0 &&
    exact_integer(mask, 7.0) && mask > 0;
  pending_sequence_ = pending_valid_ ? static_cast<uint64_t>(sequence) : 0;
  pending_mask_ = pending_valid_ ? static_cast<uint8_t>(mask) : 0;
  pending_mode_ = requested_mode();
  if ((pending_mask_ & 1U) != 0) {
    pending_valid_ = valid_velocity_command(pending_velocity_) && pending_valid_;
  }
  if ((pending_mask_ & 2U) != 0) {
    pending_valid_ = valid_position_command(pending_position_) && pending_valid_;
  }
  if ((pending_mask_ & 4U) != 0) {pending_valid_ = pending_mode_ != 0 && pending_valid_;}
  const uint16_t requested_word = requested_control_word();
  output_control_word_ = complete && is_operational_ ? requested_word : 0U;
  if (!startup_seed_sent_ && (output_control_word_ & 0x008fU) == 0x000fU) {
    output_control_word_ = 0x0007U;
  }
  output_position_raw_ = have_position_ ? actual_position_raw_ : 0;
  output_velocity_raw_ = 0;
  output_mode_ = have_mode_ && (mode_display_ == kCsp || mode_display_ == kCsv) ?
    mode_display_ : startup_mode_;
  set_state(kModeSwitchReady, 0.0);
  set_state(kModeRequestAck, 0.0);
  set_state(kCommandFresh, 0.0);

  if (!complete) {
    startup_seed_sent_ = false;
    clear_handoff(true);
    return;
  }
  if (!is_operational_ || !have_position_ || !have_velocity_ || !have_status_ || !have_mode_) {
    startup_seed_sent_ = false;
    clear_handoff(false);
    return;
  }
  if (!feedback_fresh(now) || (mode_display_ != kCsp && mode_display_ != kCsv)) {
    startup_seed_sent_ = false;
    clear_handoff(true);
    return;
  }
  set_state(
    kFeedbackAge,
    std::chrono::duration<double, std::milli>(now - last_complete_feedback_).count());

  if (!startup_seed_sent_) {
    output_position_raw_ = actual_position_raw_;
    output_velocity_raw_ = 0;
    output_mode_ = mode_display_;
    startup_seed_pending_ = true;
    invalidate_velocity_command();
    return;
  }

  if (error_latched_) {
    set_state(kModeRequestError, 1.0);
    return;
  }

  const int desired = requested_mode();
  if (desired != kCsp && desired != kCsv) {
    clear_handoff(true);
    return;
  }
  if (!operation_enabled()) {
    clear_handoff(false);
    return;
  }

  if (mode_request_in_flight_) {
    if (desired != mode_request_target_ ||
      std::chrono::duration<double>(now - mode_request_started_).count() >
      mode_ack_timeout_seconds_)
    {
      clear_handoff(true);
      return;
    }
    output_mode_ = mode_request_target_;
    if (mode_request_target_ == kCsp) {
      output_position_raw_ = csp_seed_raw_;
      set_state(kModeSwitchReady, csp_preseed_sent_ ? 1.0 : 0.0);
      if (mode_display_ == kCsp) {
        mode_request_in_flight_ = false;
        csp_preseed_sent_ = false;
        set_state(kModeRequestAck, 1.0);
      }
    } else {
      output_velocity_raw_ = 0;
      invalidate_velocity_command();
      if (mode_display_ == kCsv) {
        csv_zero_pending_ = true;
      }
    }
    return;
  }

  if (desired != mode_display_) {
    if (mode_display_ == kCsv && desired == kCsp) {
      int32_t requested_position = 0;
      const double actual_velocity = actual_velocity_raw_ / velocity_counts_per_unit_;
      if (!valid_position_command(requested_position) ||
        requested_position != actual_position_raw_ ||
        std::abs(actual_velocity) > stationary_velocity_)
      {
        return;
      }
      output_position_raw_ = actual_position_raw_;
      csp_seed_raw_ = actual_position_raw_;
      if (!csp_preseed_sent_) {
        csp_preseed_pending_ = true;
        return;
      }
      set_state(kModeSwitchReady, 1.0);
      output_mode_ = kCsp;
      mode_request_target_ = kCsp;
    } else if (mode_display_ == kCsp && desired == kCsv) {
      output_mode_ = kCsv;
      output_velocity_raw_ = 0;
      invalidate_velocity_command();
      mode_request_target_ = kCsv;
    } else {
      clear_handoff(true);
      return;
    }
    mode_request_in_flight_ = true;
    mode_request_started_ = now;
    return;
  }

  set_state(kModeSwitchReady, 1.0);
  set_state(kModeRequestAck, 1.0);
  if (desired == kCsp) {
    int32_t target = 0;
    if (valid_position_command(target)) {
      output_position_raw_ = target;
      set_state(kCommandFresh, 1.0);
    } else {
      set_state(kModeRequestError, 1.0);
    }
    return;
  }

  int32_t target_velocity = 0;
  if (csv_post_ack_quarantine_) {
    csv_post_ack_quarantine_ = false;
    invalidate_velocity_command();
    return;
  }
  if (csv_requires_fresh_command_) {
    if (valid_velocity_command(target_velocity)) {
      csv_requires_fresh_command_ = false;
      output_velocity_raw_ = target_velocity;
      set_state(kCommandFresh, 1.0);
    }
  } else if (valid_velocity_command(target_velocity)) {
    output_velocity_raw_ = target_velocity;
    set_state(kCommandFresh, 1.0);
  } else {
    set_state(kModeRequestError, 1.0);
  }
}

void KincoCyclicModeSlave::onPdoCycleSent()
{
  if (written_outputs_ != kAllOutputs || !cycle_complete_ || !is_operational_) {
    return;
  }
  set_state(kSentVelocity, output_velocity_raw_ / velocity_counts_per_unit_);
  if (pending_valid_ && !error_latched_ && operation_enabled() &&
    pending_sequence_ > sent_sequence_ && feedback_sequence_ != 0 &&
    ((pending_mask_ & 1U) == 0 || pending_velocity_ == output_velocity_raw_) &&
    ((pending_mask_ & 2U) == 0 || pending_position_ == output_position_raw_) &&
    ((pending_mask_ & 4U) == 0 || pending_mode_ == output_mode_))
  {
    sent_sequence_ = pending_sequence_;
    set_state(kSentSequence, static_cast<double>(sent_sequence_));
    set_state(kFeedbackSequenceAtSend, static_cast<double>(feedback_sequence_));
  }
  if (startup_seed_pending_ && have_position_ && have_velocity_ && have_mode_) {
    startup_seed_pending_ = false;
    startup_seed_sent_ = true;
    if (mode_display_ == kCsv) {
      csv_requires_fresh_command_ = true;
      invalidate_velocity_command();
    }
  }
  if (csp_preseed_pending_ && output_position_raw_ == actual_position_raw_ &&
    output_velocity_raw_ == 0 && output_mode_ == kCsv)
  {
    csp_preseed_pending_ = false;
    csp_preseed_sent_ = true;
    set_state(kModeSwitchReady, 1.0);
  }
  if (csv_zero_pending_ && output_velocity_raw_ == 0 && output_mode_ == kCsv) {
    csv_zero_pending_ = false;
    mode_request_in_flight_ = false;
    csv_requires_fresh_command_ = true;
    csv_post_ack_quarantine_ = true;
    invalidate_velocity_command();
    set_state(kModeRequestAck, 1.0);
  }
}

bool KincoCyclicModeSlave::initialized()
{
  return is_operational_ && cycle_complete_ && startup_seed_sent_ && have_position_ &&
         have_velocity_ && have_status_ && have_mode_;
}

void KincoCyclicModeSlave::set_state_is_operational(bool operational)
{
  is_operational_ = operational;
  if (!operational) {
    set_state(kSentSequence, 0.0);
    set_state(kFeedbackSequenceAtSend, 0.0);
    set_state(kSentVelocity, std::numeric_limits<double>::quiet_NaN());
    cycle_complete_ = false;
    startup_seed_pending_ = false;
    startup_seed_sent_ = false;
    error_latched_ = false;
    clear_handoff(false);
    set_state(kFeedbackAge, std::numeric_limits<double>::infinity());
    invalidate_velocity_command();
  }
}

bool KincoCyclicModeSlave::feedback_fresh(
  std::chrono::steady_clock::time_point now) const noexcept
{
  return last_complete_feedback_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::duration<double>(now - last_complete_feedback_).count() <=
         feedback_timeout_seconds_;
}

bool KincoCyclicModeSlave::operation_enabled() const noexcept
{
  return (status_word_ & 0x006fU) == 0x0027U;
}

bool KincoCyclicModeSlave::valid_position_command(int32_t & raw) const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[kPosition]];
  const double converted = value * position_counts_per_unit_ + position_offset_counts_;
  if (!std::isfinite(value) || value < min_position_ || value > max_position_ ||
    !valid_raw(converted))
  {
    return false;
  }
  raw = static_cast<int32_t>(std::llround(converted));
  return true;
}

bool KincoCyclicModeSlave::valid_velocity_command(int32_t & raw) const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[kVelocity]];
  const double converted = value * velocity_counts_per_unit_;
  if (!std::isfinite(value) || std::abs(value) > max_abs_velocity_ || !valid_raw(converted)) {
    return false;
  }
  raw = static_cast<int32_t>(std::llround(converted));
  return true;
}

int KincoCyclicModeSlave::requested_mode() const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[kMode]];
  if (!exact_integer(value, 127.0)) {
    return 0;
  }
  const int mode = static_cast<int>(value);
  return mode == kCsp || mode == kCsv ? mode : 0;
}

uint16_t KincoCyclicModeSlave::requested_control_word() const noexcept
{
  const double value = (*command_interface_ptr_)[commands_[kControlWord]];
  return exact_integer(value, 65535.0) ? static_cast<uint16_t>(value) : 0U;
}

void KincoCyclicModeSlave::clear_handoff(bool error) noexcept
{
  if (error) {
    error_latched_ = true;
  }
  csp_preseed_pending_ = false;
  csp_preseed_sent_ = false;
  mode_request_in_flight_ = false;
  csv_zero_pending_ = false;
  csv_post_ack_quarantine_ = false;
  mode_request_target_ = 0;
  set_state(kModeSwitchReady, 0.0);
  set_state(kModeRequestAck, 0.0);
  set_state(kCommandFresh, 0.0);
  set_state(kModeRequestError, error ? 1.0 : 0.0);
}

void KincoCyclicModeSlave::set_state(State state, double value) noexcept
{
  (*state_interface_ptr_)[states_[state]] = value;
}

void KincoCyclicModeSlave::invalidate_velocity_command() noexcept
{
  (*command_interface_ptr_)[commands_[kVelocity]] = std::numeric_limits<double>::quiet_NaN();
}
}  // namespace robot_hw_ethercat

PLUGINLIB_EXPORT_CLASS(robot_hw_ethercat::KincoCyclicModeSlave, ethercat_interface::EcSlave)
