#include "dm_swerve_driver/bringup_audit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace dm_swerve_driver {
namespace {

[[nodiscard]] std::uint16_t parse_can_id(
  const std::string & text, const char * option)
{
  std::size_t consumed{0U};
  unsigned long value{0U};
  try {
    value = std::stoul(text, &consumed, 0);
  } catch (const std::exception &) {
    throw std::invalid_argument{std::string{option} + " requires numeric CAN IDs"};
  }
  if (consumed != text.size() || value > 0x7FFUL) {
    throw std::invalid_argument{std::string{option} + " CAN IDs must fit in 11 bits"};
  }
  return static_cast<std::uint16_t>(value);
}

[[nodiscard]] AuditMotor parse_motor(const std::string & text, bool steering)
{
  const auto separator = text.find(':');
  if (separator == std::string::npos || separator == 0U || separator + 1U >= text.size()) {
    throw std::invalid_argument{"motor specification must be ESC_ID:MST_ID"};
  }
  return AuditMotor{
    parse_can_id(text.substr(0U, separator), "motor"),
    parse_can_id(text.substr(separator + 1U), "motor"),
    steering};
}

[[nodiscard]] double parse_positive_double(
  const std::string & text, const char * option)
{
  std::size_t consumed{0U};
  double value{0.0};
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument{std::string{option} + " requires a number"};
  }
  if (consumed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument{std::string{option} + " must be finite and positive"};
  }
  return value;
}

[[nodiscard]] long parse_positive_integer(
  const std::string & text, const char * option)
{
  std::size_t consumed{0U};
  long value{0};
  try {
    value = std::stol(text, &consumed, 10);
  } catch (const std::exception &) {
    throw std::invalid_argument{std::string{option} + " requires an integer"};
  }
  if (consumed != text.size() || value <= 0) {
    throw std::invalid_argument{std::string{option} + " must be positive"};
  }
  return value;
}

void validate_motors(const std::vector<AuditMotor> & motors)
{
  if (motors.empty()) {
    throw std::invalid_argument{"at least one motor must be selected"};
  }
  std::set<std::uint16_t> esc_ids;
  std::set<std::uint16_t> mst_ids;
  for (const auto & motor : motors) {
    if (!esc_ids.insert(motor.esc_id).second) {
      throw std::invalid_argument{"ESC_ID values must be unique"};
    }
    if (!mst_ids.insert(motor.mst_id).second) {
      throw std::invalid_argument{"MST_ID values must be unique"};
    }
    if (motor.mst_id == kRegisterCanId) {
      throw std::invalid_argument{"MST_ID cannot equal register command ID 0x7FF"};
    }
    if (motor.esc_id == kRegisterCanId) {
      throw std::invalid_argument{"ESC_ID cannot equal register command ID 0x7FF"};
    }
  }
  const bool overlap = std::any_of(
    esc_ids.begin(), esc_ids.end(), [&](std::uint16_t id) {
      return mst_ids.count(id) != 0U;
    });
  if (overlap) {
    throw std::invalid_argument{"ESC_ID and MST_ID sets must not overlap"};
  }
}

}  // namespace

BringupAuditOptions default_bringup_audit_options()
{
  BringupAuditOptions options;
  for (std::uint16_t index{0U}; index < 8U; ++index) {
    options.motors.push_back(AuditMotor{
        static_cast<std::uint16_t>(index + 1U),
        static_cast<std::uint16_t>(0x11U + index),
        index < 4U});
  }
  return options;
}

BringupAuditOptions parse_bringup_audit_arguments(
  const std::vector<std::string> & arguments)
{
  BringupAuditOptions options{default_bringup_audit_options()};
  bool explicit_motors{false};
  for (std::size_t index{0U}; index < arguments.size(); ++index) {
    const std::string & option = arguments[index];
    const auto require_value = [&](const char * name) -> const std::string & {
        if (++index >= arguments.size()) {
          throw std::invalid_argument{std::string{name} + " requires a value"};
        }
        return arguments[index];
      };
    if (option == "--interface") {
      options.interface_name = require_value("--interface");
    } else if (option == "--motor" || option == "--steering") {
      if (!explicit_motors) {
        options.motors.clear();
        explicit_motors = true;
      }
      options.motors.push_back(parse_motor(
          require_value(option.c_str()), option == "--steering"));
    } else if (option == "--pmax") {
      options.expected_limits.position_max =
        parse_positive_double(require_value("--pmax"), "--pmax");
    } else if (option == "--vmax") {
      options.expected_limits.velocity_max =
        parse_positive_double(require_value("--vmax"), "--vmax");
    } else if (option == "--tmax") {
      options.expected_limits.torque_max =
        parse_positive_double(require_value("--tmax"), "--tmax");
    } else if (option == "--deadline-us") {
      options.feedback_deadline = std::chrono::microseconds{
        parse_positive_integer(require_value("--deadline-us"), "--deadline-us")};
    } else if (option == "--retries") {
      const long retries{parse_positive_integer(require_value("--retries"), "--retries")};
      if (retries > std::numeric_limits<int>::max()) {
        throw std::invalid_argument{"--retries is too large"};
      }
      options.retries = static_cast<int>(retries);
    } else {
      throw std::invalid_argument{"unknown option: " + option};
    }
  }
  if (options.interface_name.empty()) {
    throw std::invalid_argument{"--interface must not be empty"};
  }
  validate_motors(options.motors);
  return options;
}

std::string bringup_audit_usage()
{
  return
    "Usage: dm_swerve_bringup_check [options]\n"
    "  --interface NAME       SocketCAN interface (default: can0)\n"
    "  --steering ESC:MST     Add steering motor; repeat as needed\n"
    "  --motor ESC:MST        Add non-steering motor; repeat as needed\n"
    "  --pmax VALUE           Expected/fallback PMAX\n"
    "  --vmax VALUE           Expected/fallback VMAX\n"
    "  --tmax VALUE           Expected/fallback TMAX\n"
    "  --deadline-us VALUE    Reply deadline per attempt\n"
    "  --retries VALUE        Register/feedback attempts\n"
    "  --help                 Show this text\n";
}

}  // namespace dm_swerve_driver
