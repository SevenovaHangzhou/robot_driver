#include "robot_hw_can/damiao_system.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hw_can
{
namespace
{

const auto kLogger = rclcpp::get_logger("robot_hw_can.DamiaoSystem");
const std::set<std::string> kStateInterfaces{
  "position", "velocity", "effort", "fault_code", "mos_temperature",
  "motor_temperature", "feedback_age_ms", "enabled"};

[[nodiscard]] const std::string & required_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & owner, const char * name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    throw std::invalid_argument{owner + " requires parameter " + name};
  }
  return found->second;
}

template<typename Integer>
[[nodiscard]] Integer parse_integer(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & owner, const char * name)
{
  const auto & text = required_parameter(parameters, owner, name);
  Integer value{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw std::invalid_argument{owner + " parameter " + name + " must be an integer"};
  }
  return value;
}

[[nodiscard]] double parse_double_text(
  const std::string & text, const std::string & description)
{
  std::size_t consumed{0U};
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument{description + " must be finite"};
  }
  return value;
}

[[nodiscard]] double parse_parameter_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & owner, const char * name)
{
  return parse_double_text(
    required_parameter(parameters, owner, name), owner + " parameter " + name);
}

[[nodiscard]] std::set<std::string> interface_names(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  std::set<std::string> result;
  for (const auto & interface : interfaces) {
    result.insert(interface.name);
  }
  return result;
}

[[nodiscard]] int remaining_milliseconds(
  std::chrono::steady_clock::time_point deadline) noexcept
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining = deadline - now;
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
  return static_cast<int>(milliseconds.count() + (milliseconds < remaining ? 1 : 0));
}

}  // namespace

DamiaoSystem::~DamiaoSystem() noexcept
{
  send_disable();
  socket_.close();
}

hardware_interface::CallbackReturn DamiaoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  try {
    if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!info.sensors.empty() || !info.gpios.empty() || info.joints.size() != 2U) {
      throw std::invalid_argument{"DaMiao head system requires exactly two joints"};
    }
    can_interface_ = required_parameter(info.hardware_parameters, info.name, "can_interface");
    configure_timeout_ms_ = parse_integer<int>(
      info.hardware_parameters, info.name, "configure_timeout_ms");
    feedback_timeout_ms_ = parse_integer<int>(
      info.hardware_parameters, info.name, "feedback_timeout_ms");
    disabled_poll_interval_ms_ = parse_integer<int>(
      info.hardware_parameters, info.name, "disabled_poll_interval_ms");
    max_rx_frames_per_cycle_ = parse_integer<std::size_t>(
      info.hardware_parameters, info.name, "max_rx_frames_per_cycle");
    if (configure_timeout_ms_ <= 0 || feedback_timeout_ms_ <= 0 ||
      disabled_poll_interval_ms_ <= 0 ||
      max_rx_frames_per_cycle_ == 0U || max_rx_frames_per_cycle_ > 256U)
    {
      throw std::invalid_argument{
              "timeouts must be positive and max_rx_frames_per_cycle must be in [1, 256]"};
    }

    motors_.clear();
    motors_.reserve(info.joints.size());
    std::set<std::uint16_t> can_ids;
    std::set<std::uint16_t> master_ids;
    for (const auto & joint : info.joints) {
      if (interface_names(joint.state_interfaces) != kStateInterfaces ||
        joint.state_interfaces.size() != kStateInterfaces.size() ||
        joint.command_interfaces.size() != 1U ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument{
                joint.name + " requires one position command and the eight documented states"};
      }
      const auto can_id = parse_integer<std::uint16_t>(joint.parameters, joint.name, "can_id");
      const auto master_id =
        parse_integer<std::uint16_t>(joint.parameters, joint.name, "master_id");
      if (can_id == 0U || can_id > 0x0FU || master_id > 0x7FFU) {
        throw std::invalid_argument{joint.name + " CAN ID must be [1,15] and master ID [0,2047]"};
      }
      if (!can_ids.insert(can_id).second || !master_ids.insert(master_id).second) {
        throw std::invalid_argument{"DaMiao CAN IDs and master IDs must be unique"};
      }
      const double velocity_limit =
        parse_parameter_double(joint.parameters, joint.name, "velocity_limit");
      const auto & command = joint.command_interfaces[0];
      if (command.min.empty() || command.max.empty()) {
        throw std::invalid_argument{joint.name + " position command requires min and max"};
      }
      const double command_min = parse_double_text(command.min, joint.name + " position min");
      const double command_max = parse_double_text(command.max, joint.name + " position max");
      if (velocity_limit <= 0.0 || command_min >= command_max) {
        throw std::invalid_argument{joint.name + " has invalid velocity or position limits"};
      }
      Motor motor{};
      motor.can_id = can_id;
      motor.master_id = master_id;
      motor.velocity_limit = velocity_limit;
      motor.command_min = command_min;
      motor.command_max = command_max;
      motor.command = std::numeric_limits<double>::quiet_NaN();
      motor.position = std::numeric_limits<double>::quiet_NaN();
      motor.velocity = std::numeric_limits<double>::quiet_NaN();
      motor.effort = std::numeric_limits<double>::quiet_NaN();
      motor.feedback_age_ms = std::numeric_limits<double>::infinity();
      motors_.push_back(motor);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    motors_.clear();
    RCLCPP_ERROR(kLogger, "DaMiao configuration rejected: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn DamiaoSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    std::vector<std::uint16_t> receive_ids;
    receive_ids.reserve(motors_.size());
    for (const auto & motor : motors_) {
      receive_ids.push_back(motor.master_id);
    }
    socket_.open(can_interface_, receive_ids);
    for (std::size_t index{0U}; index < motors_.size(); ++index) {
      const MotorConfiguration configuration{
        read_register(index, Register::can_id),
        read_register(index, Register::master_id),
        read_register(index, Register::can_timeout),
        read_register(index, Register::can_bitrate)};
      validate_motor_configuration(
        configuration, motors_[index].can_id, motors_[index].master_id);
      const auto mode = read_register(index, Register::control_mode);
      if (mode != kPositionVelocityMode) {
        throw std::runtime_error{
                info_.joints[index].name + " must already be configured in mode 2"};
      }
      motors_[index].limits.position_max = static_cast<double>(
        RegisterReply{0U, 0U, 0U, read_register(index, Register::position_max)}.float_value());
      motors_[index].limits.velocity_max = static_cast<double>(
        RegisterReply{0U, 0U, 0U, read_register(index, Register::velocity_max)}.float_value());
      motors_[index].limits.torque_max = static_cast<double>(
        RegisterReply{0U, 0U, 0U, read_register(index, Register::torque_max)}.float_value());
      if (!motors_[index].limits.valid() ||
        motors_[index].velocity_limit > motors_[index].limits.velocity_max)
      {
        throw std::runtime_error{info_.joints[index].name + " has incompatible mapping limits"};
      }
      socket_.send(make_special_command(motors_[index].can_id, SpecialCommand::disable));
      if (!await_feedback(index)) {
        throw std::runtime_error{info_.joints[index].name + " did not return initial feedback"};
      }
      if (motors_[index].enabled != 0.0 || motors_[index].fault_code != 0.0 ||
        motors_[index].position < motors_[index].command_min ||
        motors_[index].position > motors_[index].command_max)
      {
        throw std::runtime_error{
                info_.joints[index].name + " is not disabled and fault-free inside joint limits"};
      }
      motors_[index].command = motors_[index].position;
    }
    hardware_active_ = false;
    command_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    send_disable();
    socket_.close();
    RCLCPP_ERROR(kLogger, "DaMiao configure failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn DamiaoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  send_disable();
  socket_.close();
  hardware_active_ = false;
  command_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!socket_.is_open()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hardware_active_ = true;
  command_active_ = false;
  last_disabled_poll_at_ = std::chrono::steady_clock::time_point{};
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_disable();
  hardware_active_ = false;
  command_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  send_disable();
  socket_.close();
  hardware_active_ = false;
  command_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystem::on_error(const rclcpp_lifecycle::State &)
{
  send_disable();
  socket_.close();
  hardware_active_ = false;
  command_active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DamiaoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(motors_.size() * kStateInterfaces.size());
  for (std::size_t index{0U}; index < motors_.size(); ++index) {
    const auto & name = info_.joints[index].name;
    auto & motor = motors_[index];
    interfaces.emplace_back(name, "position", &motor.position);
    interfaces.emplace_back(name, "velocity", &motor.velocity);
    interfaces.emplace_back(name, "effort", &motor.effort);
    interfaces.emplace_back(name, "fault_code", &motor.fault_code);
    interfaces.emplace_back(name, "mos_temperature", &motor.mos_temperature);
    interfaces.emplace_back(name, "motor_temperature", &motor.motor_temperature);
    interfaces.emplace_back(name, "feedback_age_ms", &motor.feedback_age_ms);
    interfaces.emplace_back(name, "enabled", &motor.enabled);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DamiaoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(motors_.size());
  for (std::size_t index{0U}; index < motors_.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name, "position", &motors_[index].command);
  }
  return interfaces;
}

hardware_interface::return_type DamiaoSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto starts = relevant_interface_count(start_interfaces);
  const auto stops = relevant_interface_count(stop_interfaces);
  if ((starts != 0U && !all_position_interfaces(start_interfaces)) ||
    (stops != 0U && !all_position_interfaces(stop_interfaces)) ||
    (starts != 0U && stops != 0U))
  {
    RCLCPP_ERROR(kLogger, "Both DaMiao position interfaces must start or stop together");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DamiaoSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  try {
    if (relevant_interface_count(stop_interfaces) != 0U) {
      send_disable();
      command_active_ = false;
    }
    if (relevant_interface_count(start_interfaces) != 0U) {
      if (!hardware_active_ || !socket_.is_open()) {
        return hardware_interface::return_type::ERROR;
      }
      for (const auto & motor : motors_) {
        const auto age_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - motor.received_at).count();
        if (motor.received_at.time_since_epoch().count() == 0 ||
          age_ms > static_cast<double>(feedback_timeout_ms_) ||
          motor.enabled != 0.0 || motor.fault_code != 0.0 ||
          !std::isfinite(motor.position) || motor.position < motor.command_min ||
          motor.position > motor.command_max)
        {
          RCLCPP_ERROR(kLogger, "Refusing to enable without fresh, safe position feedback");
          return hardware_interface::return_type::ERROR;
        }
      }
      for (auto & motor : motors_) {
        motor.command = motor.position;
        socket_.send(make_position_velocity_command(
            motor.can_id, static_cast<float>(motor.command),
            static_cast<float>(motor.velocity_limit)));
      }
      for (const auto & motor : motors_) {
        socket_.send(make_special_command(motor.can_id, SpecialCommand::enable));
      }
      enable_sent_at_ = std::chrono::steady_clock::now();
      command_active_ = true;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    send_disable();
    RCLCPP_ERROR(kLogger, "DaMiao command-mode switch failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type DamiaoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    CanFrame frame{};
    for (std::size_t count{0U}; count < max_rx_frames_per_cycle_ && socket_.receive(frame); ++count) {
      observe_frame(frame, std::chrono::steady_clock::now());
    }
    const auto now = std::chrono::steady_clock::now();
    for (auto & motor : motors_) {
      motor.feedback_age_ms = motor.received_at.time_since_epoch().count() == 0 ?
        std::numeric_limits<double>::infinity() :
        std::chrono::duration<double, std::milli>(now - motor.received_at).count();
      if (command_active_ &&
        (motor.feedback_age_ms > static_cast<double>(feedback_timeout_ms_) ||
        motor.fault_code != 0.0 ||
        (motor.received_at > enable_sent_at_ && motor.enabled == 0.0 &&
        std::chrono::duration<double, std::milli>(now - enable_sent_at_).count() >
        static_cast<double>(feedback_timeout_ms_))))
      {
        send_disable();
        command_active_ = false;
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    send_disable();
    command_active_ = false;
    RCLCPP_ERROR(kLogger, "DaMiao receive failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type DamiaoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!command_active_) {
    if (hardware_active_ && socket_.is_open()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_disabled_poll_at_ >=
        std::chrono::milliseconds{disabled_poll_interval_ms_})
      {
        try {
          for (const auto & motor : motors_) {
            socket_.send(make_special_command(motor.can_id, SpecialCommand::disable));
          }
          last_disabled_poll_at_ = now;
        } catch (const std::exception & error) {
          RCLCPP_ERROR(kLogger, "DaMiao disabled-state poll failed: %s", error.what());
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    return hardware_interface::return_type::OK;
  }
  try {
    for (const auto & motor : motors_) {
      if (!std::isfinite(motor.command) || motor.command < motor.command_min ||
        motor.command > motor.command_max)
      {
        send_disable();
        command_active_ = false;
        return hardware_interface::return_type::ERROR;
      }
    }
    for (const auto & motor : motors_) {
      socket_.send(make_position_velocity_command(
          motor.can_id, static_cast<float>(motor.command),
          static_cast<float>(motor.velocity_limit)));
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    send_disable();
    command_active_ = false;
    RCLCPP_ERROR(kLogger, "DaMiao transmit failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
}

std::uint32_t DamiaoSystem::read_register(std::size_t motor_index, Register register_id)
{
  const auto & motor = motors_.at(motor_index);
  socket_.send(make_register_read(motor.can_id, register_id));
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds{configure_timeout_ms_};
  while (socket_.wait_readable(remaining_milliseconds(deadline))) {
    CanFrame frame{};
    for (std::size_t count{0U}; count < max_rx_frames_per_cycle_ && socket_.receive(frame);
      ++count)
    {
      if (frame.id != motor.master_id) {
        continue;
      }
      if (frame.data[0] != static_cast<std::uint8_t>(motor.can_id & 0xFFU) ||
        frame.data[1] != static_cast<std::uint8_t>((motor.can_id >> 8U) & 0xFFU) ||
        frame.data[2] != 0x33U ||
        frame.data[3] != static_cast<std::uint8_t>(register_id))
      {
        continue;
      }
      const auto reply = decode_register_reply(frame);
      if (reply.motor_id == motor.can_id)
      {
        return reply.raw_value;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
  }
  throw std::runtime_error{"timed out reading DaMiao register"};
}

bool DamiaoSystem::await_feedback(std::size_t motor_index)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds{configure_timeout_ms_};
  const auto previous = motors_.at(motor_index).received_at;
  while (socket_.wait_readable(remaining_milliseconds(deadline))) {
    CanFrame frame{};
    for (std::size_t count{0U}; count < max_rx_frames_per_cycle_ && socket_.receive(frame);
      ++count)
    {
      observe_frame(frame, std::chrono::steady_clock::now());
      if (motors_[motor_index].received_at > previous) {
        return true;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
  }
  return false;
}

void DamiaoSystem::observe_frame(
  const CanFrame & frame, std::chrono::steady_clock::time_point received_at)
{
  for (auto & motor : motors_) {
    if (frame.id != motor.master_id) {
      continue;
    }
    if (frame.data[0] == static_cast<std::uint8_t>(motor.can_id & 0xFFU) &&
      frame.data[1] == static_cast<std::uint8_t>((motor.can_id >> 8U) & 0xFFU) &&
      (frame.data[2] == 0x33U || frame.data[2] == 0x55U || frame.data[2] == 0xAAU))
    {
      return;
    }
    if (!motor.limits.valid()) {
      return;
    }
    const auto feedback = decode_feedback(frame, motor.limits);
    if (feedback.motor_id != motor.can_id) {
      continue;
    }
    motor.position = feedback.position;
    motor.velocity = feedback.velocity;
    motor.effort = feedback.effort;
    motor.fault_code = feedback.status <= 1U ? 0.0 : static_cast<double>(feedback.status);
    motor.enabled = feedback.status == 1U ? 1.0 : 0.0;
    motor.mos_temperature = feedback.mos_temperature_c;
    motor.motor_temperature = feedback.motor_temperature_c;
    motor.received_at = received_at;
    return;
  }
}

void DamiaoSystem::send_disable() noexcept
{
  if (!socket_.is_open()) {
    return;
  }
  for (const auto & motor : motors_) {
    try {
      socket_.send(make_special_command(motor.can_id, SpecialCommand::disable));
    } catch (const std::exception &) {
      // Shutdown paths are best-effort and must continue closing the socket.
    }
  }
}

bool DamiaoSystem::all_position_interfaces(const std::vector<std::string> & interfaces) const
{
  return relevant_interface_count(interfaces) == motors_.size();
}

std::size_t DamiaoSystem::relevant_interface_count(
  const std::vector<std::string> & interfaces) const
{
  std::size_t count{0U};
  for (std::size_t index{0U}; index < motors_.size(); ++index) {
    const std::string expected = info_.joints[index].name + "/position";
    if (std::find(interfaces.begin(), interfaces.end(), expected) != interfaces.end()) {
      ++count;
    }
  }
  return count;
}

}  // namespace robot_hw_can

PLUGINLIB_EXPORT_CLASS(
  robot_hw_can::DamiaoSystem,
  hardware_interface::SystemInterface)
