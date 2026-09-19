#include "robot_hw_can/parameter_tool.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include "robot_hw_can/socketcan.hpp"

namespace robot_hw_can
{
namespace
{

constexpr char kConfirmation[] = "WRITE_DAMIAO_PARAMETERS";
constexpr auto kTransactionTimeout = std::chrono::milliseconds{500};

template<typename Integer>
Integer parse_integer(const std::string & value, const char * field)
{
  Integer result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
    throw std::invalid_argument{std::string{field} + " must be an integer"};
  }
  return result;
}

double parse_double(const std::string & value, const char * field)
{
  std::size_t consumed{0U};
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument{std::string{field} + " must be finite"};
  }
  return result;
}

CanFrame wait_for_frame(
  SocketCan & socket, const std::function<bool(const CanFrame &)> & matches)
{
  const auto deadline = std::chrono::steady_clock::now() + kTransactionTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (!socket.wait_readable(static_cast<int>(std::max<std::int64_t>(1, remaining.count())))) {
      continue;
    }
    CanFrame frame{};
    while (socket.receive(frame)) {
      if (matches(frame)) {
        return frame;
      }
    }
  }
  throw std::runtime_error{"timed out waiting for DaMiao parameter response"};
}

std::uint32_t read_register(
  SocketCan & socket, const ParameterWriteRequest & request, Register register_id)
{
  socket.send(make_register_read(request.motor_id, register_id));
  const auto frame = wait_for_frame(socket, [&](const CanFrame & candidate) {
      return candidate.id == request.master_id &&
             candidate.data[0] == static_cast<std::uint8_t>(request.motor_id & 0xFFU) &&
             candidate.data[1] == static_cast<std::uint8_t>((request.motor_id >> 8U) & 0xFFU) &&
             candidate.data[2] == 0x33U &&
             candidate.data[3] == static_cast<std::uint8_t>(register_id);
    });
  return decode_register_reply(frame).raw_value;
}

void write_profile_value(
  SocketCan & socket, const ParameterWriteRequest & request,
  Register register_id, float value)
{
  socket.send(make_register_write_float(request.motor_id, register_id, value));
  const auto frame = wait_for_frame(socket, [&](const CanFrame & candidate) {
      return candidate.id == request.master_id &&
             candidate.data[0] == static_cast<std::uint8_t>(request.motor_id & 0xFFU) &&
             candidate.data[1] == static_cast<std::uint8_t>((request.motor_id >> 8U) & 0xFFU) &&
             candidate.data[2] == 0x55U &&
             candidate.data[3] == static_cast<std::uint8_t>(register_id);
    });
  const float echoed = decode_register_reply(frame).float_value();
  if (!std::isfinite(echoed) || std::abs(echoed - value) >
    std::max(1.0e-6F, std::abs(value) * 1.0e-4F))
  {
    throw std::runtime_error{"DaMiao parameter write echo does not match the request"};
  }
}

void confirm_disabled(SocketCan & socket, const ParameterWriteRequest & request)
{
  socket.send(make_special_command(request.motor_id, SpecialCommand::disable));
  const auto frame = wait_for_frame(socket, [&](const CanFrame & candidate) {
      return candidate.id == request.master_id &&
             candidate.data[2] != 0x33U && candidate.data[2] != 0x55U &&
             candidate.data[2] != 0xAAU &&
             (candidate.data[0] & 0x0FU) == static_cast<std::uint8_t>(request.motor_id);
    });
  if ((frame.data[0] >> 4U) > 1U) {
    throw std::runtime_error{"DaMiao motor reports a fault; refusing parameter write"};
  }
  if ((frame.data[0] >> 4U) == 1U) {
    throw std::runtime_error{"DaMiao motor remained enabled; refusing parameter write"};
  }
}

void save_parameters(SocketCan & socket, const ParameterWriteRequest & request)
{
  socket.send(make_parameter_save(request.motor_id));
  static_cast<void>(wait_for_frame(socket, [&](const CanFrame & candidate) {
      return candidate.id == request.master_id &&
             candidate.data[0] == static_cast<std::uint8_t>(request.motor_id & 0xFFU) &&
             candidate.data[1] == static_cast<std::uint8_t>((request.motor_id >> 8U) & 0xFFU) &&
             candidate.data[2] == 0xAAU && candidate.data[3] == 0x01U;
    }));
}

}  // namespace

ParameterWriteRequest parse_parameter_request(const std::vector<std::string> & arguments)
{
  ParameterWriteRequest request;
  std::string confirmation;
  bool has_interface{false};
  bool has_motor{false};
  bool has_master{false};
  bool has_acceleration{false};
  bool has_deceleration{false};
  bool has_maximum_speed{false};
  bool has_confirmation{false};
  for (std::size_t index{0U}; index < arguments.size();) {
    const std::string & option = arguments[index++];
    if (option == "--save") {
      if (request.save_to_flash) {
        throw std::invalid_argument{"--save may be specified only once"};
      }
      request.save_to_flash = true;
      continue;
    }
    if (index >= arguments.size()) {
      throw std::invalid_argument{option + " requires a value"};
    }
    const std::string & value = arguments[index++];
    if (option == "--can-interface" && !has_interface) {
      request.can_interface = value;
      has_interface = true;
    } else if (option == "--motor-id" && !has_motor) {
      request.motor_id = parse_integer<std::uint16_t>(value, "motor ID");
      has_motor = true;
    } else if (option == "--master-id" && !has_master) {
      request.master_id = parse_integer<std::uint16_t>(value, "Master ID");
      has_master = true;
    } else if (option == "--acceleration-krad-s2" && !has_acceleration) {
      request.profile.acceleration_krad_s2 = parse_double(value, "acceleration");
      has_acceleration = true;
    } else if (option == "--deceleration-krad-s2" && !has_deceleration) {
      request.profile.deceleration_krad_s2 = parse_double(value, "deceleration");
      has_deceleration = true;
    } else if (option == "--maximum-speed-rad-s" && !has_maximum_speed) {
      request.profile.maximum_speed_rad_s = parse_double(value, "maximum speed");
      has_maximum_speed = true;
    } else if (option == "--confirm" && !has_confirmation) {
      confirmation = value;
      has_confirmation = true;
    } else {
      throw std::invalid_argument{"unknown or duplicate argument: " + option};
    }
  }
  if (!has_interface || !has_motor || !has_master || !has_acceleration ||
    !has_deceleration || !has_maximum_speed || !has_confirmation)
  {
    throw std::invalid_argument{"all DaMiao parameter-write arguments are required"};
  }
  if (request.can_interface.empty() || request.can_interface.size() > 15U ||
    request.motor_id == 0U || request.motor_id > 15U || request.master_id > 0x7FFU ||
    !request.profile.valid() || confirmation != kConfirmation)
  {
    throw std::invalid_argument{"DaMiao parameter-write request failed safety validation"};
  }
  return request;
}

void execute_parameter_write(const ParameterWriteRequest & request)
{
  if (request.can_interface.empty() || request.motor_id == 0U ||
    request.motor_id > 15U || request.master_id > 0x7FFU || !request.profile.valid())
  {
    throw std::invalid_argument{"invalid DaMiao parameter-write request"};
  }
  SocketCan socket;
  socket.open(request.can_interface, {request.master_id});
  try {
    const auto mode = read_register(socket, request, Register::control_mode);
    if (mode != kPositionVelocityMode) {
      throw std::runtime_error{"DaMiao motor is not in position-velocity mode 2"};
    }
    confirm_disabled(socket, request);
    write_profile_value(
      socket, request, Register::acceleration,
      static_cast<float>(request.profile.acceleration_krad_s2));
    write_profile_value(
      socket, request, Register::deceleration,
      static_cast<float>(request.profile.deceleration_krad_s2));
    write_profile_value(
      socket, request, Register::maximum_speed,
      static_cast<float>(request.profile.maximum_speed_rad_s));
    const TrapezoidalProfile readback{
      static_cast<double>(RegisterReply{
        request.motor_id, 0x33U, static_cast<std::uint8_t>(Register::acceleration),
        read_register(socket, request, Register::acceleration)}.float_value()),
      static_cast<double>(RegisterReply{
        request.motor_id, 0x33U, static_cast<std::uint8_t>(Register::deceleration),
        read_register(socket, request, Register::deceleration)}.float_value()),
      static_cast<double>(RegisterReply{
        request.motor_id, 0x33U, static_cast<std::uint8_t>(Register::maximum_speed),
        read_register(socket, request, Register::maximum_speed)}.float_value())};
    validate_trapezoidal_profile(readback, request.profile);
    if (request.save_to_flash) {
      confirm_disabled(socket, request);
      save_parameters(socket, request);
    }
    confirm_disabled(socket, request);
  } catch (...) {
    try {
      socket.send(make_special_command(request.motor_id, SpecialCommand::disable));
    } catch (const std::exception &) {
    }
    throw;
  }
}

}  // namespace robot_hw_can
