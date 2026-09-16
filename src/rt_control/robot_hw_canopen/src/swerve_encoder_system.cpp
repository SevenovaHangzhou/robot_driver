#include "robot_hw_canopen/swerve_encoder_system.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <exception>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hw_canopen
{
namespace
{

const auto kLogger{rclcpp::get_logger("SwerveEncoderSystem")};
const std::set<std::string> kAxisParameterNames{
  "node_id",
  "counts_per_revolution",
  "ring_gear_teeth",
  "pinion_gear_teeth",
  "direction",
  "installation_offset_rad"};
const std::set<std::string> kStateInterfaceNames{"position", "feedback_age_ms"};
const std::array<const char *, 4U> kHardwareParameterNames{
  "bus_config", "master_config", "master_bin", "can_interface_name"};

[[nodiscard]] const std::string & required_parameter(
  const hardware_interface::ComponentInfo & sensor, const char * name)
{
  const auto found = sensor.parameters.find(name);
  if (found == sensor.parameters.end() || found->second.empty()) {
    throw std::invalid_argument{
            "swerve encoder " + sensor.name + " requires parameter " + name};
  }
  return found->second;
}

template<typename Integer>
[[nodiscard]] Integer parse_integer(
  const hardware_interface::ComponentInfo & sensor, const char * name)
{
  const std::string & text{required_parameter(sensor, name)};
  Integer result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw std::invalid_argument{
            "swerve encoder " + sensor.name + " parameter " + name +
            " must be an integer"};
  }
  return result;
}

[[nodiscard]] double parse_double(
  const hardware_interface::ComponentInfo & sensor, const char * name)
{
  const std::string & text{required_parameter(sensor, name)};
  std::size_t consumed{0U};
  const double result{std::stod(text, &consumed)};
  if (consumed != text.size() || !std::isfinite(result)) {
    throw std::invalid_argument{
            "swerve encoder " + sensor.name + " parameter " + name +
            " must be finite"};
  }
  return result;
}

void validate_hardware_parameters(const hardware_interface::HardwareInfo & info)
{
  for (const char * name : kHardwareParameterNames) {
    const auto found = info.hardware_parameters.find(name);
    if (found == info.hardware_parameters.end() || found->second.empty()) {
      throw std::invalid_argument{
              std::string{"swerve encoder system requires hardware parameter "} + name};
    }
  }
}

[[nodiscard]] SwerveEncoderAxisConfig axis_config(
  const hardware_interface::ComponentInfo & sensor)
{
  if (sensor.type != "sensor" || !sensor.command_interfaces.empty()) {
    throw std::invalid_argument{"swerve encoder resources must be read-only sensors"};
  }
  std::set<std::string> interface_names;
  for (const auto & interface : sensor.state_interfaces) {
    interface_names.insert(interface.name);
  }
  if (interface_names != kStateInterfaceNames ||
    sensor.state_interfaces.size() != kStateInterfaceNames.size())
  {
    throw std::invalid_argument{
            "swerve encoder sensors require position and feedback_age_ms states"};
  }
  std::set<std::string> parameter_names;
  for (const auto & item : sensor.parameters) {
    parameter_names.insert(item.first);
  }
  if (parameter_names != kAxisParameterNames) {
    throw std::invalid_argument{"swerve encoder sensor parameters do not match the contract"};
  }

  const auto node_id{parse_integer<unsigned int>(sensor, "node_id")};
  const auto counts{parse_integer<std::uint32_t>(sensor, "counts_per_revolution")};
  const auto ring_teeth{parse_integer<std::uint32_t>(sensor, "ring_gear_teeth")};
  const auto pinion_teeth{parse_integer<std::uint32_t>(sensor, "pinion_gear_teeth")};
  const auto direction{parse_integer<int>(sensor, "direction")};
  if (node_id > std::numeric_limits<std::uint8_t>::max()) {
    throw std::invalid_argument{"swerve encoder node ID exceeds uint8 range"};
  }
  return {
    static_cast<std::uint8_t>(node_id), counts, ring_teeth, pinion_teeth,
    direction, parse_double(sensor, "installation_offset_rad")};
}

}  // namespace

SwerveEncoderSystem::SwerveEncoderSystem()
{
  position_rad_.fill(std::numeric_limits<double>::quiet_NaN());
  feedback_age_ms_.fill(std::numeric_limits<double>::infinity());
}

SwerveEncoderSystem::~SwerveEncoderSystem() noexcept
{
  if (!clean()) {
    RCLCPP_FATAL(
      kLogger,
      "SwerveEncoderSystem could not quiesce CANopen callbacks before state destruction");
    std::terminate();
  }
}

hardware_interface::CallbackReturn SwerveEncoderSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  try {
    validate_hardware_parameters(info);
    if (!info.joints.empty() || !info.gpios.empty() ||
      info.sensors.size() != kSwerveEncoderCount)
    {
      throw std::invalid_argument{
              "swerve encoder system requires exactly four sensors and no joints or GPIO"};
    }
    std::array<SwerveEncoderAxisConfig, kSwerveEncoderCount> config{};
    for (std::size_t index{0U}; index < config.size(); ++index) {
      config[index] = axis_config(info.sensors[index]);
    }
    auto feedback = std::make_unique<SwerveEncoderFeedback>(config);
    if (canopen_ros2_control::CanopenSystem::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    feedback_ = std::move(feedback);
    position_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    feedback_age_ms_.fill(std::numeric_limits<double>::infinity());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    feedback_.reset();
    RCLCPP_ERROR(kLogger, "Swerve encoder configuration rejected: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface>
SwerveEncoderSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(2U * kSwerveEncoderCount);
  for (std::size_t index{0U}; index < kSwerveEncoderCount; ++index) {
    result.emplace_back(info_.sensors[index].name, "position", &position_rad_[index]);
    result.emplace_back(
      info_.sensors[index].name, "feedback_age_ms", &feedback_age_ms_[index]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface>
SwerveEncoderSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type SwerveEncoderSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!feedback_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto samples = feedback_->sample(now);
  for (std::size_t index{0U}; index < samples.size(); ++index) {
    position_rad_[index] = samples[index].position_rad;
    feedback_age_ms_[index] = samples[index].feedback_age_ms;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SwerveEncoderSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

void SwerveEncoderSystem::on_rpdo_received(
  ros2_canopen::COData data, std::uint8_t id,
  std::chrono::steady_clock::time_point received_at)
{
  if (!feedback_) {
    return;
  }
  const auto received_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    received_at.time_since_epoch()).count();
  static_cast<void>(feedback_->observe(
      id, data.index_, data.subindex_, data.data_, received_ns));
}

}  // namespace robot_hw_canopen

PLUGINLIB_EXPORT_CLASS(
  robot_hw_canopen::SwerveEncoderSystem,
  hardware_interface::SystemInterface)
