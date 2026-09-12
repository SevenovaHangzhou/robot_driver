#include "dm_swerve_driver/kinco_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "dm_swerve_driver/kinco_units.hpp"

namespace dm_swerve_driver {
namespace {

[[nodiscard]] double direction(bool inverted) noexcept
{
  return inverted ? -1.0 : 1.0;
}

[[nodiscard]] KincoOperationMode mode_for_axis(std::size_t index) noexcept
{
  return index < kSwerveModuleCount ?
         KincoOperationMode::cyclic_synchronous_position :
         KincoOperationMode::cyclic_synchronous_velocity;
}

[[nodiscard]] bool all_positive(
  const std::array<std::uint32_t, kSwerveModuleCount> & values) noexcept
{
  return std::all_of(values.begin(), values.end(), [](std::uint32_t value) {
      return value > 0U;
    });
}

[[nodiscard]] bool all_finite(
  const std::array<double, kSwerveModuleCount> & values) noexcept
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

[[nodiscard]] std::uint32_t signed_raw(std::int16_t value) noexcept
{
  return static_cast<std::uint16_t>(value);
}

[[nodiscard]] bool axis_feedback_healthy(
  const KincoAxisFeedback & feedback,
  KincoOperationMode expected_mode,
  bool require_enabled) noexcept
{
  if (!feedback.online || feedback.error_word != 0U || feedback.extended_error_word != 0U) {
    return false;
  }
  const Ds402State state{decode_ds402_state(feedback.status_word)};
  if (state == Ds402State::fault || state == Ds402State::fault_reaction_active ||
    state == Ds402State::unknown)
  {
    return false;
  }
  return !require_enabled ||
         (state == Ds402State::operation_enabled &&
         feedback.mode_display == static_cast<std::int8_t>(expected_mode));
}

}  // namespace

bool EthercatDomainStatus::healthy() const noexcept
{
  return link_up && all_slaves_operational && expected_working_counter > 0U &&
         working_counter == expected_working_counter;
}

void validate_kinco_hardware_config(const KincoSwerveHardwareConfig & config)
{
  const bool geometry_valid = std::isfinite(config.steering_gear_ratio) &&
    config.steering_gear_ratio > 0.0 && std::isfinite(config.drive_gear_ratio) &&
    config.drive_gear_ratio > 0.0 && std::isfinite(config.wheel_radius_m) &&
    config.wheel_radius_m > 0.0 && all_finite(config.steering_zero_offset_rad) &&
    valid_steering_angle_limits(config.steering_limits);
  const bool resolutions_valid = all_positive(config.steering_encoder_resolution) &&
    all_positive(config.drive_encoder_resolution);
  const bool feedforward_valid =
    (!config.position_velocity_feedforward_raw.has_value() ||
    *config.position_velocity_feedforward_raw <= 256U) &&
    (!config.position_acceleration_feedforward.has_value() ||
    (*config.position_acceleration_feedforward >= 10U &&
    *config.position_acceleration_feedforward <= 32767U));
  if (!geometry_valid || !resolutions_valid || !feedforward_valid ||
    config.startup_cycle_limit == 0U)
  {
    throw std::invalid_argument{"invalid Kinco swerve hardware configuration"};
  }
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    for (const auto endpoint : {config.steering_limits.minimum_rad + config.steering_limits.margin_rad,
      config.steering_limits.maximum_rad - config.steering_limits.margin_rad})
    {
      static_cast<void>(kinco_position_from_radians(
        (endpoint + config.steering_zero_offset_rad[i]) * config.steering_gear_ratio *
        direction(config.steering_inverted[i]), config.steering_encoder_resolution[i]));
    }
  }
}

KincoSwerveHardware::KincoSwerveHardware(
  KincoSwerveHardwareConfig config,
  std::unique_ptr<KincoEthercatBus> bus)
: config_{std::move(config)}, bus_{std::move(bus)}
{
  validate_kinco_hardware_config(config_);
  if (bus_ == nullptr) {
    throw std::invalid_argument{"Kinco swerve hardware requires an EtherCAT bus"};
  }
}

KincoSwerveHardware::~KincoSwerveHardware() noexcept
{
  stop();
}

KincoCommandBatch KincoSwerveHardware::disabled_commands() const noexcept
{
  KincoCommandBatch commands{};
  for (std::size_t index{0U}; index < commands.size(); ++index) {
    commands[index].mode = mode_for_axis(index);
  }
  return commands;
}

std::vector<KincoSdoWrite> KincoSwerveHardware::preop_writes() const
{
  std::vector<KincoSdoWrite> writes;
  writes.reserve(16U);
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    if (config_.position_velocity_feedforward_raw.has_value()) {
      writes.push_back(KincoSdoWrite{
          index, 0x60FBU, 2U, 16U, *config_.position_velocity_feedforward_raw});
    }
    if (config_.position_acceleration_feedforward.has_value()) {
      writes.push_back(KincoSdoWrite{
          index, 0x60FBU, 3U, 16U, *config_.position_acceleration_feedforward});
    }
  }
  if (config_.fault_reaction_option_code.has_value()) {
    for (std::size_t index{0U}; index < kKincoAxisCount; ++index) {
      writes.push_back(KincoSdoWrite{
          index, 0x605EU, 0U, 16U, signed_raw(*config_.fault_reaction_option_code)});
    }
  }
  return writes;
}

bool KincoSwerveHardware::startup_cycle_healthy(
  const KincoEthercatCycle & cycle) const noexcept
{
  if (!cycle.domain.healthy()) {
    return false;
  }
  for (std::size_t index{0U}; index < cycle.feedback.size(); ++index) {
    if (!axis_feedback_healthy(cycle.feedback[index], mode_for_axis(index), false)) {
      return false;
    }
  }
  return true;
}

bool KincoSwerveHardware::all_axes_enabled(
  const KincoEthercatCycle & cycle) const noexcept
{
  if (!cycle.domain.healthy()) {
    return false;
  }
  for (std::size_t index{0U}; index < cycle.feedback.size(); ++index) {
    if (!axis_feedback_healthy(cycle.feedback[index], mode_for_axis(index), true)) {
      return false;
    }
  }
  return true;
}

bool KincoSwerveHardware::initialize()
{
  return prepare() && enable();
}

bool KincoSwerveHardware::prepare(bool validate_position)
{
  if (prepared_) {
    return true;
  }
  try {
    bus_->open();
    bus_->configure_preop(preop_writes());
    bus_->activate();
    KincoCommandBatch commands{disabled_commands()};
    for (std::size_t attempt{0U}; attempt < config_.startup_cycle_limit; ++attempt) {
      last_cycle_ = bus_->exchange(commands);
      if (startup_cycle_healthy(last_cycle_)) {
        const auto measured = feedback();
        for (std::size_t i{0U}; i < measured.modules.size(); ++i) {
          const auto & module = measured.modules[i];
          if (validate_position && !steering_measurement_within_tolerance(
              module.motor_steering_angle_rad, config_.steering_limits))
          {
            throw std::out_of_range{"Kinco startup steering angle exceeds mechanical limits"};
          }
          const double safe = clamp_steering_measurement_to_safe_range(
            module.motor_steering_angle_rad, config_.steering_limits);
          safe_hold_positions_[i] = kinco_position_from_radians(
            (safe + config_.steering_zero_offset_rad[i]) * config_.steering_gear_ratio *
            direction(config_.steering_inverted[i]), config_.steering_encoder_resolution[i]);
        }
        prepared_ = true;
        return true;
      }
    }
  } catch (...) {
    close_bus();
    throw;
  }
  close_bus();
  return false;
}

bool KincoSwerveHardware::enable()
{
  if (initialized_) {return true;}
  if (!prepared_) {throw std::logic_error{"prepare Kinco hardware before enable"};}
  try {
    auto commands = disabled_commands();
    for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
      commands[index].target_position = safe_hold_positions_[index];
    }
    for (std::size_t attempt{0U}; attempt < config_.startup_cycle_limit; ++attempt) {
      for (std::size_t index{0U}; index < commands.size(); ++index) {
        commands[index].control_word = ds402_enable_control_word(
          decode_ds402_state(last_cycle_.feedback[index].status_word));
      }
      last_cycle_ = bus_->exchange(commands);
      if (!startup_cycle_healthy(last_cycle_)) {
        close_bus();
        return false;
      }
      if (all_axes_enabled(last_cycle_)) {
        initialized_ = true;
        return true;
      }
    }
  } catch (...) {
    close_bus();
    throw;
  }
  close_bus();
  return false;
}

KincoHardwareCycle KincoSwerveHardware::exchange(
  const std::array<KincoModuleTarget, kSwerveModuleCount> & targets)
{
  std::array<std::uint16_t, kKincoAxisCount> control_words{};
  control_words.fill(0x000FU);
  return exchange(targets, control_words);
}

KincoHardwareCycle KincoSwerveHardware::exchange(
  const std::array<KincoModuleTarget, kSwerveModuleCount> & targets,
  const std::array<std::uint16_t, kKincoAxisCount> & control_words)
{
  if (!initialized_) {
    throw std::logic_error{"Kinco swerve hardware is not initialized"};
  }
  KincoCommandBatch commands{};
  for (std::size_t index{0U}; index < targets.size(); ++index) {
    const auto & target = targets[index];
    if (!std::isfinite(target.steering_angle_rad) || !std::isfinite(target.wheel_speed_mps)) {
      throw std::invalid_argument{"Kinco module targets must be finite"};
    }
    if (!steering_angle_within_limits(target.steering_angle_rad, config_.steering_limits)) {
      throw std::out_of_range{"Kinco steering target exceeds mechanical limits"};
    }
    const double steering_motor_angle =
      (target.steering_angle_rad + config_.steering_zero_offset_rad[index]) *
      config_.steering_gear_ratio * direction(config_.steering_inverted[index]);
    auto & steering = commands[index];
    steering.control_word = control_words[index];
    steering.mode = KincoOperationMode::cyclic_synchronous_position;
    steering.target_position = kinco_position_from_radians(
      steering_motor_angle, config_.steering_encoder_resolution[index]);
    safe_hold_positions_[index] = steering.target_position;
    steering.target_velocity = 0;

    const double wheel_speed{target.drive_enabled ? target.wheel_speed_mps : 0.0};
    const double drive_motor_velocity = wheel_speed / config_.wheel_radius_m *
      config_.drive_gear_ratio * direction(config_.drive_inverted[index]);
    auto & drive = commands[index + kSwerveModuleCount];
    drive.control_word = control_words[index + kSwerveModuleCount];
    drive.mode = KincoOperationMode::cyclic_synchronous_velocity;
    drive.target_velocity = kinco_velocity_from_radians_per_second(
      drive_motor_velocity, config_.drive_encoder_resolution[index]);
  }

  last_cycle_ = bus_->exchange(commands);
  return feedback();
}

KincoHardwareCycle KincoSwerveHardware::feedback() const
{
  KincoHardwareCycle result;
  result.raw = last_cycle_;
  result.valid = last_cycle_.domain.healthy();
  for (std::size_t index{0U}; index < kKincoAxisCount; ++index) {
    const auto & feedback = last_cycle_.feedback[index];
    result.axis_faults[index] = classify_kinco_error_word(feedback.error_word);
    if (feedback.extended_error_word != 0U) {
      result.axis_faults[index].disposition = FaultDisposition::latch;
      result.axis_faults[index].internal_or_configuration = true;
    }
    result.valid = result.valid && axis_feedback_healthy(
      feedback, mode_for_axis(index), true);
  }
  for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
    const auto & steering = last_cycle_.feedback[index];
    const auto & drive = last_cycle_.feedback[index + kSwerveModuleCount];
    const double steering_motor_angle = kinco_position_to_radians(
      steering.actual_position, config_.steering_encoder_resolution[index]);
    const double drive_motor_angle = kinco_position_to_radians(
      drive.actual_position, config_.drive_encoder_resolution[index]);
    const double drive_motor_velocity = kinco_velocity_to_radians_per_second(
      drive.actual_velocity, config_.drive_encoder_resolution[index]);
    result.modules[index] = KincoModuleFeedback{
      steering_motor_angle / (config_.steering_gear_ratio *
      direction(config_.steering_inverted[index])) -
      config_.steering_zero_offset_rad[index],
      drive_motor_angle * direction(config_.drive_inverted[index]) /
      config_.drive_gear_ratio * config_.wheel_radius_m,
      drive_motor_velocity * direction(config_.drive_inverted[index]) /
      config_.drive_gear_ratio * config_.wheel_radius_m};
  }
  return result;
}

void KincoSwerveHardware::stop() noexcept
{
  if (bus_ == nullptr || !bus_->is_open()) {
    initialized_ = false;
    return;
  }
  try {
    KincoCommandBatch commands{disabled_commands()};
    for (std::size_t index{0U}; index < kSwerveModuleCount; ++index) {
      commands[index].target_position = safe_hold_positions_[index];
    }
    if (initialized_) {
      auto zero = commands;
      for (std::size_t i{0U}; i < zero.size(); ++i) {
        if (decode_ds402_state(last_cycle_.feedback[i].status_word) ==
          Ds402State::operation_enabled) {zero[i].control_word = 0x000FU;}
      }
      static_cast<void>(bus_->exchange(zero));
      static_cast<void>(bus_->exchange(zero));
    }
    static_cast<void>(bus_->exchange(commands));
  } catch (...) {
  }
  close_bus();
}

bool KincoSwerveHardware::initialized() const noexcept
{
  return initialized_;
}

void KincoSwerveHardware::close_bus() noexcept
{
  initialized_ = false;
  prepared_ = false;
  bus_->deactivate();
  bus_->close();
}

}  // namespace dm_swerve_driver
