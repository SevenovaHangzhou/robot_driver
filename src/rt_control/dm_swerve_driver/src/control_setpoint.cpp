#include "control_loop_impl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace dm_swerve_driver {
namespace {

struct SteeringMeasurements {
  std::array<double, kSwerveModuleCount> planning_angles{};
  bool physical_limit_violation{false};
};

[[nodiscard]] ChassisSpeeds limit_chassis_command(
  ChassisSpeeds command, const ChassisParameters & limits)
{
  const double linear_speed{std::hypot(command.vx_mps, command.vy_mps)};
  if (linear_speed > limits.max_linear_speed_mps) {
    const double scale{limits.max_linear_speed_mps / linear_speed};
    command.vx_mps *= scale;
    command.vy_mps *= scale;
  }
  command.omega_radps = std::clamp(
    command.omega_radps,
    -limits.max_angular_speed_radps,
    limits.max_angular_speed_radps);
  return command;
}

[[nodiscard]] SteeringAngleLimits steering_limits(
  const DriverParameters & parameters) noexcept
{
  return SteeringAngleLimits{
    parameters.steering.joint_limit_min_rad,
    parameters.steering.joint_limit_max_rad,
    parameters.steering.joint_limit_margin_rad,
    parameters.steering.joint_limit_tolerance_rad};
}

[[nodiscard]] SteeringMeasurements normalize_steering_measurements(
  const std::array<double, kSwerveModuleCount> & measured,
  const SteeringAngleLimits & limits)
{
  SteeringMeasurements result;
  for (std::size_t index{0U}; index < measured.size(); ++index) {
    result.physical_limit_violation = result.physical_limit_violation ||
      !steering_measurement_within_tolerance(measured[index], limits);
    result.planning_angles[index] = clamp_steering_measurement_to_safe_range(
      measured[index], limits);
  }
  return result;
}

[[nodiscard]] std::array<SwerveModuleState, kSwerveModuleCount> desired_states(
  const ChassisSpeeds & command,
  const std::array<double, kSwerveModuleCount> & angles,
  bool hold_steering,
  bool return_to_zero,
  const DriverParameters & parameters)
{
  std::array<SwerveModuleState, kSwerveModuleCount> desired{};
  if (hold_steering) {
    for (std::size_t index{0U}; index < desired.size(); ++index) {
      desired[index] = SwerveModuleState{0.0, angles[index]};
    }
    return desired;
  }
  auto previous_angles = angles;
  if (return_to_zero) {
    previous_angles.fill(0.0);
  }
  return inverse_kinematics(
    command, module_locations(parameters), previous_angles,
    parameters.chassis.velocity_deadband_mps);
}

}  // namespace

double ControlLoop::Impl::measured_cycle_period(SteadyClock::time_point now)
{
  const double nominal{1.0 / parameters_.control.rate_hz};
  double measured{nominal};
  if (last_cycle_time_.has_value() && now > *last_cycle_time_) {
    measured = std::chrono::duration<double>{now - *last_cycle_time_}.count();
  }
  last_cycle_time_ = now;
  return std::clamp(measured, 0.5 * nominal, 2.0 * nominal);
}

double ControlLoop::Impl::wheel_speed_cap() const noexcept
{
  double cap{parameters_.chassis.max_wheel_speed_mps};
  for (const auto & module : modules_) {
    const double motor_cap = module.drive_motor().limits().velocity_max *
      parameters_.chassis.wheel_radius_m / parameters_.drive.gear_ratio;
    cap = std::min(cap, motor_cap);
  }
  return cap;
}

CyclePlan ControlLoop::Impl::prepare_cycle(SteadyClock::time_point now)
{
  CyclePlan plan;
  plan.dt_seconds = measured_cycle_period(now);
  plan.mailbox = mailbox_snapshot();
  const auto command_timestamp = plan.mailbox.command.valid ?
    std::optional<SteadyClock::time_point>{plan.mailbox.command.timestamp} : std::nullopt;
  plan.command = safety_.command_for_cycle(
    plan.mailbox.command.value, command_timestamp, now);
  const ChassisSpeeds limited{limit_chassis_command(
      plan.command.command, parameters_.chassis)};
  plan.discrete_command = discretize(limited, plan.dt_seconds);

  const auto measurements = normalize_steering_measurements(
    current_angles(), steering_limits(parameters_));
  if (safety_.observe_steering_limit_violation(
      measurements.physical_limit_violation))
  {
    log(
      measurements.physical_limit_violation ? DriverLogLevel::error : DriverLogLevel::warning,
      measurements.physical_limit_violation ?
      "steering measurement exceeded physical limit tolerance; fault latched" :
      "steering measurements returned within physical limit tolerance; manual clear required");
  }

  const auto & angles = measurements.planning_angles;
  const bool safety_faulted{safety_.faulted()};
  const bool timeout_holds_steering = plan.command.timed_out &&
    parameters_.control.hold_steer_on_timeout;
  const bool hold_steering = safety_faulted || timeout_holds_steering;
  if (hold_steering) {
    setpoint_generator_.reset();
  }
  auto desired = desired_states(
    plan.discrete_command, angles, hold_steering,
    plan.command.timed_out && !parameters_.control.hold_steer_on_timeout,
    parameters_);
  desaturate_wheel_speeds(desired, wheel_speed_cap());
  plan.alignment = setpoint_generator_.generate(desired, angles, plan.dt_seconds);
  plan.hold_steering = hold_steering;
  plan.drive_gated = plan.alignment.gated || safety_faulted;
  return plan;
}

}  // namespace dm_swerve_driver
