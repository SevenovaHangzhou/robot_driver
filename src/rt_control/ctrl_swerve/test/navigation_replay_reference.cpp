#include "navigation_replay.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "swerve_driver/control_core.hpp"

namespace nav_driver_model
{
// Test-only use of the reference's existing snapshot initialization seam. Restore
// every state field used by the current consumer without synthesizing branches,
// holding flags, or acceleration from measurements. No algorithm is reimplemented.
struct PredictionSeed
{
  static void apply(
    ControlCore & core, const ModuleFeedbackArray & feedback,
    const navigation_replay::State & state)
  {
    if (!core.activate(feedback)) {
      throw std::runtime_error("reference activation rejected replay feedback");
    }
    core.body_alignment_waiting_ = state.waiting;
    auto & steering = core.setpoints_;
    steering.gated_ = state.gated;
    steering.translation_reversed_.reset();
    if (state.branch >= 0) {
      steering.translation_reversed_ = state.branch != 0;
    }
    for (std::size_t i = 0; i < 4; ++i) {
      const auto & module = state.modules[i];
      core.holding_modules_[i] = module.holding;
      core.previous_speeds_[i] = module.wheel_reference;
      core.last_steering_targets_[i] = module.reference;
      steering.previous_targets_[i] = module.reference;
      steering.reversed_[i] = module.reversed;
      steering.steering_rates_[i] = module.rate;
      steering.steering_accelerations_[i] = module.acceleration;
      steering.selected_goals_[i] = module.goal;
      steering.selected_errors_[i] = module.error;
    }
    auto & body = core.body_shaper_;
    for (std::size_t i = 0; i < 3; ++i) {
      const double scale = i == 2 ? body.limits_.rotation_radius_m : 1.0;
      body.input_.current_velocity[i] = state.velocity[i] * scale;
      body.input_.current_acceleration[i] = state.acceleration[i] * scale;
    }
  }
};
}  // namespace nav_driver_model

namespace navigation_replay
{
State reference_step(
  const Config & c, const State & seed, const std::array<double, 3> & command, double dt,
  bool authorized, bool fresh)
{
  nav_driver_model::CoreConfig config;
  for (std::size_t i = 0; i < 4; ++i) {
    config.locations[i] = {c.x[i], c.y[i]};
  }
  config.wheel_radius = c.radius;
  config.steering_min = c.minimum;
  config.steering_max = c.maximum;
  config.steering_limit_margin = c.margin;
  config.steering_limit_tolerance = c.tolerance;
  config.max_linear_speed = c.linear_speed;
  config.max_angular_speed = c.rotation_speed;
  config.max_wheel_speed = c.wheel_speed;
  config.max_wheel_acceleration = c.wheel_acceleration;
  config.max_body_acceleration = c.linear_acceleration;
  config.max_body_jerk = c.body_jerk;
  config.max_angular_acceleration = c.rotation_acceleration;
  config.translating_max_angular_speed = c.angular_speed;
  config.translating_max_angular_acceleration = c.angular_acceleration;
  config.velocity_deadband = c.deadband;
  config.max_encoder_difference = c.encoder_difference;
  config.max_update_period = c.update_period;
  config.slip_residual_threshold = c.slip_threshold;
  config.encoder_fault_window = c.encoder_window;
  config.setpoint.alignment_threshold_rad = c.gate;
  config.setpoint.alignment_release_rad = c.release;
  config.setpoint.flip_hysteresis_rad = c.hysteresis;
  config.setpoint.maximum_steering_slew_radps = c.steering_speed;
  config.setpoint.maximum_steering_acceleration_radps2 = c.steering_acceleration;
  config.setpoint.maximum_steering_jerk_radps3 = c.steering_jerk;
  config.setpoint.translation_heading_epsilon_rad = c.heading_epsilon;
  config.setpoint.steering_angle_deadband_rad = c.angle_deadband;
  nav_driver_model::ModuleFeedbackArray feedback{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto & m = seed.modules[i];
    feedback[i] = {m.actual, m.encoder, m.wheel_position, m.wheel_actual / c.radius[i], true,
                   true,     true};
  }
  nav_driver_model::ControlCore core{config};
  nav_driver_model::PredictionSeed::apply(core, feedback, seed);
  core.set_terminal_rotation(authorized);
  core.update(feedback, {command[0], command[1], command[2]}, fresh, dt);
  using W = nav_driver_model::ExecutionStateWire;
  const auto wire = core.execution_state(feedback, 1.0, 1.0, {1, 2, 3, 4});
  State out;
  out.gated = wire[W::Gated] != 0.0;
  out.waiting = wire[W::Waiting] != 0.0;
  out.branch = static_cast<int>(wire[W::TranslationBranch]);
  out.status = static_cast<int>(wire[W::Status]);
  for (std::size_t i = 0; i < 3; ++i) {
    out.velocity[i] = wire[W::BodyVelocity + i];
    out.acceleration[i] = wire[W::BodyAcceleration + i];
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const auto k = W::Modules + i * W::Stride;
    auto & m = out.modules[i];
    m.actual = wire[k + W::Angle];
    m.encoder = feedback[i].steering_angle;
    m.wheel_actual = wire[k + W::WheelVelocity];
    m.wheel_position = feedback[i].wheel_position;
    m.reference = wire[k + W::ReferenceAngle];
    m.rate = wire[k + W::ReferenceRate];
    m.acceleration = wire[k + W::ReferenceAcceleration];
    m.wheel_reference = wire[k + W::ReferenceWheelVelocity];
    m.goal = wire[k + W::Goal];
    m.error = wire[k + W::Error];
    m.reversed = wire[k + W::Reversed] != 0.0;
    m.holding = wire[k + W::Holding] != 0.0;
  }
  return out;
}
}  // namespace navigation_replay
