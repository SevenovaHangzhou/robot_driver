#ifndef CTRL_SWERVE_TEST_NAVIGATION_REPLAY_HPP_
#define CTRL_SWERVE_TEST_NAVIGATION_REPLAY_HPP_

#include <array>

// Keep the two core implementations in separate translation units: their upstream
// headers intentionally retain identical include guards. No vendor source is copied.
namespace navigation_replay
{
struct Config
{
  std::array<double, 4> x{}, y{}, radius{}, minimum{}, maximum{}, margin{}, tolerance{};
  double linear_speed{}, angular_speed{}, rotation_speed{}, wheel_speed{}, wheel_acceleration{};
  double linear_acceleration{}, angular_acceleration{}, rotation_acceleration{}, body_jerk{};
  double deadband{}, encoder_difference{}, update_period{}, slip_threshold{}, encoder_window{};
  double gate{}, release{}, hysteresis{}, steering_speed{}, steering_acceleration{},
    steering_jerk{};
  double heading_epsilon{}, angle_deadband{};
};
struct Module
{
  double actual{}, encoder{}, wheel_actual{}, wheel_position{};
  double reference{}, rate{}, acceleration{}, wheel_reference{}, goal{}, error{};
  bool reversed{}, holding{};
};
struct State
{
  std::array<double, 3> velocity{}, acceleration{};
  std::array<Module, 4> modules{};
  bool gated{}, waiting{};
  int branch{-1}, status{};
};

// Execute the supplied unmodified production core from exactly the exported state.
// Authorization is supplied by the driver's real admission decision, not inferred
// from the command. The reference does not own a request lease or stopped admission.
State reference_step(
  const Config & config, const State & seed, const std::array<double, 3> & command, double dt,
  bool authorized, bool fresh = true);
}  // namespace navigation_replay
#endif
