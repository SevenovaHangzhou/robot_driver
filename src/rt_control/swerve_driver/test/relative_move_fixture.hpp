#ifndef SWERVE_DRIVER_TEST__RELATIVE_MOVE_FIXTURE_HPP_
#define SWERVE_DRIVER_TEST__RELATIVE_MOVE_FIXTURE_HPP_
#include "swerve_driver/relative_move_session.hpp"

namespace swerve_driver
{
// Synthetic values exclusively for offline tests; no production calibration.
inline MoveSessionConfig session_config()
{
  MoveSessionConfig c;
  c.motion.max_translation_velocity_mps = 0.2;
  c.motion.max_translation_acceleration_mps2 = 0.3;
  c.motion.max_yaw_velocity_radps = 0.4;
  c.motion.max_yaw_acceleration_radps2 = 0.5;
  const std::array<Translation2d, 4> locations{{{0.4, 0.3}, {0.4, -0.3}, {-0.4, 0.3}, {-0.4, -0.3}}};
  for (std::size_t i = 0; i < 4; ++i) {
    c.motion.modules[i] = {locations[i], 0.1, {-kPi, kPi, 0.05, 0.01}, 3, 5};
  }
  c.max_steering_velocity = 2; c.max_steering_acceleration = 4;
  c.feedback_timeout = 0.1; c.imu_timeout = 0.1; c.imu_max_increment = 0.2;
  c.max_update_period = 0.1; c.stationary_wheel_velocity = 0.01;
  c.stationary_steering_velocity = 0.01; c.stationary_dwell = 0.02;
  c.alignment_tolerance = 0.02; c.alignment_dwell = 0.02; c.alignment_timeout = 5;
  c.steering_error = 0.15; c.steering_error_dwell = 0.02; c.encoder_difference = 0.1;
  c.position_tolerance = 0.001; c.velocity_tolerance = 0.01;
  c.settling_dwell = 0.03; c.settling_timeout = 1;
  c.slip_threshold = 0.05; c.slip_dwell = 0.02;
  c.pose_translation_tolerance = 0.01; c.pose_yaw_tolerance = 0.01; c.yaw_discrepancy = 0.02;
  c.switch_timeout = 1; c.stop_timeout = 3; c.max_goal_duration = 30;
  return c;
}
inline MoveRequest request(Pose2d goal = {0.5, 0, 0})
{
  return {goal, session_config().motion, 20};
}
struct SessionRig
{
  MoveSessionConfig config{session_config()};
  RelativeMoveSession session;
  MoveFeedback feedback;
  SessionRig()
  {
    (void)session.configure(config);
    for (auto & p : feedback.positions) {p = {0, 0, 0};}
    feedback.age = feedback.imu_age = feedback.imu_yaw = 0;
    feedback.imu_valid = feedback.bus_ok = feedback.drives_ok = true;
    feedback.steering_mode.fill(8); feedback.drive_mode.fill(9);
    for (int i = 0; i < 10; ++i) {session.update(0.01, feedback);}
  }
  void operation()
  {
    (void)session.set_mode(ChassisMode::operation, true);
    feedback.drive_mode.fill(8);
    for (int i = 0; i < 10; ++i) {session.update(0.01, feedback);}
  }
  void tick(double dt = 0.01, double yaw_bias_rate = 0)
  {
    const auto out = session.output();
    std::array<SwerveModulePosition, 4> previous{}, next{};
    std::array<Translation2d, 4> locations{};
    for (std::size_t i = 0; i < 4; ++i) {
      locations[i] = config.motion.modules[i].location;
      previous[i] = {feedback.positions[i].drive_position_rad * 0.1,
        feedback.positions[i].steering_measured_rad, true};
      if (!out.inhibited) {
        feedback.positions[i] = {out.steering_position[i], out.steering_position[i], out.wheel_position[i]};
        feedback.wheel_velocity[i] = out.wheel_velocity[i];
        feedback.steering_velocity[i] = out.steering_velocity[i];
      }
      next[i] = {feedback.positions[i].drive_position_rad * 0.1,
        feedback.positions[i].steering_measured_rad, true};
    }
    const auto delta = wheel_chassis_delta_from_position_deltas(previous, next, locations);
    if (delta) {feedback.imu_yaw += delta->dtheta_rad + yaw_bias_rate * dt;}
    session.update(dt, feedback);
  }
  void finish()
  {
    for (int i = 0; i < 4000 && session.state().active; ++i) {tick();}
  }
};
}
#endif
