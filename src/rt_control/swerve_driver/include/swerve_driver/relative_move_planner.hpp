#ifndef SWERVE_DRIVER__RELATIVE_MOVE_PLANNER_HPP_
#define SWERVE_DRIVER__RELATIVE_MOVE_PLANNER_HPP_

#include <array>
#include <limits>

#include "swerve_driver/swerve_odometry.hpp"

namespace swerve_driver
{
// Missing physical values deliberately fail validation, including geometry at zero.
inline constexpr double kRelativeMoveRequired = std::numeric_limits<double>::quiet_NaN();

struct RelativeMoveModuleConfig
{
  Translation2d location{kRelativeMoveRequired, kRelativeMoveRequired};  // base frame, m
  double wheel_radius_m{kRelativeMoveRequired};
  SteeringAngleLimits steering_limits{
    kRelativeMoveRequired, kRelativeMoveRequired,
    kRelativeMoveRequired, kRelativeMoveRequired};
  double max_wheel_velocity_radps{kRelativeMoveRequired};
  double max_wheel_acceleration_radps2{kRelativeMoveRequired};
};

struct RelativeMoveConfig
{
  std::array<RelativeMoveModuleConfig, kSwerveModuleCount> modules{};  // FL, FR, RL, RR
  double max_translation_velocity_mps{kRelativeMoveRequired};  // (0, 0.2]
  // Bounds the total base-origin acceleration, including the centripetal term.
  double max_translation_acceleration_mps2{kRelativeMoveRequired};
  double max_yaw_velocity_radps{kRelativeMoveRequired};
  double max_yaw_acceleration_radps2{kRelativeMoveRequired};
};

struct RelativeMoveModuleStart
{
  // Both steering readings use the same calibrated output-axis rad coordinate.
  double steering_motor_rad{kRelativeMoveRequired};
  double steering_measured_rad{kRelativeMoveRequired};
  double drive_position_rad{kRelativeMoveRequired};  // wheel-side motor-derived position
};
using RelativeMoveStart = std::array<RelativeMoveModuleStart, kSwerveModuleCount>;

struct RelativeMoveModulePlan
{
  double steering_start_rad{0.0};  // motor-derived start for a separate alignment ramp
  double steering_target_rad{0.0};  // absolute calibrated CSP coordinate, held to the end
  double steering_alignment_error_rad{0.0};  // target minus external measurement
  double drive_start_rad{0.0};
  double drive_travel_rad{0.0};  // signed travel for progress 0 -> 1
};

struct RelativeMovePlan
{
  Pose2d goal{};  // execution-start base frame: x/y in m, heading in rad
  BodyDelta body_log{};  // constant body twist ratios per unit normalized progress
  std::array<RelativeMoveModulePlan, kSwerveModuleCount> modules{};
  double progress_velocity_limit{0.0};  // 1/s (before triangular peak reduction)
  double progress_acceleration{0.0};  // 1/s^2, also used for cancel deceleration
  double peak_progress_velocity{0.0};
  double acceleration_duration_s{0.0};
  double cruise_duration_s{0.0};
  double duration_s{0.0};  // drive execution only; alignment is separate
};

enum class RelativeMoveError
{
  none, busy, invalid_goal, goal_out_of_range, invalid_config, invalid_start,
  steering_infeasible, numeric_range
};

enum class RelativeMoveState {unconfigured, running, stopping, complete, canceled};

struct RelativeMoveSample
{
  RelativeMoveState state{RelativeMoveState::unconfigured};
  double progress{0.0};
  double progress_velocity{0.0};
  double progress_acceleration{0.0};
  Pose2d planned_pose{};  // reference only, NOT a measured pose estimate
  ChassisSpeeds body_velocity{};
  std::array<double, kSwerveModuleCount> drive_position_rad{};
  std::array<double, kSwerveModuleCount> drive_velocity_radps{};
};

// Single-owner, fixed-size, allocation-free math; no ROS, device access or hardware FSM.
// configure() requires a stationary start supplied by the caller. Do not advance until
// stationary CSV->CSP confirmation and the separate steering alignment have completed.
// The caller owns IMU admission/loss, fault latching, feedback/settling tolerances, and
// runtime gates. 'complete' means only that the reference profile ended, NOT action success.
// cancel() is a reference deceleration, never proof of stopping with a failed drive/bus.
class RelativeMovePlanner final
{
public:
  // Failed configuration leaves the previous plan/sample unchanged; active replacement
  // is rejected. There is no queue or measured-position reset during motion.
  [[nodiscard]] RelativeMoveError configure(
    const Pose2d & goal, const RelativeMoveConfig & config,
    const RelativeMoveStart & start) noexcept;
  [[nodiscard]] const RelativeMovePlan & plan() const noexcept {return plan_;}
  [[nodiscard]] const RelativeMoveSample & sample() const noexcept {return sample_;}
  // Invalid/nonpositive dt returns false without changing state. Large finite dt is
  // sampled analytically through all phase boundaries. Timing watchdog is caller-owned.
  [[nodiscard]] bool advance(double dt_seconds) noexcept;
  void cancel() noexcept;  // idempotent; preserves position and velocity at the call

private:
  void update_targets() noexcept;
  RelativeMovePlan plan_{};
  RelativeMoveSample sample_{};
  double elapsed_s_{0.0};
  double cancel_start_progress_{0.0};
  double cancel_start_velocity_{0.0};
  double cancel_duration_s_{0.0};
  double cancel_end_progress_{0.0};
};
}  // namespace swerve_driver

#endif  // SWERVE_DRIVER__RELATIVE_MOVE_PLANNER_HPP_
