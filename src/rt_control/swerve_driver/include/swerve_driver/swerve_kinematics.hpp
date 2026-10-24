#ifndef SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_
#define SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_

#include <array>
#include <limits>
#include <cstddef>
#include <optional>

namespace swerve_driver {

inline constexpr std::size_t kSwerveModuleCount{4U};
inline constexpr double kPi{3.141592653589793238462643383279502884};

struct ChassisSpeeds {
  double vx_mps{0.0};
  double vy_mps{0.0};
  double omega_radps{0.0};
};

struct Translation2d {
  double x{0.0};
  double y{0.0};
};

struct ChassisDelta {
  double dx_m{0.0};
  double dy_m{0.0};
  double dtheta_rad{0.0};
};

struct SwerveModuleState {
  double speed_mps{0.0};
  double angle_rad{0.0};
};

struct SwerveModuleMeasurement {
  double speed_mps{0.0};
  double angle_rad{0.0};
  bool valid{true};
};

struct SwerveModulePosition {
  double distance_m{0.0};
  double angle_rad{0.0};
  bool valid{true};
  bool rejected_as_slip{false};
};

struct ChassisSpeedFit {
  ChassisSpeeds speeds{};
  std::array<double, kSwerveModuleCount> residual_mps{};
  std::array<bool, kSwerveModuleCount> used_modules{};
  std::array<bool, kSwerveModuleCount> slipping_modules{};
  std::size_t used_module_count{0U};

  [[nodiscard]] bool slip_detected() const noexcept;
};

struct SteeringAngleLimits {
  double minimum_rad{-kPi};
  double maximum_rad{kPi};
  double margin_rad{0.0};
  double measurement_tolerance_rad{0.0};
};

struct OptimizedModuleState {
  double speed_mps{0.0};
  double target_angle_rad{0.0};
  double error_rad{0.0};
  bool reversed{false};
};

struct AlignmentResult {
  std::array<OptimizedModuleState, kSwerveModuleCount> modules{};
  double maximum_error_rad{0.0};
  bool gated{false};
};

struct SwerveSetpointParameters {
  double alignment_threshold_rad{0.349};
  double flip_hysteresis_rad{0.1};
  double maximum_steering_slew_radps{3.0};
  double translation_heading_epsilon_rad{1e-6};
  double steering_angle_deadband_rad{0.0};
  std::array<SteeringAngleLimits, kSwerveModuleCount> angle_limits{};
  // Infinite keeps the legacy velocity-step profile; finite values ramp steering rate.
  double maximum_steering_acceleration_radps2{std::numeric_limits<double>::infinity()};
  // Infinite keeps the trapezoidal profile; finite values limit the change of steering
  // acceleration (S-curve), which removes the acceleration steps that hit gear backlash.
  double maximum_steering_jerk_radps3{std::numeric_limits<double>::infinity()};
  // Gate hysteresis (last member: keeps existing aggregate initialisers valid). Once gated,
  // wheels stay stopped until every module error is at most this value. NaN releases at
  // alignment_threshold_rad (no hysteresis, legacy behaviour).
  double alignment_release_rad{std::numeric_limits<double>::quiet_NaN()};
};

class SwerveSetpointGenerator final {
public:
  explicit SwerveSetpointGenerator(const SwerveSetpointParameters & parameters);

  [[nodiscard]] AlignmentResult generate(
    const std::array<SwerveModuleState, kSwerveModuleCount> & desired,
    const std::array<double, kSwerveModuleCount> & measured_angles_rad, double dt_seconds,
    const std::array<bool, kSwerveModuleCount> & hold_modules = {});
  void reset() noexcept;
  void reset(const std::array<double, kSwerveModuleCount> & targets) noexcept;
  // Called at a stopped control-cycle boundary; preserves every trajectory/branch state.
  bool set_alignment_policy(double threshold, double release) noexcept;

private:
  SwerveSetpointParameters parameters_;
  std::array<bool, kSwerveModuleCount> reversed_{};
  std::optional<bool> translation_reversed_;
  std::array<std::optional<double>, kSwerveModuleCount> previous_targets_{};
  std::array<double, kSwerveModuleCount> steering_rates_{};
  std::array<double, kSwerveModuleCount> steering_accelerations_{};
  bool gated_{false};
  // One jerk-limited step toward a goal ``delta`` away; updates rate and acceleration.
  double jerk_limited_step(std::size_t index, double delta, double dt_seconds) noexcept;
  // Jerk-limited ramp of an in-flight rate to zero (hold); returns the step.
  double jerk_limited_stop(std::size_t index, double dt_seconds) noexcept;
};

[[nodiscard]] double wrap_pi(double angle_rad) noexcept;
[[nodiscard]] bool valid_steering_angle_limits(
  const SteeringAngleLimits & limits) noexcept;
[[nodiscard]] bool steering_angle_within_limits(
  double angle_rad, const SteeringAngleLimits & limits) noexcept;
[[nodiscard]] bool steering_measurement_within_tolerance(
  double angle_rad, const SteeringAngleLimits & limits) noexcept;
[[nodiscard]] double clamp_steering_measurement_to_safe_range(
  double angle_rad, const SteeringAngleLimits & limits);
[[nodiscard]] std::optional<double> nearest_equivalent_steering_target(
  double angle_rad,
  double current_angle_rad,
  const SteeringAngleLimits & limits) noexcept;
[[nodiscard]] ChassisSpeeds discretize(const ChassisSpeeds & speeds, double dt_seconds);

[[nodiscard]] std::array<SwerveModuleState, kSwerveModuleCount> inverse_kinematics(
  const ChassisSpeeds & speeds,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  const std::array<double, kSwerveModuleCount> & previous_angles_rad,
  double velocity_deadband_mps);

void desaturate_wheel_speeds(
  std::array<SwerveModuleState, kSwerveModuleCount> & states,
  double maximum_speed_mps);

[[nodiscard]] OptimizedModuleState optimize_module(
  const SwerveModuleState & desired, double current_unwrapped_angle_rad);

[[nodiscard]] OptimizedModuleState optimize_module(
  const SwerveModuleState & desired,
  double current_angle_rad,
  bool previous_reversed,
  double hysteresis_rad,
  const SteeringAngleLimits & limits = {});

[[nodiscard]] AlignmentResult optimize_and_apply_alignment(
  const std::array<SwerveModuleState, kSwerveModuleCount> & desired,
  const std::array<double, kSwerveModuleCount> & current_unwrapped_angles_rad,
  double alignment_threshold_rad);

[[nodiscard]] std::optional<Translation2d> wheel_translation_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  double delta_yaw_rad);

[[nodiscard]] std::optional<ChassisDelta> wheel_chassis_delta_from_position_deltas(
  const std::array<SwerveModulePosition, kSwerveModuleCount> & previous,
  const std::array<SwerveModulePosition, kSwerveModuleCount> & current,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations);

[[nodiscard]] std::optional<ChassisSpeeds> chassis_speeds_from_module_states(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations);

[[nodiscard]] std::optional<ChassisSpeedFit> chassis_speeds_with_slip_rejection(
  const std::array<SwerveModuleMeasurement, kSwerveModuleCount> & modules,
  const std::array<Translation2d, kSwerveModuleCount> & module_locations,
  double slip_residual_threshold);

}  // namespace swerve_driver

#endif  // SWERVE_DRIVER__SWERVE_KINEMATICS_HPP_
