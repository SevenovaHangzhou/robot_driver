#ifndef SWERVE_DRIVER__CHASSIS_MODE_HANDOFF_HPP_
#define SWERVE_DRIVER__CHASSIS_MODE_HANDOFF_HPP_

#include <array>
#include <cstdint>
#include <limits>

namespace swerve_driver
{
inline constexpr double kHandoffRequired = std::numeric_limits<double>::quiet_NaN();
using HandoffWheels = std::array<double, 4>;  // FL, FR, RL, RR; wheel-side rad or rad/s

enum class HandoffPhase : uint8_t
{
  inactive, navigation, stopping_csv, seed_csp, confirm_csp, operation,
  seed_csv, confirm_csv, inhibited
};
enum class HandoffCause : uint8_t
{
  none, timing, feedback, drive, mode, stop_timeout, switch_timeout,
  seed_moved, acknowledgement, canceled, deactivated, sequence_exhausted
};
struct HandoffStatusPredicate
{
  uint16_t mask{0}, value{0}; // Required explicit device-profile predicate; zero mask is invalid.
};
struct ChassisHandoffConfig
{
  HandoffWheels max_wheel_velocity{}; // Required positive calibrated rad/s.
  HandoffWheels max_wheel_acceleration{}; // Required positive calibrated rad/s^2.
  HandoffWheels position_tolerance{}; // Required positive wheel rad; seed/hold drift bound.
  double stationary_wheel_velocity{kHandoffRequired};
  double stationary_steering_velocity{kHandoffRequired}; // output-axis rad/s
  double stationary_dwell{kHandoffRequired};
  double feedback_timeout{kHandoffRequired};
  double command_timeout{kHandoffRequired};
  double max_update_period{kHandoffRequired};
  double stop_timeout{kHandoffRequired};
  double switch_timeout{kHandoffRequired}; // Entire seed + mode confirmation, not each stage.
  HandoffStatusPredicate enabled{}, csv_ready{}, csp_ready{};
};
struct HandoffDriveFeedback
{
  double wheel_position{kHandoffRequired}, wheel_velocity{kHandoffRequired};
  double steering_velocity{kHandoffRequired};
  // Monotonic seconds: oldest required drive/steering observation, NOT time of read().
  double sample_time{kHandoffRequired};
  uint64_t drive_sequence{0}; // Increases on each genuinely new drive PDO observation.
  uint16_t status_word{0};
  int8_t mode{0}; // Actual 0x6061.
  bool healthy{false}; // Includes bus, drive fault and enabled/healthy CSP steering evidence.
  bool mode_ack{true}; // Adapter-confirmed mode/zero transaction; legacy core fixtures set true.
};
using HandoffFeedback = std::array<HandoffDriveFeedback, 4>;
struct HandoffWriteAck
{
  uint64_t write_sequence{0};
  uint64_t feedback_sequence_at_send{0};
};
using HandoffAcknowledgements = std::array<HandoffWriteAck, 4>;
struct HandoffNavigationCommand
{
  HandoffWheels wheel_velocity{};
  uint64_t generation{0};
  double received_at{kHandoffRequired}; // Original /cmd_vel receipt, never dequeue/restamp time.
};
struct HandoffOutput
{
  HandoffWheels wheel_velocity{}, wheel_position{};
  int8_t requested_mode{0}; // 0x6060; only meaningful when write_mode is true.
  uint64_t write_sequence{0};
  bool write_velocity{false}, write_position{false}, write_mode{false};
  bool inhibited{true};
};

// ELECTRI-145. Fixed-size single RT owner; serialize all calls/mailboxes outside this class.
// No controlword, SDO, service, bus, allocation or enable ownership. Backend MUST:
//  * write only flagged targets/mode, in the preconfigured cyclic PDO lane;
//  * acknowledge an exact write_sequence only AFTER all its flagged values were sent for
//    that drive, recording the latest drive feedback sequence present at send time;
//  * never infer acknowledgement from command-interface assignment, 6061, or a timer;
//  * supply new drive sequences and the oldest actual sensor timestamp; healthy includes
//    verified steering mode/state and the existing enable_manager authority;
//  * consume each output only immediately after update() on the same monotonic clock;
//    freeze navigation steering admission at begin_operation() as part of the caller's
//    ownership transfer (this class does not write steering targets);
//  * on inhibited output revoke motion admission and invoke the owner safety/lifecycle
//    policy. Inhibit does not promise physical braking or discard a hardware held target.
// Sequence acknowledgements are retained by the backend until superseded. CSP bit 12
// (Kinco manual printed p154) is additionally required; other status bits are configured.
// In operation(), the relative executor owns CSP targets. Before begin_navigation(), it
// must finish its controlled stop and relinquish its last held target; the coordinator
// preserves that target until CSV confirmation. No simultaneous position writers.
class ChassisModeHandoff final
{
public:
  // initial_generation must be nonzero and never reused across controller/process lifetimes.
  // A latched cause cannot be cleared on this instance; supervised restart creates a new one.
  [[nodiscard]] bool configure(const ChassisHandoffConfig &, uint64_t initial_generation) noexcept;
  // Explicit no-motion startup: all drives already CSV, healthy/stationary, and last actually
  // sent CSV references zero. Admission still waits for zero write ACK, state and dwell.
  [[nodiscard]] bool start_navigation(
    double now, const HandoffFeedback &,
    const HandoffWheels & last_sent_velocity) noexcept;
  [[nodiscard]] bool accept_navigation(const HandoffNavigationCommand &, double now) noexcept;
  // Recompute the current body command's wheel setpoints without changing its receipt
  // or generation. This is the RT kinematics owner, never a callback/queued command.
  [[nodiscard]] bool refine_navigation(const HandoffNavigationCommand &, double now) noexcept;
  void discard_navigation() noexcept {have_command_ = false;}
  [[nodiscard]] bool reconfirm_navigation(double now) noexcept;
  [[nodiscard]] bool begin_operation(double now) noexcept;
  [[nodiscard]] bool begin_navigation(double now, const HandoffWheels & held_csp_target) noexcept;
  void update(double now, const HandoffFeedback &, const HandoffAcknowledgements &) noexcept;
  void cancel() noexcept;
  void deactivate() noexcept;

  [[nodiscard]] HandoffPhase phase() const noexcept {return phase_;}
  [[nodiscard]] HandoffCause cause() const noexcept {return cause_;}
  [[nodiscard]] uint64_t generation() const noexcept {return generation_;}
  [[nodiscard]] bool navigation_open() const noexcept {return phase_ == HandoffPhase::navigation;}
  [[nodiscard]] bool operation_ready() const noexcept {return phase_ == HandoffPhase::operation;}
  [[nodiscard]] const HandoffOutput & output() const noexcept {return output_;}

private:
  [[nodiscard]] HandoffCause check_feedback(double now, const HandoffFeedback &) const noexcept;
  [[nodiscard]] bool request_fresh(double now) const noexcept;
  [[nodiscard]] bool all_mode(int8_t mode) const noexcept;
  [[nodiscard]] bool stationary_sample() const noexcept;
  [[nodiscard]] bool stationary() const noexcept;
  [[nodiscard]] bool seed_stable() const noexcept;
  [[nodiscard]] bool acknowledged(
    const HandoffAcknowledgements &,
    bool newer_feedback) const noexcept;
  [[nodiscard]] bool bump_generation() noexcept;
  [[nodiscard]] bool next_write() noexcept;
  void inhibit(HandoffCause) noexcept;
  void reopen_navigation(double now) noexcept;
  void slew_velocity(double dt, const HandoffWheels & desired) noexcept;

  ChassisHandoffConfig config_{};
  HandoffFeedback feedback_{};
  HandoffOutput output_{};
  HandoffNavigationCommand command_{};
  HandoffPhase phase_{HandoffPhase::inactive};
  HandoffCause cause_{HandoffCause::none};
  uint64_t generation_{0};
  double last_update_{0}, phase_started_{0}, stationary_since_{-1}, admission_since_{0};
  int8_t source_mode_{9};
  bool configured_{false}, have_command_{false};
};
}  // namespace swerve_driver
#endif  // SWERVE_DRIVER__CHASSIS_MODE_HANDOFF_HPP_
