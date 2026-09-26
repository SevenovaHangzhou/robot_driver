#ifndef SWERVE_DRIVER__RELATIVE_MOVE_SESSION_HPP_
#define SWERVE_DRIVER__RELATIVE_MOVE_SESSION_HPP_

#include <array>
#include <cstdint>
#include "swerve_driver/relative_move_planner.hpp"

namespace swerve_driver
{
enum class ChassisMode : uint8_t {unknown, navigation, operation};
enum class MovePhase : uint8_t {idle, switching, aligning, executing, stopping, holding, fault};
enum class MoveFault : uint16_t {
  none, imu_lost, feedback_invalid, slip, steering_error, drive, bus, mode_switch, timeout, execution};
enum class MoveResult : uint16_t {succeeded, canceled, faulted, timed_out, none};

struct MoveSessionConfig
{
  RelativeMoveConfig motion{};
  double max_steering_velocity{kRelativeMoveRequired};
  double max_steering_acceleration{kRelativeMoveRequired};
  double feedback_timeout{kRelativeMoveRequired};
  double imu_timeout{kRelativeMoveRequired};
  double imu_max_increment{kRelativeMoveRequired};
  double max_update_period{kRelativeMoveRequired};
  double stationary_wheel_velocity{kRelativeMoveRequired};
  double stationary_steering_velocity{kRelativeMoveRequired};
  double stationary_dwell{kRelativeMoveRequired};
  double alignment_tolerance{kRelativeMoveRequired};
  double alignment_dwell{kRelativeMoveRequired};
  double alignment_timeout{kRelativeMoveRequired};
  double steering_error{kRelativeMoveRequired};
  double steering_error_dwell{kRelativeMoveRequired};
  double encoder_difference{kRelativeMoveRequired};
  double position_tolerance{kRelativeMoveRequired};
  double velocity_tolerance{kRelativeMoveRequired};
  double settling_dwell{kRelativeMoveRequired};
  double settling_timeout{kRelativeMoveRequired};
  double slip_threshold{kRelativeMoveRequired};
  double slip_dwell{kRelativeMoveRequired};
  double pose_translation_tolerance{kRelativeMoveRequired};
  double pose_yaw_tolerance{kRelativeMoveRequired};
  double yaw_discrepancy{kRelativeMoveRequired};
  double switch_timeout{kRelativeMoveRequired};
  double stop_timeout{kRelativeMoveRequired};
  double max_goal_duration{kRelativeMoveRequired};
};

struct MoveFeedback
{
  RelativeMoveStart positions{};
  std::array<double, 4> wheel_velocity{};
  std::array<double, 4> steering_velocity{};
  std::array<int, 4> steering_mode{};
  std::array<int, 4> drive_mode{};
  double age{kRelativeMoveRequired};  // oldest required encoder/drive sample age, seconds
  double imu_age{kRelativeMoveRequired};
  double imu_yaw{kRelativeMoveRequired};  // already aligned base-frame yaw
  bool imu_valid{false};
  bool bus_ok{false};
  bool drives_ok{false};
};
struct MoveRequest
{
  Pose2d goal{};
  RelativeMoveConfig limits{};  // geometry is ignored; only six velocity/acceleration limits used
  double max_duration{kRelativeMoveRequired};
};
struct MoveOutput
{
  std::array<double, 4> steering_position{};
  std::array<double, 4> steering_velocity{};
  std::array<double, 4> wheel_position{};
  std::array<double, 4> wheel_velocity{};
  ChassisMode requested_mode{ChassisMode::unknown};
  bool inhibited{true}; // no promise of control authority after drive/bus/feedback loss
};
struct MoveSnapshot
{
  ChassisMode mode{ChassisMode::unknown};
  MovePhase phase{MovePhase::idle};
  MoveFault fault{MoveFault::none};
  MoveResult result{MoveResult::none};
  bool ready{false}, stationary{false}, imu_valid{false}, active{false}, estimate_valid{false};
  double progress{0.0};
  Pose2d actual{};
  double imu_yaw{0.0}, wheel_yaw{0.0};
  uint32_t quality{0};
  uint64_t measurement_sequence{0}; // advances only for coherent estimate snapshots
};

// Fixed-size, single-owner core. No ROS, locks, allocation, device access or enable writer.
// Caller supplies monotonic dt and timestamped feedback; lifetime restart discards all work.
class RelativeMoveSession final
{
public:
  [[nodiscard]] bool configure(const MoveSessionConfig & config) noexcept;
  // Called only by the cyclic handoff owner AFTER all-four sent/readback confirmation.
  // Preserve its frozen CSP seed instead of taking a second measured-position seed.
  [[nodiscard]] bool enter_confirmed_operation(const MoveFeedback &,
    const std::array<double, 4> & wheel_seed, const std::array<double, 4> & steering_seed) noexcept;
  void update(double dt, const MoveFeedback & feedback) noexcept;
  // Codes match private service Response constants. 0 means request started/completed;
  // asynchronous backend users must await switching completion before returning service OK.
  [[nodiscard]] uint16_t set_mode(ChassisMode mode, bool confirm) noexcept;
  [[nodiscard]] uint16_t reset_fault(bool confirm) noexcept;
  [[nodiscard]] bool start(const MoveRequest & request) noexcept;
  void cancel() noexcept;
  // Accepted mailbox command became inadmissible before execution; latch without motion.
  void reject_execution() noexcept {stop(MoveFault::execution, MoveResult::faulted);}
  [[nodiscard]] const MoveSnapshot & state() const noexcept {return state_;}
  [[nodiscard]] const MoveOutput & output() const noexcept {return output_;}
  [[nodiscard]] const RelativeMovePlan & plan() const noexcept {return planner_.plan();}

private:
  struct Ramp {
    double p{0}, v{0}, elapsed{0}, a{0}, peak{0}, ta{0}, tc{0}, duration{0};
    double stop_p{0}, stop_v{0};
    bool stopping{false}, done{true};
    void reset(double distance, double velocity, double acceleration) noexcept;
    void advance(double dt) noexcept;
    void cancel() noexcept;
  } alignment_{};
  [[nodiscard]] bool healthy_feedback(const MoveFeedback & feedback) const noexcept;
  [[nodiscard]] bool valid_imu(const MoveFeedback & feedback) const noexcept;
  [[nodiscard]] ChassisMode read_mode(const MoveFeedback & feedback) const noexcept;
  void stop(MoveFault fault, MoveResult result) noexcept;
  void finish(MoveResult result) noexcept;
  void estimate(const MoveFeedback & feedback) noexcept;
  void references() noexcept;
  MoveSessionConfig config_{};
  MoveFeedback feedback_{};
  MoveSnapshot state_{};
  MoveOutput output_{};
  RelativeMovePlanner planner_{};
  std::array<SwerveModulePosition, 4> previous_positions_{};
  std::array<Translation2d, 4> locations_{};
  double previous_imu_{0}, stationary_time_{0}, phase_time_{0}, elapsed_{0}, duration_{0};
  double alignment_good_time_{0}, settled_time_{0}, slip_time_{0}, steering_error_time_{0};
  MoveResult stop_result_{MoveResult::canceled};
  bool configured_{false}, stopping_alignment_{false};
};
}  // namespace swerve_driver
#endif
