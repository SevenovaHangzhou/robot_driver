#ifndef DM_SWERVE_DRIVER__SRC__CONTROL_LOOP_IMPL_HPP_
#define DM_SWERVE_DRIVER__SRC__CONTROL_LOOP_IMPL_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "dm_swerve_driver/control_loop.hpp"
#include "dm_swerve_driver/safety_monitor.hpp"
#include "motor_startup.hpp"

namespace dm_swerve_driver {

using SteadyClock = std::chrono::steady_clock;

struct TimedCommand {
  ChassisSpeeds value{};
  SteadyClock::time_point timestamp{};
  bool valid{false};
};

struct TimedYaw {
  double value{0.0};
  SteadyClock::time_point timestamp{};
  bool valid{false};
};

struct MailboxSnapshot {
  TimedCommand command{};
  TimedYaw yaw{};
};

struct CyclePlan {
  MailboxSnapshot mailbox{};
  CommandDecision command{};
  ChassisSpeeds discrete_command{};
  AlignmentResult alignment{};
  bool drive_gated{false};
};

class ControlLoop::Impl {
public:
  Impl(
    DriverParameters parameters,
    std::unique_ptr<CanTransport> transport,
    ControlLoopCallbacks callbacks);
  ~Impl() noexcept;

  [[nodiscard]] bool initialize(SteadyClock::time_point now);
  [[nodiscard]] bool step(SteadyClock::time_point now);
  void start();
  void stop() noexcept;
  void submit_command(const ChassisSpeeds & command, SteadyClock::time_point timestamp);
  void submit_imu_yaw(double yaw_rad, SteadyClock::time_point timestamp);
  void request_clear_faults() noexcept;
  [[nodiscard]] bool is_running() const noexcept;
  [[nodiscard]] ControlLoopStatus status() const;

private:
  void initialize_odometry(SteadyClock::time_point now);
  [[nodiscard]] std::array<DmMotor *, kMotorCount> motor_pointers() noexcept;
  [[nodiscard]] StartupLogger logger();
  void log(DriverLogLevel level, const std::string & message) const noexcept;
  [[nodiscard]] MailboxSnapshot mailbox_snapshot() const;
  [[nodiscard]] SteadyClock::duration publish_period() const;
  [[nodiscard]] std::array<SwerveModulePosition, kSwerveModuleCount>
  current_module_positions(
    const std::optional<std::array<bool, kMotorCount>> & received) const;
  [[nodiscard]] std::array<double, kSwerveModuleCount> current_angles();
  [[nodiscard]] std::array<DmMotorHealth, kMotorCount> motor_health() noexcept;

  [[nodiscard]] bool execute_cycle(SteadyClock::time_point now);
  [[nodiscard]] CyclePlan prepare_cycle(SteadyClock::time_point now);
  [[nodiscard]] std::vector<CanFrame> make_cycle_frames(const CyclePlan & plan);
  [[nodiscard]] FeedbackRouteResult exchange_cycle_frames(
    const std::vector<CanFrame> & commands, SteadyClock::time_point now);
  [[nodiscard]] Pose2d update_cycle_odometry(
    const MailboxSnapshot & mailbox,
    const std::array<bool, kMotorCount> & received,
    SteadyClock::time_point now,
    bool & imu_fallback);
  void process_recovery(SteadyClock::time_point now);
  void mark_missing_feedback(const std::array<bool, kMotorCount> & received);
  void update_cycle_status(
    const FeedbackRouteResult & route,
    const Pose2d & pose,
    bool command_timed_out,
    bool imu_fallback,
    bool bus_silent);
  void dispatch_recovery_actions(
    const RecoveryActions & actions, SteadyClock::time_point now);
  void send_special_actions(
    const std::array<bool, kMotorCount> & selected,
    SpecialCommand command,
    SteadyClock::time_point now);

  void maybe_publish(
    SteadyClock::time_point now,
    const Pose2d & pose,
    const ChassisSpeeds & command,
    bool alignment_gated);
  [[nodiscard]] ControlLoopOutput make_output(
    SteadyClock::time_point now,
    const Pose2d & pose,
    const ChassisSpeeds & command,
    bool alignment_gated) const;
  void refresh_status();
  void send_zero_cycles() noexcept;
  [[nodiscard]] std::vector<CanFrame> make_zero_frames(double dt);
  void configure_realtime_priority() noexcept;
  void run() noexcept;

  DriverParameters parameters_;
  std::unique_ptr<CanTransport> transport_;
  ControlLoopCallbacks callbacks_;
  std::array<SwerveModule, kSwerveModuleCount> modules_;
  SafetyMonitor safety_;
  std::optional<SwerveOdometry> odometry_;
  std::optional<std::array<SwerveModulePosition, kSwerveModuleCount>> previous_positions_;
  std::array<double, kSwerveModuleCount> last_angles_{};
  mutable std::mutex mailbox_mutex_;
  TimedCommand command_mailbox_{};
  TimedYaw yaw_mailbox_{};
  mutable std::mutex status_mutex_;
  ControlLoopStatus status_{};
  std::mutex io_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> clear_faults_requested_{false};
  bool initialized_{false};
  std::thread thread_;
  SteadyClock::time_point last_publish_time_{};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SRC__CONTROL_LOOP_IMPL_HPP_
