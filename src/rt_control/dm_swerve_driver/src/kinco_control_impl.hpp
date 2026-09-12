#ifndef DM_SWERVE_DRIVER__SRC__KINCO_CONTROL_IMPL_HPP_
#define DM_SWERVE_DRIVER__SRC__KINCO_CONTROL_IMPL_HPP_
#include <atomic>
#include <mutex>
#include <optional>
#include <thread>
#include "dm_swerve_driver/kinco_control_loop.hpp"
#include "dm_swerve_driver/encoder_snapshot_store.hpp"
#include "dm_swerve_driver/safety_monitor.hpp"

namespace dm_swerve_driver {
using KincoClock = std::chrono::steady_clock;

class KincoControlLoop::Impl {
public:
  Impl(DriverParameters common, KincoParameters parameters,
    std::unique_ptr<KincoEthercatBus> bus, std::unique_ptr<CanTransport> encoders,
    ControlLoopCallbacks callbacks);
  ~Impl() noexcept {stop();}
  bool initialize(KincoClock::time_point now);
  bool step(KincoClock::time_point now);
  void start();
  void stop() noexcept;
  void log(DriverLogLevel level, const std::string & message) const noexcept;
  void update_sources(KincoClock::time_point now);
  void update_health(KincoClock::time_point now);
  std::array<KincoModuleTarget, kSwerveModuleCount> plan(
    KincoClock::time_point now, const ChassisSpeeds & command, bool timeout);
  ControlLoopOutput update_odometry(KincoClock::time_point now,
    const std::optional<TimedYawSample> & imu, double yaw_rate);
  void refresh_status();

  DriverParameters common_;
  KincoParameters parameters_;
  KincoSwerveHardware hardware_;
  CanopenEncoderClient encoders_;
  EncoderSnapshotStore store_;
  ControlLoopCallbacks callbacks_;
  SafetyMonitor safety_;
  SwerveSetpointGenerator planner_;
  std::array<std::optional<SteeringAngleSourceSelector>, kSwerveModuleCount> selectors_{};
  std::array<ExternalSteeringEncoderConfig, kSwerveModuleCount> encoder_configs_{};
  std::array<SteeringAngleSelection, kSwerveModuleCount> sources_{};
  std::array<DmMotorHealth, kMotorCount> health_{};
  std::array<Ds402FaultResetSequence, kMotorCount> reset_sequences_{};
  std::array<bool, kMotorCount> recovering_{};
  std::array<std::uint16_t, kMotorCount> controls_{};
  std::array<double, kSwerveModuleCount> previous_speed_{};
  KincoHardwareCycle feedback_{};
  CanopenEncoderCycle encoder_cycle_{};
  std::array<EncoderHardwareInfo, kSwerveModuleCount> encoder_info_{};
  std::optional<SwerveOdometry> odometry_;
  std::array<SwerveModulePosition, kSwerveModuleCount> previous_positions_{};
  ControlLoopOutput output_{};
  std::atomic<bool> running_{false};
  std::atomic<bool> clear_requested_{false};
  bool initialized_{false};
  bool source_fault_{false};
  bool clearing_{false};
  KincoClock::time_point clear_deadline_{}, last_publish_{}, last_step_{};
  std::thread thread_;
  mutable std::mutex status_mutex_;
  ControlLoopStatus status_{};
  std::mutex io_mutex_;
  std::mutex mailbox_mutex_;
  ChassisSpeeds command_{};
  std::optional<KincoClock::time_point> command_time_;
  std::optional<TimedYawSample> imu_;
  double yaw_rate_{0.0};
};
}  // namespace dm_swerve_driver
#endif
