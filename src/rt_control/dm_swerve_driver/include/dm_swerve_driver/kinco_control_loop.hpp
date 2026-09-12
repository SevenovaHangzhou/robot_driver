#ifndef DM_SWERVE_DRIVER__KINCO_CONTROL_LOOP_HPP_
#define DM_SWERVE_DRIVER__KINCO_CONTROL_LOOP_HPP_

#include "dm_swerve_driver/control_types.hpp"
#include "dm_swerve_driver/kinco_params.hpp"

namespace dm_swerve_driver {
class KincoControlLoop final {
public:
  KincoControlLoop(DriverParameters common, KincoParameters parameters,
    std::unique_ptr<KincoEthercatBus> bus, std::unique_ptr<CanTransport> encoders,
    ControlLoopCallbacks callbacks = {});
  ~KincoControlLoop() noexcept;
  KincoControlLoop(const KincoControlLoop &) = delete;
  KincoControlLoop & operator=(const KincoControlLoop &) = delete;
  bool initialize(std::chrono::steady_clock::time_point now);
  bool step(std::chrono::steady_clock::time_point now);
  void start();
  void stop() noexcept;
  void submit_command(const ChassisSpeeds &, std::chrono::steady_clock::time_point);
  bool submit_imu_yaw(double, std::chrono::steady_clock::time_point, double = 0.0);
  void request_clear_faults() noexcept;
  void restore_fault_state(
    bool, const std::array<std::uint32_t, kKincoAxisCount> &);
  bool is_running() const noexcept;
  ControlLoopStatus status() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::unique_ptr<CanTransport> make_encoder_transport(const KincoParameters &);
}  // namespace dm_swerve_driver
#endif
