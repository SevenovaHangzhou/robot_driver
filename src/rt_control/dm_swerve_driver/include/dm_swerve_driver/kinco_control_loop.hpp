#ifndef DM_SWERVE_DRIVER__KINCO_CONTROL_LOOP_HPP_
#define DM_SWERVE_DRIVER__KINCO_CONTROL_LOOP_HPP_

#include "dm_swerve_driver/control_loop.hpp"
#include "dm_swerve_driver/kinco_params.hpp"

namespace dm_swerve_driver {
class KincoControlLoop final : public ControlRunner {
public:
  KincoControlLoop(DriverParameters common, KincoParameters parameters,
    std::unique_ptr<KincoEthercatBus> bus, std::unique_ptr<CanTransport> encoders,
    ControlLoopCallbacks callbacks = {});
  ~KincoControlLoop() noexcept override;
  KincoControlLoop(const KincoControlLoop &) = delete;
  KincoControlLoop & operator=(const KincoControlLoop &) = delete;
  bool initialize(std::chrono::steady_clock::time_point now) override;
  bool step(std::chrono::steady_clock::time_point now);
  void start() override;
  void stop() noexcept override;
  void submit_command(const ChassisSpeeds &, std::chrono::steady_clock::time_point) override;
  bool submit_imu_yaw(double, std::chrono::steady_clock::time_point, double = 0.0) override;
  void request_clear_faults() noexcept override;
  void restore_fault_state(bool, const std::array<std::uint32_t, kMotorCount> &) override;
  bool is_running() const noexcept override;
  ControlLoopStatus status() const override;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::unique_ptr<CanTransport> make_encoder_transport(const KincoParameters &);
}  // namespace dm_swerve_driver
#endif
