#ifndef DM_SWERVE_DRIVER__KINCO_BACKEND_HPP_
#define DM_SWERVE_DRIVER__KINCO_BACKEND_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "dm_swerve_driver/kinco_ds402.hpp"
#include "dm_swerve_driver/kinco_pdo.hpp"
#include "dm_swerve_driver/swerve_kinematics.hpp"

namespace dm_swerve_driver {

struct KincoSdoWrite {
  std::size_t axis_index{0U};
  std::uint16_t index{0U};
  std::uint8_t subindex{0U};
  std::uint8_t bit_length{0U};
  std::uint32_t raw_value{0U};
};

struct EthercatDomainStatus {
  std::uint32_t working_counter{0U};
  std::uint32_t expected_working_counter{0U};
  bool link_up{false};
  bool all_slaves_operational{false};

  [[nodiscard]] bool healthy() const noexcept;
};

struct KincoEthercatCycle {
  KincoFeedbackBatch feedback{};
  EthercatDomainStatus domain{};
};

class KincoEthercatBus {
public:
  virtual ~KincoEthercatBus() = default;

  virtual void open() = 0;
  virtual void configure_preop(const std::vector<KincoSdoWrite> & writes) = 0;
  virtual void activate() = 0;
  [[nodiscard]] virtual KincoEthercatCycle exchange(
    const KincoCommandBatch & commands) = 0;
  virtual void deactivate() noexcept = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;
};

struct KincoSwerveHardwareConfig {
  std::array<std::uint32_t, kSwerveModuleCount> steering_encoder_resolution{};
  std::array<std::uint32_t, kSwerveModuleCount> drive_encoder_resolution{};
  double steering_gear_ratio{0.0};
  double drive_gear_ratio{0.0};
  double wheel_radius_m{0.0};
  std::array<double, kSwerveModuleCount> steering_zero_offset_rad{};
  std::array<bool, kSwerveModuleCount> steering_inverted{};
  std::array<bool, kSwerveModuleCount> drive_inverted{};
  SteeringAngleLimits steering_limits{};
  std::size_t startup_cycle_limit{0U};
  std::optional<std::uint16_t> position_velocity_feedforward_raw;
  std::optional<std::uint16_t> position_acceleration_feedforward;
  std::optional<std::int16_t> fault_reaction_option_code;
};

void validate_kinco_hardware_config(const KincoSwerveHardwareConfig & config);

struct KincoModuleTarget {
  double steering_angle_rad{0.0};
  double wheel_speed_mps{0.0};
  bool drive_enabled{false};
};

struct KincoModuleFeedback {
  double motor_steering_angle_rad{0.0};
  double wheel_distance_m{0.0};
  double wheel_speed_mps{0.0};
};

struct KincoHardwareCycle {
  std::array<KincoModuleFeedback, kSwerveModuleCount> modules{};
  std::array<KincoFaultReport, kKincoAxisCount> axis_faults{};
  KincoEthercatCycle raw{};
  bool valid{false};
};

class KincoSwerveHardware final {
public:
  KincoSwerveHardware(
    KincoSwerveHardwareConfig config,
    std::unique_ptr<KincoEthercatBus> bus);
  ~KincoSwerveHardware() noexcept;

  KincoSwerveHardware(const KincoSwerveHardware &) = delete;
  KincoSwerveHardware & operator=(const KincoSwerveHardware &) = delete;
  KincoSwerveHardware(KincoSwerveHardware &&) = delete;
  KincoSwerveHardware & operator=(KincoSwerveHardware &&) = delete;

  [[nodiscard]] bool initialize();
  [[nodiscard]] bool prepare(bool validate_position = true);
  [[nodiscard]] bool enable();
  [[nodiscard]] KincoHardwareCycle feedback() const;
  [[nodiscard]] KincoHardwareCycle exchange(
    const std::array<KincoModuleTarget, kSwerveModuleCount> & targets);
  [[nodiscard]] KincoHardwareCycle exchange(
    const std::array<KincoModuleTarget, kSwerveModuleCount> & targets,
    const std::array<std::uint16_t, kKincoAxisCount> & control_words);
  void stop() noexcept;
  [[nodiscard]] bool initialized() const noexcept;

private:
  [[nodiscard]] KincoCommandBatch disabled_commands() const noexcept;
  [[nodiscard]] std::vector<KincoSdoWrite> preop_writes() const;
  [[nodiscard]] bool startup_cycle_healthy(const KincoEthercatCycle & cycle) const noexcept;
  [[nodiscard]] bool all_axes_enabled(const KincoEthercatCycle & cycle) const noexcept;
  void close_bus() noexcept;

  KincoSwerveHardwareConfig config_;
  std::unique_ptr<KincoEthercatBus> bus_;
  KincoEthercatCycle last_cycle_{};
  std::array<std::int32_t, kSwerveModuleCount> safe_hold_positions_{};
  bool initialized_{false};
  bool prepared_{false};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_BACKEND_HPP_
