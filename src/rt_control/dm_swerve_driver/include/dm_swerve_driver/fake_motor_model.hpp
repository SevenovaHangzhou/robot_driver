#ifndef DM_SWERVE_DRIVER__FAKE_MOTOR_MODEL_HPP_
#define DM_SWERVE_DRIVER__FAKE_MOTOR_MODEL_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

#include "dm_swerve_driver/dm_frame_codec.hpp"

namespace dm_swerve_driver {

struct FakeMotorConfig {
  std::uint16_t esc_id{1U};
  std::uint16_t mst_id{0x11U};
  MotorLimits limits{12.5, 30.0, 10.0};
  std::chrono::duration<double> velocity_time_constant{0.03};
};

class FakeMotorModel final {
public:
  explicit FakeMotorModel(FakeMotorConfig config);

  [[nodiscard]] std::uint16_t esc_id() const noexcept;
  [[nodiscard]] std::uint16_t mst_id() const noexcept;
  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] double position() const noexcept;
  [[nodiscard]] double velocity() const noexcept;

  [[nodiscard]] std::optional<CanFrame> handle_frame(
    const CanFrame & request, std::chrono::duration<double> elapsed);

private:
  [[nodiscard]] CanFrame feedback_frame() const;
  [[nodiscard]] std::optional<CanFrame> handle_register(const CanFrame & request);

  FakeMotorConfig config_;
  bool enabled_{false};
  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  std::uint32_t timeout_register_{0U};
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__FAKE_MOTOR_MODEL_HPP_
