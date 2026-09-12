#ifndef DM_SWERVE_DRIVER__KINCO_UNITS_HPP_
#define DM_SWERVE_DRIVER__KINCO_UNITS_HPP_

#include <cstdint>

namespace dm_swerve_driver {

[[nodiscard]] std::int32_t kinco_position_from_radians(
  double radians, std::uint32_t encoder_resolution);
[[nodiscard]] double kinco_position_to_radians(
  std::int32_t position, std::uint32_t encoder_resolution);

[[nodiscard]] std::int32_t kinco_velocity_from_radians_per_second(
  double radians_per_second, std::uint32_t encoder_resolution);
[[nodiscard]] double kinco_velocity_to_radians_per_second(
  std::int32_t velocity, std::uint32_t encoder_resolution);

[[nodiscard]] std::int16_t kinco_torque_percent_from_ratio(double rated_torque_ratio);
[[nodiscard]] double kinco_torque_ratio_from_percent(std::int16_t torque_percent) noexcept;

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__KINCO_UNITS_HPP_
