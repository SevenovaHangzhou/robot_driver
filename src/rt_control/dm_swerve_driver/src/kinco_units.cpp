#include "dm_swerve_driver/kinco_units.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace dm_swerve_driver {
namespace {

constexpr double kTau{6.283185307179586476925286766559};
constexpr double kSecondsPerMinute{60.0};
constexpr double kVelocityNumerator{512.0};
constexpr double kVelocityDenominator{1875.0};
constexpr double kPercentPerRatio{100.0};

void validate_resolution(std::uint32_t encoder_resolution)
{
  if (encoder_resolution == 0U) {
    throw std::invalid_argument{"Kinco encoder resolution must be positive"};
  }
}

template<typename Integer>
[[nodiscard]] Integer checked_round(double value, const char * description)
{
  if (value < static_cast<double>(std::numeric_limits<Integer>::lowest()) ||
    value > static_cast<double>(std::numeric_limits<Integer>::max()))
  {
    throw std::out_of_range{description};
  }
  return static_cast<Integer>(std::llround(value));
}

void validate_finite(double value, const char * description)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument{description};
  }
}

}  // namespace

std::int32_t kinco_position_from_radians(
  double radians, std::uint32_t encoder_resolution)
{
  validate_finite(radians, "Kinco position must be finite");
  validate_resolution(encoder_resolution);
  const double increments{radians / kTau * static_cast<double>(encoder_resolution)};
  return checked_round<std::int32_t>(increments, "Kinco position exceeds Integer32");
}

double kinco_position_to_radians(
  std::int32_t position, std::uint32_t encoder_resolution)
{
  validate_resolution(encoder_resolution);
  return static_cast<double>(position) * kTau / static_cast<double>(encoder_resolution);
}

std::int32_t kinco_velocity_from_radians_per_second(
  double radians_per_second, std::uint32_t encoder_resolution)
{
  validate_finite(radians_per_second, "Kinco velocity must be finite");
  validate_resolution(encoder_resolution);
  const double rpm{radians_per_second * kSecondsPerMinute / kTau};
  const double drive_units{
    rpm * kVelocityNumerator * static_cast<double>(encoder_resolution) /
    kVelocityDenominator};
  return checked_round<std::int32_t>(drive_units, "Kinco velocity exceeds Integer32");
}

double kinco_velocity_to_radians_per_second(
  std::int32_t velocity, std::uint32_t encoder_resolution)
{
  validate_resolution(encoder_resolution);
  const double rpm{
    static_cast<double>(velocity) * kVelocityDenominator /
    (kVelocityNumerator * static_cast<double>(encoder_resolution))};
  return rpm * kTau / kSecondsPerMinute;
}

std::int16_t kinco_torque_percent_from_ratio(double rated_torque_ratio)
{
  validate_finite(rated_torque_ratio, "Kinco torque ratio must be finite");
  return checked_round<std::int16_t>(
    rated_torque_ratio * kPercentPerRatio,
    "Kinco torque percentage exceeds Integer16");
}

double kinco_torque_ratio_from_percent(std::int16_t torque_percent) noexcept
{
  return static_cast<double>(torque_percent) / kPercentPerRatio;
}

}  // namespace dm_swerve_driver
