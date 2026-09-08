#ifndef DM_SWERVE_DRIVER__STEERING_REZERO_HPP_
#define DM_SWERVE_DRIVER__STEERING_REZERO_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

enum class SteeringRezeroFailureReason {
  save_zero_acknowledgement_missing,
  verification_feedback_missing,
  position_out_of_tolerance,
};

struct SteeringRezeroFailure {
  std::uint16_t esc_id{0U};
  SteeringRezeroFailureReason reason{
    SteeringRezeroFailureReason::save_zero_acknowledgement_missing};
};

struct SteeringRezeroResult {
  std::vector<SteeringRezeroFailure> failures;

  [[nodiscard]] bool success() const noexcept {return failures.empty();}
};

[[nodiscard]] SteeringRezeroResult perform_steering_rezero(
  const DriverParameters & parameters,
  CanTransport & transport);
[[nodiscard]] std::string steering_rezero_failure_message(
  const SteeringRezeroResult & result);

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__STEERING_REZERO_HPP_
