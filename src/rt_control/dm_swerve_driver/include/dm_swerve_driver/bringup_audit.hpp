#ifndef DM_SWERVE_DRIVER__BRINGUP_AUDIT_HPP_
#define DM_SWERVE_DRIVER__BRINGUP_AUDIT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "dm_swerve_driver/dm_frame_codec.hpp"

namespace dm_swerve_driver {

struct AuditMotor {
  std::uint16_t esc_id{0U};
  std::uint16_t mst_id{0U};
  bool steering{false};
};

struct BringupAuditOptions {
  std::string interface_name{"can0"};
  std::vector<AuditMotor> motors;
  MotorLimits expected_limits{12.5, 30.0, 10.0};
  std::chrono::microseconds feedback_deadline{10000};
  int retries{3};
};

[[nodiscard]] BringupAuditOptions default_bringup_audit_options();
[[nodiscard]] BringupAuditOptions parse_bringup_audit_arguments(
  const std::vector<std::string> & arguments);
[[nodiscard]] std::string bringup_audit_usage();

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__BRINGUP_AUDIT_HPP_
