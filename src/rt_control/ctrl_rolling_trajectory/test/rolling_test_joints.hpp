#ifndef ROLLING_TEST_JOINTS_HPP_
#define ROLLING_TEST_JOINTS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ctrl_rolling_trajectory/axis_set.hpp"

namespace ctrl_rolling_trajectory
{

// Test data: the Alfa_V3 arm order.
inline const JointNames kTestJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "right_joint7", "left_joint1",  "left_joint2",  "left_joint3",
  "left_joint4",  "left_joint5",  "left_joint6",  "left_joint7"};

}  // namespace ctrl_rolling_trajectory

// Pure core tests do not link rclcpp; only controller tests need the parameter helper.
#if __has_include("rclcpp/parameter.hpp")
#include "rclcpp/parameter.hpp"

namespace ctrl_rolling_trajectory
{

inline rclcpp::Parameter testJointsParameter()
{
  return rclcpp::Parameter(
    "joints", std::vector<std::string>(kTestJointNames.begin(), kTestJointNames.end()));
}

// Complete, valid controller parameters as the composer would render them. The limits are
// deliberately generous (+-10 rad, 10 rad/s, 10 rad/s^2, 0.01 rad margin) so tests choose
// trajectories freely; splice tolerances are near zero so splices must be exact.
inline std::vector<rclcpp::Parameter> testControllerParameters()
{
  const std::size_t axes = kTestJointNames.size();
  return {
    testJointsParameter(),
    rclcpp::Parameter("joint_position_lower", std::vector<double>(axes, -10.0)),
    rclcpp::Parameter("joint_position_upper", std::vector<double>(axes, 10.0)),
    rclcpp::Parameter("joint_velocity_limits", std::vector<double>(axes, 10.0)),
    rclcpp::Parameter("joint_acceleration_limits", std::vector<double>(axes, 10.0)),
    rclcpp::Parameter("position_margin", 0.01),
    rclcpp::Parameter("buffer_capacity", std::int64_t{64}),
    rclcpp::Parameter("max_horizon_ms", std::int64_t{600}),
    rclcpp::Parameter("update_timeout_ms", std::int64_t{200}),
    rclcpp::Parameter("replace_lead_ms", std::int64_t{16}),
    rclcpp::Parameter("state_publish_period_ms", std::int64_t{20}),
    rclcpp::Parameter("nominal_controller_period_ms", std::int64_t{4}),
    rclcpp::Parameter("maximum_controller_period_ms", std::int64_t{8}),
    rclcpp::Parameter("one_cycle_detection_guard_ms", std::int64_t{4}),
    rclcpp::Parameter("stop_time_growth_guard_ms", std::int64_t{4}),
    rclcpp::Parameter("non_rt_to_rt_visibility_guard_ms", std::int64_t{4}),
    rclcpp::Parameter("period_quantization_guard_ms", std::int64_t{4}),
    rclcpp::Parameter("feedback_age_limit_ms", 500.0),
    rclcpp::Parameter("takeover_tolerances", std::vector<double>(axes, 0.125)),
    rclcpp::Parameter("splice_position_tolerances", std::vector<double>(axes, 1.0e-9)),
    rclcpp::Parameter("splice_velocity_tolerances", std::vector<double>(axes, 1.0e-9))};
}

}  // namespace ctrl_rolling_trajectory
#endif

#endif  // ROLLING_TEST_JOINTS_HPP_
