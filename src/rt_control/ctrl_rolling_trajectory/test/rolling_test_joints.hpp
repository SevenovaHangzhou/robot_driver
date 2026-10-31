#ifndef ROLLING_TEST_JOINTS_HPP_
#define ROLLING_TEST_JOINTS_HPP_

#include <string>
#include <vector>

#include "ctrl_rolling_trajectory/axis_set.hpp"

namespace ctrl_rolling_trajectory
{

// Test data: the Alfa_V3 arm order, which the frozen protocol axis-set hash was computed from.
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

// The composer computes both digests; these are its values for the test joints and the
// Alfa_V3 provisional envelope (integ_composer test_compose pins the same strings).
inline rclcpp::Parameter testAxisSetHashParameter()
{
  return rclcpp::Parameter(
    "axis_set_hash", "f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4");
}

inline rclcpp::Parameter testProvisionalLimitsVersionParameter()
{
  return rclcpp::Parameter(
    "limits_version", "1673393117a50ed04b7b9fa99dd01dbeb171cc691e9733a65e49fb13b418baab");
}

}  // namespace ctrl_rolling_trajectory
#endif

#endif  // ROLLING_TEST_JOINTS_HPP_
