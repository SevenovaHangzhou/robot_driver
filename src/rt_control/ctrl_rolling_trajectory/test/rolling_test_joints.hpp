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

}  // namespace ctrl_rolling_trajectory
#endif

#endif  // ROLLING_TEST_JOINTS_HPP_
