#ifndef ROLLING_TEST_JOINTS_HPP_
#define ROLLING_TEST_JOINTS_HPP_

#include <string>
#include <vector>

#include "rolling_trajectory_controller/axis_set.hpp"

namespace rolling_trajectory_controller
{

// Test data: the Alfa_V3 arm order, which the frozen protocol axis-set hash was computed from.
inline const JointNames kTestJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "right_joint7", "left_joint1",  "left_joint2",  "left_joint3",
  "left_joint4",  "left_joint5",  "left_joint6",  "left_joint7"};

}  // namespace rolling_trajectory_controller

// Pure core tests do not link rclcpp; only controller tests need the parameter helper.
#if __has_include("rclcpp/parameter.hpp")
#include "rclcpp/parameter.hpp"

namespace rolling_trajectory_controller
{

inline rclcpp::Parameter testJointsParameter()
{
  return rclcpp::Parameter(
    "joints", std::vector<std::string>(kTestJointNames.begin(), kTestJointNames.end()));
}

}  // namespace rolling_trajectory_controller
#endif

#endif  // ROLLING_TEST_JOINTS_HPP_
