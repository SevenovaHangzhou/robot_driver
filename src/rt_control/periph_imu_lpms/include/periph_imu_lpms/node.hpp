#ifndef PERIPH_IMU_LPMS__NODE_HPP_
#define PERIPH_IMU_LPMS__NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace periph_imu_lpms
{
[[nodiscard]] rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options);
}  // namespace periph_imu_lpms

#endif  // PERIPH_IMU_LPMS__NODE_HPP_
