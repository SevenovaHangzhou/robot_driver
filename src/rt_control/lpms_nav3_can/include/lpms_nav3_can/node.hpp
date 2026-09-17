#ifndef LPMS_NAV3_CAN__NODE_HPP_
#define LPMS_NAV3_CAN__NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace lpms_nav3_can
{
[[nodiscard]] rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options);
}  // namespace lpms_nav3_can

#endif  // LPMS_NAV3_CAN__NODE_HPP_
