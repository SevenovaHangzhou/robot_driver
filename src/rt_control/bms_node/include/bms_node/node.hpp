#ifndef BMS_NODE__NODE_HPP_
#define BMS_NODE__NODE_HPP_

#include "rclcpp/node.hpp"

#include <memory>

namespace bms_node
{

[[nodiscard]] std::shared_ptr<rclcpp::Node> make_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

}  // namespace bms_node

#endif  // BMS_NODE__NODE_HPP_
