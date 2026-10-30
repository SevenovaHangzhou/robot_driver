#ifndef PERIPH_BMS__NODE_HPP_
#define PERIPH_BMS__NODE_HPP_

#include "rclcpp/node.hpp"

#include <memory>

namespace periph_bms
{

[[nodiscard]] std::shared_ptr<rclcpp::Node> make_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

}  // namespace periph_bms

#endif  // PERIPH_BMS__NODE_HPP_
