#ifndef CTRL_SWERVE__NAVIGATION_FEEDBACK_HPP_
#define CTRL_SWERVE__NAVIGATION_FEEDBACK_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include "ctrl_swerve/control_core.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ctrl_swerve
{
// Wire layout: robot_interfaces contract/navigation-execution-v1.md (#334).
constexpr std::size_t kExecutionStateSize = 91;
using ExecutionStateArray = std::array<double, kExecutionStateSize>;
ExecutionStateArray encode_execution_state(
  const ExecutionSnapshot & snapshot, double stamp, uint64_t sequence, bool enabled, bool ready,
  bool command_fresh, bool request_fresh, ControlStatus status) noexcept;

class NavigationFeedback final
{
public:
  explicit NavigationFeedback(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void publish(
    const ExecutionSnapshot & snapshot, const rclcpp::Time & stamp, double dt, bool enabled,
    bool ready, bool command_fresh, bool request_fresh, ControlStatus status);

private:
  using StateMessage = std_msgs::msg::Float64MultiArray;
  using StatePublisher = realtime_tools::RealtimePublisher<StateMessage>;
  std::unique_ptr<StatePublisher> state_, compatibility_;
  rclcpp::Publisher<StateMessage>::SharedPtr state_publisher_, compatibility_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  StateMessage inactive_state_;
  sensor_msgs::msg::JointState inactive_joints_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> joints_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_;
  rclcpp::TimerBase::SharedPtr rotation_timer_;
  // Grant publication never reads mutable control state from the executor thread.
  std::atomic<int64_t> grant_refresh_ns_{0};
  double elapsed_{0.0};
  std::atomic<uint64_t> sequence_{0};
  std::atomic_bool enabled_{false};
  bool rotation_tick_{false};
};
}  // namespace ctrl_swerve
#endif
