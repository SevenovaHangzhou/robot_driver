#ifndef SWERVE_DRIVER__RELATIVE_MOVE_MOCK_NODE_HPP_
#define SWERVE_DRIVER__RELATIVE_MOVE_MOCK_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rt_control_interfaces/action/chassis_relative_move.hpp"
#include "rt_control_interfaces/srv/chassis_set_mode.hpp"
#include "rt_control_interfaces/srv/chassis_reset_fault.hpp"
#include "swerve_driver/relative_move_session.hpp"

namespace swerve_driver
{
// Explicit offline adapter. No hardware interfaces, plugin loaders, buses or controlwords.
// All callbacks use the default mutually-exclusive group. The fixed session has one owner;
// ROS allocations/publication stay outside update(). This is not a production RT executor.
class RelativeMoveMockNode final : public rclcpp::Node
{
public:
  explicit RelativeMoveMockNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  using Action = rt_control_interfaces::action::ChassisRelativeMove;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using SetMode = rt_control_interfaces::srv::ChassisSetMode;
  using ResetFault = rt_control_interfaces::srv::ChassisResetFault;
private:
  friend class RelativeMoveMockTestAccess;  // Tests control dispatch, never fabricate sample ages.
  void tick();
  void simulate(double dt);
  struct ReceiptFeedback
  {
    MoveFeedback feedback;
    double elapsed;
    bool cycle_fresh, encoders_fresh, imu_fresh;
  };
  ReceiptFeedback effective_feedback(std::chrono::steady_clock::time_point receipt) const;
  MoveRequest convert(const Action::Goal & goal) const;
  rt_control_interfaces::msg::ChassisState chassis_state() const;
  rt_control_interfaces::msg::ChassisMoveState move_state() const;
  double required(const std::string & name);
  std::array<double, 4> required_array(const std::string & name);
  MoveSessionConfig config_{};
  RelativeMoveSession session_{};
  MoveFeedback feedback_{};
  ChassisMode mock_mode_{ChassisMode::navigation};
  std::array<SwerveModulePosition, 4> mock_previous_{};
  std::array<Translation2d, 4> locations_{};
  double mock_yaw_{0}, period_{0};
  std::chrono::steady_clock::time_point previous_tick_{}, accepted_at_{};
  rclcpp::Time execution_start_{0, 0, RCL_ROS_TIME}, estimate_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t estimate_sequence_{0};
  // Capacity-one command slots; no goal/velocity queue and no retained commands.
  bool reserved_{false}, started_{false}, cancel_pending_{false};
  MoveRequest pending_goal_{};
  rclcpp_action::GoalUUID goal_id_{};
  std::shared_ptr<Handle> handle_{};
  std::shared_ptr<rmw_request_id_t> mode_header_{};
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Service<SetMode>::SharedPtr mode_service_;
  rclcpp::Service<ResetFault>::SharedPtr reset_service_;
  rclcpp::Publisher<rt_control_interfaces::msg::ChassisState>::SharedPtr state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
#endif
