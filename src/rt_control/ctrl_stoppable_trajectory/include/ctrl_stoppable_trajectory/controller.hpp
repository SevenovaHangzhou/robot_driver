#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rt_control_interfaces/srv/stop_joint_trajectory.hpp"
#include "ctrl_stoppable_trajectory/stop_profile.hpp"

namespace ctrl_stoppable_trajectory
{
class Controller : public joint_trajectory_controller::JointTrajectoryController
{
public:
  using Base = joint_trajectory_controller::JointTrajectoryController;
  using CallbackReturn = controller_interface::CallbackReturn;
  using Stop = rt_control_interfaces::srv::StopJointTrajectory;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  enum class Phase { idle, preparing, queued, braking, complete, rejected, fault };
  // One bounded request slot. NRT writes only in idle; RT reads after acquire.
  // Completion is consumed by the NRT timer before the slot is made idle again.
  struct RequestSlot
  {
    execution_stop::GoalId id{};
    execution_stop::Limits limits{};
    std::vector<double> origin, direction;
    std::size_t lead{};
    double length{};
    RealtimeGoalHandlePtr goal;
    std::shared_ptr<rmw_request_id_t> header;
    rclcpp::Time applied{0, 0, RCL_ROS_TIME};
    double duration{};
    std::atomic<std::uint8_t> result{Stop::Response::FAILED};
  } request_;
  std::atomic<Phase> phase_{Phase::idle};
  std::atomic<bool> active_{false}, own_reference_{false};
  std::atomic<bool> response_pending_{false};
  std::atomic<std::uint64_t> normal_generation_{0};
  std::uint64_t stopped_generation_{};
  execution_stop::Receiver receiver_;
  std::vector<double> cut_positions_, stop_direction_;
  execution_stop::GoalId stop_id_{};
  double cut_distance_{};
  std::int64_t previous_tick_{};
  bool owns_reference_{};  // RT only; own_reference_ is its NRT-visible snapshot.
  bool detached_goal_{};
  rclcpp::Service<Stop>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr completion_timer_;

  void receive_stop(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<Stop::Request>);
  bool prepare_line(const FollowJTrajAction::Goal &);
  void finish_request();
  void invalidate_request();
  void begin_stop(const rclcpp::Time &);
  controller_interface::return_type update_stop(const rclcpp::Time &);
  bool feedback_fresh() const;
  void publish_stop_state();
};
}  // namespace ctrl_stoppable_trajectory
