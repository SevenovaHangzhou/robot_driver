#ifndef SWERVE_DRIVER__CHASSIS_RUNTIME_HPP_
#define SWERVE_DRIVER__CHASSIS_RUNTIME_HPP_

#include <array>
#include <atomic>
#include <mutex>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rt_control_interfaces/action/chassis_relative_move.hpp"
#include "rt_control_interfaces/srv/chassis_set_mode.hpp"
#include "rt_control_interfaces/srv/chassis_reset_fault.hpp"
#include "swerve_driver/chassis_mode_handoff.hpp"
#include "swerve_driver/control_core.hpp"
#include "swerve_driver/relative_move_session.hpp"

namespace swerve_driver
{
// Single producer/consumer, fixed storage, no RT locks, allocation or retry loop.
// Non-RT producers/consumers are serialized by the endpoint mutex.
template<class T, size_t N>
class ChassisMailbox
{
public:
  bool push(const T & value) noexcept
  {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % N;
    if (next == read_.load(std::memory_order_acquire)) {return false;}
    slots_[write] = value; write_.store(next, std::memory_order_release); return true;
  }
  bool pop(T & value) noexcept
  {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {return false;}
    value = slots_[read]; read_.store((read + 1) % N, std::memory_order_release); return true;
  }

private:
  std::array<T, N> slots_{};
  std::atomic<size_t> read_{0}, write_{0};
};

struct ChassisRuntimeFeedback
{
  MoveFeedback move{};
  HandoffFeedback drives{};
  HandoffAcknowledgements ack{};
  HandoffWheels sent_velocity{};
};
struct ChassisRuntimeOutput
{
  HandoffOutput drive{};
  HandoffWheels steering{};
};

// Runtime extension of SwerveController. It does not own hardware lifecycle or controlword.
// Callback requests and RT results cross bounded value mailboxes. Tokens/ROS handles stay
// on the non-RT side. All RT methods run on the controller manager's one update thread.
class ChassisRuntime final : public std::enable_shared_from_this<ChassisRuntime>
{
public:
  using Action = rt_control_interfaces::action::ChassisRelativeMove;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using SetMode = rt_control_interfaces::srv::ChassisSetMode;
  using Reset = rt_control_interfaces::srv::ChassisResetFault;
  static void declare_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  ChassisRuntime(const rclcpp_lifecycle::LifecycleNode::SharedPtr &, const CoreConfig &);
  bool activate(double now, const ChassisRuntimeFeedback &, uint64_t lifetime);
  void deactivate(); // non-RT, update already quiesced
  ChassisRuntimeOutput update(
    double now, double dt, int64_t ros_ns,
    const ChassisRuntimeFeedback &, const ControlOutput & navigation,
    double command_receipt, uint64_t command_generation, bool command_valid) noexcept;
  bool navigation_open() const noexcept {return handoff_.navigation_open();} // RT only
  uint64_t navigation_generation() const noexcept {return navigation_generation_.load();}
  uint64_t wire_base() const noexcept {return wire_base_;}

private:
  enum class RequestKind {mode, goal, reset};
  struct Request
  {
    RequestKind kind{RequestKind::mode}; uint64_t lifetime{0}, id{0};
    ChassisMode mode{ChassisMode::unknown}; MoveRequest goal{}; double receipt{0};
  };
  struct Snapshot
  {
    MoveSnapshot move{}; HandoffPhase phase{HandoffPhase::inactive};
    HandoffCause cause{HandoffCause::none}; double time{0};
    uint64_t response_id{0}, goal_id{0}; uint16_t response_code{0};
    int64_t ros_ns{0}, execution_ns{0}, estimate_ns{0};
    double feedback_age{0}, imu_age{0};
    RelativeMoveSession admission{}; // Fixed-size copy for identical non-RT goal preview.
  };
  void publish();
  void drain_snapshots();
  bool snapshot_fresh() const;
  rt_control_interfaces::msg::ChassisState chassis_state() const;
  rt_control_interfaces::msg::ChassisMoveState move_state() const;
  MoveRequest convert(const Action::Goal &) const;
  void complete_goal(bool deactivated);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  MoveSessionConfig config_{};
  ChassisHandoffConfig handoff_config_{};
  ChassisModeHandoff handoff_{};
  RelativeMoveSession session_{};
  ChassisMailbox<Request, 2> requests_;
  ChassisMailbox<Snapshot, 8> snapshots_;
  std::atomic<uint64_t> navigation_generation_{0}, cancel_id_{0};
  std::mutex endpoint_mutex_;
  bool active_{false}, reserved_{false}, session_live_{false};
  uint64_t lifetime_{0}, wire_base_{0}, request_id_{0}, goal_id_{0}, rt_goal_id_{0};
  uint64_t pending_mode_id_{0}, rt_response_id_{0}, last_measurement_{0};
  uint16_t rt_response_code_{0};
  ChassisMode pending_mode_{ChassisMode::unknown};
  HandoffWheels steering_{};
  Snapshot latest_{};
  int64_t execution_ns_{0}, estimate_ns_{0};
  std::shared_ptr<Handle> handle_;
  std::shared_ptr<rmw_request_id_t> mode_header_, reset_header_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Service<SetMode>::SharedPtr mode_service_;
  rclcpp::Service<Reset>::SharedPtr reset_service_;
  rclcpp::Publisher<rt_control_interfaces::msg::ChassisState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
#endif
