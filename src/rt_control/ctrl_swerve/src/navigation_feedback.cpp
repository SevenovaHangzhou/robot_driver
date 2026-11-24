#include "ctrl_swerve/navigation_feedback.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include "robot_interfaces_qos/profiles.hpp"

namespace ctrl_swerve
{
namespace
{
int64_t steady_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}
}  // namespace

ExecutionStateArray encode_execution_state(
  const ExecutionSnapshot & s, double stamp, uint64_t sequence, bool enabled, bool ready,
  bool command_fresh, bool request_fresh, ControlStatus status) noexcept
{
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  ExecutionStateArray data;
  data.fill(unavailable);
  data[0] = 1.0;
  data[1] = static_cast<double>(sequence);
  data[2] = stamp;
  data[3] = enabled && ready && s.ready ? 1.0 : 0.0;
  data[4] = static_cast<double>(status);
  data[5] = enabled && s.gated ? 1.0 : 0.0;
  data[6] = enabled && s.waiting ? 1.0 : 0.0;
  data[7] = enabled && ready && request_fresh && s.rotation_active ? 1.0 : 0.0;
  data[8] = enabled && request_fresh ? 1.0 : 0.0;
  if (enabled && ready) {
    data[9] = s.body_reference.vx_mps;
    data[10] = s.body_reference.vy_mps;
    data[11] = s.body_reference.omega_radps;
    data[12] = s.body_acceleration.vx_mps;
    data[13] = s.body_acceleration.vy_mps;
    data[14] = s.body_acceleration.omega_radps;
  }
  if (enabled && s.feedback_valid) {
    data[15] = s.body_measured.vx_mps;
    data[16] = s.body_measured.vy_mps;
    data[17] = s.body_measured.omega_radps;
  }
  data[18] = s.max_linear_speed;
  data[19] = s.max_angular_speed;
  data[20] = s.max_linear_acceleration;
  data[21] = s.max_angular_acceleration;
  data[22] = s.modules[0].steering_max_rate;
  data[23] = s.modules[0].steering_max_acceleration;
  data[24] = s.modules[0].steering_max_jerk;
  data[25] = s.modules[0].wheel_max_speed;
  data[26] = s.modules[0].wheel_max_acceleration;
  data[27] = s.max_body_jerk;
  data[28] = enabled ? 1.0 : 0.0;
  data[29] = enabled && s.feedback_valid ? 1.0 : 0.0;
  data[30] = enabled && command_fresh ? 1.0 : 0.0;
  for (std::size_t i = 0; i < s.modules.size(); ++i) {
    const auto & m = s.modules[i];
    const std::size_t b = 31 + 15 * i;
    if (enabled && m.ready) {
      data[b] = m.steering_encoder;
      data[b + 1] = m.wheel_actual;
    }
    if (enabled && ready) {
      data[b + 2] = m.steering_reference;
      data[b + 3] = m.steering_rate;
      data[b + 4] = m.steering_acceleration;
      data[b + 5] = m.steering_target;
      data[b + 6] = m.ready ? m.steering_error : unavailable;
      data[b + 7] = m.reversed ? -1.0 : 1.0;
      data[b + 14] = m.wheel_reference;
    } else {
      data[b + 7] = 0.0;
    }
    data[b + 8] = m.steering_min;
    data[b + 9] = m.steering_max;
    data[b + 10] = m.wheel_radius;
    data[b + 11] = m.location.x;
    data[b + 12] = m.location.y;
    data[b + 13] = enabled && m.ready ? 1.0 : 0.0;
  }
  for (auto & value : data) {
    if (!std::isfinite(value)) {
      value = unavailable;
    }
  }
  return data;
}

NavigationFeedback::NavigationFeedback(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  state_publisher_ = node->create_publisher<StateMessage>(
    "/rt/base/execution_state", robot_interfaces_qos::fast_state());
  compatibility_publisher_ = node->create_publisher<StateMessage>(
    "/rt_control/chassis/execution_state", robot_interfaces_qos::fast_state());
  state_ = std::make_unique<StatePublisher>(state_publisher_);
  compatibility_ = std::make_unique<StatePublisher>(compatibility_publisher_);
  for (auto * publisher : {state_.get(), compatibility_.get()}) {
    publisher->msg_.data.resize(kExecutionStateSize);
    publisher->msg_.layout.dim.resize(1);
    auto & dimension = publisher->msg_.layout.dim[0];
    dimension.label = "rt_base_execution_state_v1";
    dimension.size = kExecutionStateSize;
    dimension.stride = kExecutionStateSize;
  }
  joint_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
    "/rt_control/chassis/joint_states", robot_interfaces_qos::fast_state());
  joints_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    joint_publisher_);
  auto & joints = joints_->msg_;
  joints.header.frame_id = "base_footprint";
  joints.name.reserve(8);
  for (std::size_t i = 0; i < 4; ++i) {
    joints.name.push_back("module_" + std::to_string(i + 1) + "_steering");
    joints.name.push_back("module_" + std::to_string(i + 1) + "_wheel");
  }
  joints.position.resize(8);
  joints.velocity.resize(8);
  rotation_ = node->create_publisher<std_msgs::msg::Bool>(
    "/rt/base/terminal_rotation_active", robot_interfaces_qos::fast_state());
  inactive_state_ = state_->msg_;
  inactive_joints_ = joints;
  const auto unavailable = std::numeric_limits<double>::quiet_NaN();
  std::fill(inactive_state_.data.begin(), inactive_state_.data.end(), unavailable);
  inactive_state_.data[0] = 1.0;
  for (const auto index : {3U, 4U, 5U, 6U, 7U, 8U, 28U, 29U, 30U}) {
    inactive_state_.data[index] = 0.0;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    inactive_state_.data[31 + 15 * i + 7] = 0.0;
    inactive_state_.data[31 + 15 * i + 13] = 0.0;
  }
  std::fill(inactive_joints_.position.begin(), inactive_joints_.position.end(), unavailable);
  std::fill(inactive_joints_.velocity.begin(), inactive_joints_.velocity.end(), unavailable);
  rotation_timer_ =
    node->create_wall_timer(std::chrono::milliseconds(10), [this, clock = node->get_clock()]() {
      if (!enabled_.load()) {
        const auto stamp = clock->now();
        inactive_state_.data[1] = static_cast<double>(sequence_.fetch_add(1) & ((1ULL << 53) - 1));
        inactive_state_.data[2] = stamp.seconds();
        inactive_joints_.header.stamp = stamp;
        state_publisher_->publish(inactive_state_);
        compatibility_publisher_->publish(inactive_state_);
        joint_publisher_->publish(inactive_joints_);
      }
      rotation_tick_ = !rotation_tick_;
      if (rotation_tick_) {
        const auto refreshed = grant_refresh_ns_.load();
        const auto now = steady_ns();
        std_msgs::msg::Bool message;
        message.data = refreshed > 0 && now >= refreshed && now - refreshed <= 300000000;
        rotation_->publish(message);
      }
    });
}

void NavigationFeedback::publish(
  const ExecutionSnapshot & snapshot, const rclcpp::Time & stamp, double dt, bool enabled,
  bool ready, bool command_fresh, bool request_fresh, ControlStatus status)
{
  enabled_.store(enabled);
  grant_refresh_ns_.store(
    enabled && ready && request_fresh && snapshot.rotation_active ? steady_ns() : 0);
  if (std::isfinite(dt) && dt > 0.0) {
    elapsed_ += dt;
  }
  if (elapsed_ < 0.01) {
    return;
  }
  elapsed_ = std::fmod(elapsed_, 0.01);
  const auto data = encode_execution_state(
    snapshot, stamp.seconds(), sequence_.fetch_add(1) & ((1ULL << 53) - 1), enabled, ready,
    command_fresh, request_fresh, status);
  // Both aliases carry the same sequence and snapshot; no RT resizing or blocking lock.
  if (state_->trylock()) {
    if (compatibility_->trylock()) {
      std::copy(data.begin(), data.end(), state_->msg_.data.begin());
      std::copy(data.begin(), data.end(), compatibility_->msg_.data.begin());
      state_->unlockAndPublish();
      compatibility_->unlockAndPublish();
    } else {
      state_->unlock();
    }
  }
  if (joints_->trylock()) {
    auto & message = joints_->msg_;
    message.header.stamp = stamp;
    const auto unavailable = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < 4; ++i) {
      const auto & module = snapshot.modules[i];
      const bool valid = enabled && module.ready;
      message.position[2 * i] = valid ? module.steering_encoder : unavailable;
      message.position[2 * i + 1] = valid ? module.wheel_position : unavailable;
      message.velocity[2 * i] = unavailable;
      message.velocity[2 * i + 1] = valid ? module.wheel_actual / module.wheel_radius : unavailable;
    }
    joints_->unlockAndPublish();
  }
}
}  // namespace ctrl_swerve
