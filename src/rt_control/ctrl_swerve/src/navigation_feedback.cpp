#include "ctrl_swerve/navigation_feedback.hpp"
#include "ctrl_swerve/swerve_controller.hpp"

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
  ControlStatus status) noexcept
{
  using W = ExecutionStateWire;
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  ExecutionStateArray data;
  data.fill(unavailable);
  data[W::Version] = 1.0;
  data[W::Stamp] = stamp;
  data[W::Sequence] = static_cast<double>(sequence);
  data[W::Ready] =
    enabled && ready && s.ready && s.feedback_valid &&
        (status == ControlStatus::running || status == ControlStatus::alignment_gated ||
         status == ControlStatus::command_timeout)
      ? 1.0
      : 0.0;
  data[W::Status] = static_cast<double>(status);
  data[W::Gated] = s.gated ? 1.0 : 0.0;
  data[W::Waiting] = s.waiting ? 1.0 : 0.0;
  data[W::TranslationBranch] = static_cast<double>(s.translation_branch);
  if (enabled && ready) {
    data[W::BodyVelocity] = s.body_reference.vx_mps;
    data[W::BodyVelocity + 1] = s.body_reference.vy_mps;
    data[W::BodyVelocity + 2] = s.body_reference.omega_radps;
    data[W::BodyAcceleration] = s.body_acceleration.vx_mps;
    data[W::BodyAcceleration + 1] = s.body_acceleration.vy_mps;
    data[W::BodyAcceleration + 2] = s.body_acceleration.omega_radps;
  }
  if (enabled && s.feedback_valid) {
    data[W::MeasuredTwist] = s.body_measured.vx_mps;
    data[W::MeasuredTwist + 1] = s.body_measured.vy_mps;
    data[W::MeasuredTwist + 2] = s.body_measured.omega_radps;
  }
  data[W::SteeringSpeed] = s.modules[0].steering_max_rate;
  data[W::SteeringAcceleration] = s.modules[0].steering_max_acceleration;
  data[W::SteeringJerk] = s.modules[0].steering_max_jerk;
  data[W::BodyAccelerationLimit] = s.max_linear_acceleration;
  data[W::BodyJerk] = s.max_body_jerk;
  data[W::WheelAcceleration] = s.modules[0].wheel_max_acceleration;
  data[W::WheelSpeed] = s.modules[0].wheel_max_speed;
  data[W::Gate] = s.gate;
  data[W::Release] = s.release;
  data[W::FlipHysteresis] = s.flip_hysteresis;
  for (std::size_t i = 0; i < s.modules.size(); ++i) {
    const auto & m = s.modules[i];
    const std::size_t b = W::Modules + W::Stride * i;
    data[b + W::Id] = static_cast<double>(i + 1);
    if (enabled && m.ready) {
      // Prediction consumes the same measured control-axis angle as the core.
      data[b + W::Angle] = m.steering_actual;
      data[b + W::WheelVelocity] = m.wheel_actual;
    }
    if (enabled && ready) {
      data[b + W::ReferenceAngle] = m.steering_reference;
      data[b + W::ReferenceRate] = m.steering_rate;
      data[b + W::ReferenceAcceleration] = m.steering_acceleration;
      data[b + W::ReferenceWheelVelocity] = m.wheel_reference;
      data[b + W::Goal] = m.steering_target;
      data[b + W::Error] = m.ready ? m.steering_error : unavailable;
    }
    data[b + W::Reversed] = m.reversed ? 1.0 : 0.0;
    data[b + W::Holding] = m.holding ? 1.0 : 0.0;
    data[b + W::Minimum] = m.steering_min;
    data[b + W::Maximum] = m.steering_max;
    data[b + W::Radius] = m.wheel_radius;
    data[b + W::X] = m.location.x;
    data[b + W::Y] = m.location.y;
  }
  for (auto & value : data) {
    if (!std::isfinite(value)) {
      value = unavailable;
      data[W::Ready] = 0.0;
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
    "/rt/base/terminal_rotation_active", rclcpp::QoS(1).reliable().durability_volatile());
  inactive_state_ = state_->msg_;
  inactive_joints_ = joints;
  const auto unavailable = std::numeric_limits<double>::quiet_NaN();
  const auto inactive =
    encode_execution_state(ExecutionSnapshot{}, 0.0, 0, false, false, ControlStatus::inactive);
  std::copy(inactive.begin(), inactive.end(), inactive_state_.data.begin());
  std::fill(inactive_joints_.position.begin(), inactive_joints_.position.end(), unavailable);
  std::fill(inactive_joints_.velocity.begin(), inactive_joints_.velocity.end(), unavailable);
  rotation_timer_ =
    node->create_wall_timer(std::chrono::milliseconds(10), [this, clock = node->get_clock()]() {
      if (!enabled_.load()) {
        const auto stamp = clock->now();
        inactive_state_.data[ExecutionStateWire::Stamp] = stamp.seconds();
        inactive_state_.data[ExecutionStateWire::Sequence] =
          static_cast<double>(sequence_.fetch_add(1) & ((1ULL << 53) - 1));
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
  bool ready, bool request_fresh, ControlStatus status,
  const std::array<double, 4> & observed_steering_rates)
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
    snapshot, stamp.seconds(), sequence_.fetch_add(1) & ((1ULL << 53) - 1), enabled, ready, status);
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
      const bool valid = enabled && ready && snapshot.feedback_valid && module.ready;
      message.position[2 * i] = valid ? module.steering_encoder : unavailable;
      message.position[2 * i + 1] = valid ? module.wheel_position : unavailable;
      message.velocity[2 * i] = valid ? observed_steering_rates[i] : unavailable;
      message.velocity[2 * i + 1] = valid ? module.wheel_actual / module.wheel_radius : unavailable;
    }
    joints_->unlockAndPublish();
  }
}

void SwerveController::reset_steering_observations() noexcept
{
  for (auto & observer : steering_observers_) {
    observer.reset();
  }
  observed_steering_rates_.fill(std::numeric_limits<double>::quiet_NaN());
  last_observation_stamp_ns_ = 0;
}

void SwerveController::observe_steering(
  const ModuleFeedbackArray & feedback, const rclcpp::Time & stamp, int64_t now, bool valid)
{
  if (
    !valid || stamp.nanoseconds() <= last_observation_stamp_ns_ ||
    (last_observation_stamp_ns_ > 0 &&
     static_cast<double>(stamp.nanoseconds() - last_observation_stamp_ns_) * 1e-9 >
       config_.max_update_period)) {
    reset_steering_observations();
    return;
  }
  last_observation_stamp_ns_ = stamp.nanoseconds();
  for (std::size_t i = 0; i < feedback.size(); ++i) {
    observed_steering_rates_[i] = steering_observers_[i].observe(
      feedback[i].steering_angle, states_[9 * i + 8]->get_value(), static_cast<double>(now) * 1e-9,
      feedback[i].encoder_valid, feedback_timeout_,
      config_.steering_min[i] + config_.steering_limit_margin[i],
      config_.steering_max[i] - config_.steering_limit_margin[i],
      config_.setpoint.maximum_steering_slew_radps, config_.steering_limit_tolerance[i]);
  }
}
}  // namespace ctrl_swerve
