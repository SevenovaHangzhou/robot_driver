#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include "ctrl_swerve/navigation_feedback.hpp"
#include "ctrl_swerve/steering_velocity_observer.hpp"
#include "navigation_feedback_test_support.hpp"
// Both are the unmodified supplied navigation sources, not a local decoder copy.
#include "swerve_mpc_controller/driver_prediction.hpp"
#include "stopped_maneuver.hpp"

namespace
{
using W = ctrl_swerve::ExecutionStateWire;
namespace nav = swerve_mpc_controller;

class NavigationConsumerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  ctrl_swerve::CoreConfig config = ctrl_swerve::test::feedback_config();
  ctrl_swerve::ControlCore core{config};
  ctrl_swerve::ModuleFeedbackArray modules = ctrl_swerve::test::stopped_feedback();
  std::array<ctrl_swerve::SteeringVelocityObserver, 4> observers{};
  const std::array<int, 4> ids{{4, 2, 1, 3}};  // Consumer order differs from producer order.
  nav::DriverPredictionLimits limits;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<ctrl_swerve::NavigationFeedback> publisher;
  std_msgs::msg::Float64MultiArray::SharedPtr state, compatibility;
  sensor_msgs::msg::JointState::SharedPtr joints;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub, compatibility_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub;
  bool active_seen{false}, active{true};

  void SetUp() override
  {
    ASSERT_TRUE(core.activate(modules));
    core.update(modules, {}, true, 0.004);
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const auto source = static_cast<std::size_t>(ids[i] - 1);
      limits.x[i] = config.locations[source].x;
      limits.y[i] = config.locations[source].y;
    }
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("navigation_feedback_producer");
    client = std::make_shared<rclcpp::Node>("real_navigation_consumer_probe");
    executor.add_node(node->get_node_base_interface());
    executor.add_node(client);
    publisher = std::make_unique<ctrl_swerve::NavigationFeedback>(node);
    state_sub = client->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rt/base/execution_state", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr value) { state = value; });
    compatibility_sub = client->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rt_control/chassis/execution_state", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr value) { compatibility = value; });
    joint_sub = client->create_subscription<sensor_msgs::msg::JointState>(
      "/rt_control/chassis/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr value) { joints = value; });
    active_sub = client->create_subscription<std_msgs::msg::Bool>(
      "/rt/base/terminal_rotation_active", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::SharedPtr value) {
        active_seen = true;
        active = value->data;
      });
  }

  std::array<double, 4> observe(double angle, double age_ms, double now, bool valid = true)
  {
    std::array<double, 4> rates;
    for (std::size_t i = 0; i < rates.size(); ++i) {
      rates[i] = observers[i].observe(
        angle, age_ms, now, valid, 0.02, config.steering_min[i] + config.steering_limit_margin[i],
        config.steering_max[i] - config.steering_limit_margin[i],
        config.setpoint.maximum_steering_slew_radps, config.steering_limit_tolerance[i]);
    }
    return rates;
  }

  void deliver(const std::array<double, 4> & rates, bool enabled = true, bool ready = true)
  {
    state.reset();
    compatibility.reset();
    joints.reset();
    const auto stamp = node->now();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    do {
      publisher->publish(
        core.execution_snapshot(), stamp, 0.01, enabled, ready, false,
        core.execution_snapshot().status, rates);
      executor.spin_some();
      if (
        state && compatibility && joints && state->data[W::Stamp] == stamp.seconds() &&
        state->data[W::Sequence] == compatibility->data[W::Sequence] &&
        rclcpp::Time(joints->header.stamp).nanoseconds() == stamp.nanoseconds()) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    FAIL() << "DDS feedback aliases/joints did not deliver the same published snapshot";
  }

  bool stopped(std::string & reason)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = joints->header.stamp;
    odometry.child_frame_id = "base_footprint";
    const auto & measured = core.execution_snapshot().body_measured;
    odometry.twist.twist.linear.x = measured.vx_mps;
    odometry.twist.twist.linear.y = measured.vy_mps;
    odometry.twist.twist.angular.z = measured.omega_radps;
    return robot_nav_control::stoppedManeuverFeedback(
      odometry, *joints, rclcpp::Time(joints->header.stamp).seconds(), "base_footprint", &reason);
  }
};

TEST_F(NavigationConsumerTest, PublishedWireIsDecodedByUnmodifiedNavigation)
{
  deliver(observe(0.0, 0.5, 10.0));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_EQ(state->data, compatibility->data);
  nav::DriverPredictionFeedback decoded;
  std::string reason;
  ASSERT_TRUE(
    nav::decodeExecutionState(state->data, node->now().seconds(), ids, limits, decoded, reason))
    << reason;
  const auto & snapshot = core.execution_snapshot();
  EXPECT_EQ(decoded.translation_branch, snapshot.translation_branch);
  EXPECT_EQ(decoded.reference_waiting, snapshot.waiting);
  EXPECT_EQ(decoded.gated, snapshot.gated);
  EXPECT_DOUBLE_EQ(decoded.reference_gate, snapshot.gate);
  EXPECT_DOUBLE_EQ(decoded.reference_body_velocity.wz, snapshot.body_reference.omega_radps);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const auto & module = snapshot.modules[static_cast<std::size_t>(ids[i] - 1)];
    EXPECT_EQ(decoded.reference_holding[i], module.holding);
    EXPECT_EQ(decoded.reversed[i], module.reversed);
    EXPECT_DOUBLE_EQ(decoded.angles[i], module.steering_actual);
    EXPECT_DOUBLE_EQ(decoded.reference_angles[i], module.steering_reference);
    EXPECT_DOUBLE_EQ(decoded.reference_wheel_speeds[i], module.wheel_reference);
  }
  EXPECT_FALSE(nav::decodeExecutionState(
    state->data, state->data[W::Stamp] + 0.101, ids, limits, decoded, reason));
  EXPECT_EQ(reason, "timestamp");
  // The driver must not copy consumer geometry just to pass its policy check.
  EXPECT_FALSE(nav::decodeExecutionState(
    state->data, state->data[W::Stamp], ids, nav::DriverPredictionLimits{}, decoded, reason));
  EXPECT_EQ(reason, "geometry_mismatch");
  auto duplicate = state->data;
  duplicate[W::Modules + W::Stride + W::Id] = duplicate[W::Modules + W::Id];
  EXPECT_FALSE(
    nav::decodeExecutionState(duplicate, state->data[W::Stamp], ids, limits, decoded, reason));
}

TEST_F(NavigationConsumerTest, RealStoppedCheckRequiresTwoFreshEncoderObservations)
{
  std::string reason;
  deliver(observe(0.0, 0.5, 10.0));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_FALSE(stopped(reason));
  EXPECT_NE(reason.find("moving or invalid joint"), std::string::npos);
  deliver(observe(0.0, 3.5, 10.003));  // Same received frame: still unknown.
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_FALSE(stopped(reason));
  deliver(observe(0.0, 0.5, 10.004));
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_TRUE(stopped(reason)) << reason;
  observe(0.0, 3.5, 10.007);
  deliver(observe(0.002, 0.5, 10.008));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_FALSE(stopped(reason));  // Actual observed 0.5 rad/s, reference remains zero.
  EXPECT_NEAR(joints->velocity[0], 0.5, 1e-9);
  deliver(observe(0.002, 1.5, 10.009));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_NEAR(joints->velocity[0], 0.5, 1e-9);  // Repeated old angle cannot fake a stop.
  EXPECT_FALSE(stopped(reason));
  deliver(observe(0.002, 21.5, 10.029));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_TRUE(std::isnan(joints->velocity[0]));
  EXPECT_FALSE(stopped(reason));
}

TEST_F(NavigationConsumerTest, InvalidAndReactivatedFeedbackCannotReuseStoppedEvidence)
{
  observe(0.0, 0.5, 10.0);
  observe(0.0, 3.5, 10.003);
  deliver(observe(0.0, 0.5, 10.004));
  ASSERT_FALSE(HasFatalFailure());
  std::string reason;
  ASSERT_TRUE(stopped(reason)) << reason;
  for (auto & module : modules) {
    module.valid = false;
  }
  core.update(modules, {}, true, 0.004);
  deliver(observe(0.0, 0.0, 10.005, false), true, false);
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_DOUBLE_EQ(state->data[W::Ready], 0.0);
  EXPECT_FALSE(stopped(reason));
  nav::DriverPredictionFeedback decoded;
  EXPECT_FALSE(
    nav::decodeExecutionState(state->data, node->now().seconds(), ids, limits, decoded, reason));
  core.deactivate();
  for (auto & observer : observers) {
    observer.reset();
  }
  deliver(observe(0.0, 0.0, 10.006, false), false, false);
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_DOUBLE_EQ(state->data[W::Status], 0.0);
  modules = ctrl_swerve::test::stopped_feedback();
  ASSERT_TRUE(core.activate(modules));
  core.update(modules, {}, true, 0.004);
  deliver(observe(0.0, 0.5, 10.007));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_FALSE(stopped(reason));
  observe(0.0, 3.5, 10.010);
  deliver(observe(0.0, 0.5, 10.011));
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_TRUE(stopped(reason)) << reason;
  deliver(observe(0.0, 0.0, 9.0));  // Receive-time reset invalidates the rate again.
  ASSERT_FALSE(HasFatalFailure());
  EXPECT_FALSE(stopped(reason));
}

TEST_F(NavigationConsumerTest, ReliableRotationSubscriberReceivesHeartbeat)
{
  deliver(observe(0.0, 0.5, 10.0));
  ASSERT_FALSE(HasFatalFailure());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!active_seen && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(active_seen);
  EXPECT_FALSE(active);
}
}  // namespace
