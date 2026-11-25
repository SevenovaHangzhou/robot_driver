#include <gtest/gtest.h>

#include "std_msgs/msg/bool.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <thread>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "ctrl_swerve/swerve_controller.hpp"
#include "synthetic_parameters.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using Controller = ctrl_swerve::SwerveController;

class SwerveControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  Controller controller;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  std::array<double, 8> commands{};
  std::array<double, 37> states{};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;

  void configure()
  {
    ASSERT_EQ(controller.init("swerve_test"), controller_interface::return_type::OK);
    controller.get_node()->set_parameters(synthetic_parameters());
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  void bind()
  {
    command_handles.clear();
    state_handles.clear();
    const auto c = controller.command_interface_configuration().names;
    const auto s = controller.state_interface_configuration().names;
    ASSERT_EQ(c.size(), commands.size());
    ASSERT_EQ(s.size(), states.size());
    for (size_t i = 0; i < c.size(); ++i) {
      const auto slash = c[i].find_last_of('/');
      command_handles.emplace_back(c[i].substr(0, slash), c[i].substr(slash + 1), &commands[i]);
    }
    for (size_t i = 0; i < s.size(); ++i) {
      const auto slash = s[i].find_last_of('/');
      state_handles.emplace_back(s[i].substr(0, slash), s[i].substr(slash + 1), &states[i]);
    }
    std::vector<hardware_interface::LoanedCommandInterface> c_loaned;
    std::vector<hardware_interface::LoanedStateInterface> s_loaned;
    for (auto & handle : command_handles) {
      c_loaned.emplace_back(handle);
    }
    for (auto & handle : state_handles) {
      s_loaned.emplace_back(handle);
    }
    controller.assign_interfaces(std::move(c_loaned), std::move(s_loaned));
  }

  void SetUp() override
  {
    configure();
    if (HasFatalFailure()) {
      return;
    }
    const auto names = controller.state_interface_configuration().names;
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i].find("status_word") != std::string::npos) {
        states[i] = 0x0027;
      }
      if (names[i].find("mode_of_operation_display") != std::string::npos) {
        states[i] = names[i][0] == 's' ? 8.0 : 9.0;
      }
    }
    bind();
    if (HasFatalFailure()) {
      return;
    }
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    executor.add_node(controller.get_node()->get_node_base_interface());
    client = std::make_shared<rclcpp::Node>("swerve_test_client");
    executor.add_node(client);
    publisher = client->create_publisher<geometry_msgs::msg::Twist>(
      "/rt/base/cmd_vel", robot_interfaces_qos::control());
  }

  void TearDown() override { controller.on_deactivate(rclcpp_lifecycle::State{}); }

  void spin_until(std::function<bool()> done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
    }
    ASSERT_TRUE(done());
  }

  void send(double vx, double vy = 0.0)
  {
    spin_until([&]() { return publisher->get_subscription_count() == 1; });
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    publisher->publish(msg);
    spin_until([&]() {
      controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
      return std::abs(commands[1]) > 0.0 || std::abs(commands[0]) > 0.0;
    });
  }

  void send_without_wait(double vx)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    publisher->publish(msg);
  }

  void state(const std::string & name, double value)
  {
    const auto names = controller.state_interface_configuration().names;
    const auto it = std::find(names.begin(), names.end(), name);
    ASSERT_NE(it, names.end());
    states[static_cast<size_t>(it - names.begin())] = value;
  }

  void expect_stopped()
  {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(commands[2 * i + 1], 0.0);
    }
  }

  void configure_finite_feedback_limits()
  {
    ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    ASSERT_EQ(controller.on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    for (const auto & result : controller.get_node()->set_parameters(
           {{"max_steering_acceleration", 10.0},
            {"max_steering_jerk", 10.0},
            {"max_linear_acceleration", 1.0},
            {"max_angular_acceleration", 1.0},
            {"max_body_jerk", 4.0}})) {
      ASSERT_TRUE(result.successful) << result.reason;
    }
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    bind();
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }
};

TEST_F(SwerveControllerTest, ClaimsEightMotionInterfacesAndNoControlWord)
{
  const auto names = controller.command_interface_configuration().names;
  EXPECT_EQ(names[0], "s0/position");
  EXPECT_EQ(names[1], "d0/velocity");
  for (const auto & name : names) {
    EXPECT_EQ(name.find("control_word"), std::string::npos);
  }
  expect_stopped();
}

TEST_F(SwerveControllerTest, RosCommandProducesCspAndCsvTargets)
{
  send(1.0);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(commands[2 * i], 0.0);
    EXPECT_GT(commands[2 * i + 1], 0.0);
  }
}

TEST_F(SwerveControllerTest, EncoderLossLatchesUntilReactivation)
{
  send(1.0);
  state("e2/feedback_age_ms", 100.0);
  // The external encoder is a monitor: inside the 50 ms window motion continues.
  for (int cycle = 0; cycle < 12; ++cycle) {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
  }
  EXPECT_GT(std::abs(commands[1]), 0.0);
  expect_stopped();
  expect_stopped();
  // ELECTRI-127: encoder loss is a motion fault; recovery alone never resumes motion.
  state("e2/feedback_age_ms", 0.0);
  expect_stopped();
  send_without_wait(1.0);
  for (int cycle = 0; cycle < 20; ++cycle) {
    executor.spin_some();
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
  }
  EXPECT_DOUBLE_EQ(commands[1], 0.0);
  // A new activation (after the operator re-enables) clears the latch.
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  send(1.0);
  EXPECT_GT(commands[1], 0.0);
}

TEST_F(SwerveControllerTest, WrongModeAndBusFaultGateAllDriveAxes)
{
  send(1.0);
  state("s0/mode_of_operation_display", 9.0);
  expect_stopped();
  state("s0/mode_of_operation_display", 8.0);
  state("ethercat_domain/process_data_age_ms", 100.0);
  expect_stopped();
}

TEST_F(SwerveControllerTest, RepeatedActivationRequiresNewCommand)
{
  send(1.0);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  expect_stopped();
}

TEST_F(SwerveControllerTest, UnverifiedConfigurationIsRejected)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameter({"calibration_verified", false});
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, PluginIsInstalled)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.createSharedInstance("ctrl_swerve/SwerveController"));
}

TEST_F(SwerveControllerTest, DoesNotExposeRemovedMotorControls)
{
  const auto parameters = controller.get_node()->list_parameters({}, 10).names;
  for (const auto & name : parameters) {
    for (const auto * removed : {"kp", "kd", "ks", "kv", "ka", "rezero_tolerance_rad"}) {
      EXPECT_NE(name, removed);
      EXPECT_NE(name, std::string("drive.") + removed);
      EXPECT_NE(name, std::string("steering.") + removed);
    }
  }
  for (const auto & entry : controller.get_node()->get_service_names_and_types()) {
    EXPECT_EQ(entry.first.find("rezero"), std::string::npos);
  }
}

TEST_F(SwerveControllerTest, QueuedCommandFromPreviousActivationCannotMoveTheRobot)
{
  spin_until([&]() { return publisher->get_subscription_count() == 1; });
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 1.0;
  publisher->publish(msg);
  ASSERT_TRUE(publisher->wait_for_all_acked(std::chrono::seconds(1)));
  controller.on_deactivate(rclcpp_lifecycle::State{});
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    expect_stopped();
  }
}

TEST_F(SwerveControllerTest, LateralCommandTraversesInterfacesAndProducesMeasuredOdometry)
{
  nav_msgs::msg::Odometry::SharedPtr odometry;
  auto subscription = client->create_subscription<nav_msgs::msg::Odometry>(
    "/rt/base/odom", robot_interfaces_qos::fast_state(),
    [&](nav_msgs::msg::Odometry::SharedPtr message) { odometry = message; });
  send(0.0, 1.0);
  std::array<double, 4> wheel_position{};
  for (int cycle = 0; cycle < 400; ++cycle) {
    for (size_t i = 0; i < 4; ++i) {
      wheel_position[i] += commands[2 * i + 1] * 0.004;
      state("s" + std::to_string(i) + "/position", commands[2 * i]);
      state("e" + std::to_string(i) + "/position", commands[2 * i]);
      state("d" + std::to_string(i) + "/velocity", commands[2 * i + 1]);
      state("d" + std::to_string(i) + "/position", wheel_position[i]);
    }
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    executor.spin_some();
  }
  spin_until([&]() { return odometry && odometry->pose.pose.position.y > 0.1; });
  ASSERT_TRUE(odometry);
  EXPECT_NEAR(commands[0], ctrl_swerve::kPi / 2.0, 1e-9);
  EXPECT_GT(odometry->twist.twist.linear.y, 0.5);
  EXPECT_LT(std::abs(odometry->pose.pose.position.x), 0.1);
  EXPECT_EQ(odometry->header.frame_id, "odom");
  EXPECT_EQ(odometry->child_frame_id, "base_footprint");
}

TEST_F(SwerveControllerTest, SlipInflatesCovarianceAndNamesRejectedModule)
{
  nav_msgs::msg::Odometry::SharedPtr odometry;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics;
  auto odometry_subscription = client->create_subscription<nav_msgs::msg::Odometry>(
    "/rt/base/odom", robot_interfaces_qos::fast_state(),
    [&](nav_msgs::msg::Odometry::SharedPtr message) { odometry = message; });
  auto diagnostic_subscription = client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/swerve_test/diagnostics", robot_interfaces_qos::diagnostic(),
    [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) { diagnostics = message; });
  for (size_t i = 0; i < 4; ++i) {
    state("d" + std::to_string(i) + "/velocity", i == 2U ? 30.0 : 10.0);
  }

  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return odometry && diagnostics && !diagnostics->status.empty() &&
           diagnostics->status.front().message == "wheel_slip";
  });

  ASSERT_TRUE(odometry);
  ASSERT_TRUE(diagnostics);
  ASSERT_FALSE(diagnostics->status.empty());
  const auto & item = diagnostics->status.front();
  EXPECT_EQ(item.level, item.WARN);
  const auto modules = std::find_if(item.values.begin(), item.values.end(), [](const auto & value) {
    return value.key == "slipping_modules";
  });
  ASSERT_NE(modules, item.values.end());
  EXPECT_EQ(modules->value, "RL");
  EXPECT_DOUBLE_EQ(odometry->pose.covariance[0], 40.0);
}

TEST_F(SwerveControllerTest, InvalidRosCommandAndMotorFaultStopTheChassis)
{
  send(1.0);
  geometry_msgs::msg::Twist invalid;
  invalid.linear.x = std::numeric_limits<double>::quiet_NaN();
  publisher->publish(invalid);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return commands[1] == 0.0;
  });
  send(1.0);
  state("d1/status_word", 0x0008);
  expect_stopped();
}

TEST_F(SwerveControllerTest, MotionFaultLatchesAndReportsStopOnlyAfterWheelsStop)
{
  bool fault_stop = false;
  auto subscription = client->create_subscription<std_msgs::msg::Bool>(
    "/swerve_test/fault_stop", rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::Bool::SharedPtr message) { fault_stop = message->data; });
  send(1.0);
  state("d1/status_word", 0x0008);
  state("d0/velocity", 5.0);  // wheel 1 still turning: no ordered-disable request yet
  const auto spin_for = [&](int milliseconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < end) {
      controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
      executor.spin_some();
    }
  };
  spin_for(150);
  EXPECT_FALSE(fault_stop);
  state("d0/velocity", 0.0);
  spin_for(150);
  EXPECT_TRUE(fault_stop);
  // Recovered feedback and a fresh command never resume motion while latched.
  state("d1/status_word", 0x0027);
  send_without_wait(1.0);
  spin_for(50);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(commands[2 * i + 1], 0.0);
  }
}

TEST_F(SwerveControllerTest, LocalReceiveTimeoutStopsWithoutAnotherMessage)
{
  send(1.0);
  const auto expired = std::chrono::steady_clock::now() + std::chrono::milliseconds(550);
  spin_until([&]() { return std::chrono::steady_clock::now() >= expired; });
  // An ordinary timeout ramps the wheels down at the wheel acceleration limit, never upward.
  double previous = std::abs(commands[1]);
  for (int cycle = 0; cycle < 5000 && std::abs(commands[1]) > 0.0; ++cycle) {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    EXPECT_LE(std::abs(commands[1]), previous);
    previous = std::abs(commands[1]);
  }
  expect_stopped();
}

TEST_F(SwerveControllerTest, OptionalImuPublishesMeasuredYawRateAndRejectsBadOrientation)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameters(
    {{"imu_enabled", true},
     {"imu_frame_id", "base_footprint"},
     {"imu_timeout", 0.2},
     {"max_imu_yaw_step", 0.5},
     {"quaternion_norm_tolerance", 0.05}});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  nav_msgs::msg::Odometry::SharedPtr odometry;
  auto subscription = client->create_subscription<nav_msgs::msg::Odometry>(
    "/rt/base/odom", robot_interfaces_qos::fast_state(),
    [&](nav_msgs::msg::Odometry::SharedPtr message) { odometry = message; });
  auto imu_publisher = client->create_publisher<sensor_msgs::msg::Imu>(
    "/swerve_test/imu", robot_interfaces_qos::fast_state());
  spin_until([&]() { return imu_publisher->get_subscription_count() == 1; });
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_footprint";
  imu.orientation.w = 1.0;
  imu.angular_velocity.z = 0.3;
  imu_publisher->publish(imu);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return odometry && odometry->twist.twist.angular.z == 0.3;
  });
  imu.orientation.w = 0.0;
  imu_publisher->publish(imu);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return odometry && odometry->twist.twist.angular.z == 0.0;
  });
  EXPECT_DOUBLE_EQ(odometry->pose.covariance[0], 2.0);
}

TEST_F(SwerveControllerTest, RejectsLegacyMitParametersInsteadOfSilentlyIgnoringThem)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->declare_parameter<double>("steering.kp", 20.0);
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsOverflowingCovariance)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameter({"pose_covariance", std::vector<double>(6, 1e308)});
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, RejectsNonpositiveSlipThreshold)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameter({"slip_residual_threshold", 0.0});
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
}

TEST_F(SwerveControllerTest, FailedActivationCanBeCleanedUpSafely)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  bind();
  state("s0/status_word", 0x0040);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  controller.release_interfaces();
  EXPECT_EQ(controller.on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}

TEST_F(SwerveControllerTest, SteeringMotionParametersAreCappedAndAppliedOnActivation)
{
  // ELECTRI-127: operator-tunable steering motion is checked against hard caps when set.
  auto node = controller.get_node();
  EXPECT_FALSE(node->set_parameter({"max_steering_slew", 11.0}).successful);
  EXPECT_FALSE(node->set_parameter({"max_steering_jerk", 0.0}).successful);
  EXPECT_FALSE(node->set_parameter({"alignment_threshold", 2.0}).successful);
  EXPECT_TRUE(node->set_parameter({"max_steering_slew", 2.0}).successful);
  EXPECT_TRUE(node->set_parameter({"max_steering_acceleration", 4.0}).successful);
  EXPECT_TRUE(node->set_parameter({"max_steering_jerk", 10.0}).successful);
  EXPECT_TRUE(node->set_parameter({"alignment_threshold", 1.5}).successful);
  // Applied at the next activation (the console only changes them while disabled).
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  send(1.0);
  EXPECT_GT(commands[1], 0.0);
}

TEST_F(SwerveControllerTest, ExecutionFeedbackSeparatesFinalTargetAndReferenceAndPreservesUnits)
{
  configure_finite_feedback_limits();
  ASSERT_FALSE(HasFatalFailure());
  using W = ctrl_swerve::ExecutionStateWire;
  using State = std_msgs::msg::Float64MultiArray;
  State::SharedPtr execution, compatibility;
  sensor_msgs::msg::JointState::SharedPtr joints;
  auto state_sub = client->create_subscription<State>(
    "/rt/base/execution_state", robot_interfaces_qos::fast_state(),
    [&](State::SharedPtr value) { execution = value; });
  auto compat_sub = client->create_subscription<State>(
    "/rt_control/chassis/execution_state", robot_interfaces_qos::fast_state(),
    [&](State::SharedPtr value) { compatibility = value; });
  auto joint_sub = client->create_subscription<sensor_msgs::msg::JointState>(
    "/rt_control/chassis/joint_states", robot_interfaces_qos::fast_state(),
    [&](sensor_msgs::msg::JointState::SharedPtr value) { joints = value; });
  for (std::size_t i = 0; i < 4; ++i) {
    state("d" + std::to_string(i) + "/velocity", 1.25);
    state("d" + std::to_string(i) + "/position", 3.0);
  }
  send(0.0, 1.0);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return execution && compatibility && joints && execution->data[W::Ready] == 1.0 &&
           execution->data[W::Sequence] == compatibility->data[W::Sequence] &&
           execution->data[W::Modules + W::Goal] > 0.1;
  });
  ASSERT_EQ(execution->data.size(), 91U);
  EXPECT_EQ(execution->layout.dim[0].label, "rt_base_execution_state_v1");
  EXPECT_DOUBLE_EQ(execution->data[0], 1.0);
  EXPECT_NEAR(execution->data[W::Modules + W::Goal], ctrl_swerve::kPi / 2.0, 1e-9);
  EXPECT_LT(execution->data[W::Modules + W::ReferenceAngle], execution->data[W::Modules + W::Goal]);
  EXPECT_NEAR(execution->data[W::Modules + W::WheelVelocity], 0.125, 1e-12);
  for (const double value : execution->data) {
    EXPECT_TRUE(std::isfinite(value));
  }
  EXPECT_EQ(joints->name[0], "module_1_steering");
  EXPECT_EQ(joints->name[1], "module_1_wheel");
  EXPECT_DOUBLE_EQ(joints->position[1], 3.0);
  EXPECT_DOUBLE_EQ(joints->velocity[1], 1.25);
  EXPECT_TRUE(std::isnan(joints->velocity[0]));  // Fixed age is not a second hardware frame.
  for (std::size_t i = 0; i < execution->data.size(); ++i) {
    if (std::isnan(execution->data[i])) {
      EXPECT_TRUE(std::isnan(compatibility->data[i]));
    } else {
      EXPECT_DOUBLE_EQ(execution->data[i], compatibility->data[i]);
    }
  }
  state("e0/feedback_age_ms", 1000.0);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return execution->data[W::Ready] == 0.0;
  });
  EXPECT_TRUE(std::isnan(execution->data[W::Modules + W::Angle]));
}

TEST_F(SwerveControllerTest, RotationLeaseExpiresAndReactivationCannotInheritRequest)
{
  configure_finite_feedback_limits();
  ASSERT_FALSE(HasFatalFailure());
  using W = ctrl_swerve::ExecutionStateWire;
  auto request = client->create_publisher<std_msgs::msg::Bool>(
    "/rt/base/terminal_rotation_request", robot_interfaces_qos::control());
  std_msgs::msg::Float64MultiArray::SharedPtr execution;
  bool active = false;
  auto state_sub = client->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/rt/base/execution_state", robot_interfaces_qos::fast_state(),
    [&](std_msgs::msg::Float64MultiArray::SharedPtr value) { execution = value; });
  auto active_sub = client->create_subscription<std_msgs::msg::Bool>(
    "/rt/base/terminal_rotation_active", rclcpp::QoS(1).reliable().durability_volatile(),
    [&](std_msgs::msg::Bool::SharedPtr value) { active = value->data; });
  spin_until([&]() {
    return request->get_subscription_count() == 1 && publisher->get_subscription_count() == 1;
  });
  std_msgs::msg::Bool intent;
  intent.data = true;
  send_without_wait(0.0);
  request->publish(intent);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return active && execution && execution->data[W::Ready] == 1.0;
  });
  EXPECT_DOUBLE_EQ(execution->data[W::SteeringJerk], 10.0);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return !active && execution->data[W::Ready] == 1.0;
  });
  EXPECT_DOUBLE_EQ(execution->data[W::SteeringJerk], 10.0);
  send_without_wait(0.0);
  request->publish(intent);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return active && execution->data[W::Ready] == 1.0;
  });
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  spin_until([&]() { return !active && execution->data[W::Status] == 0.0; });
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  send_without_wait(0.0);
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
    return execution->data[W::Ready] == 1.0 &&
           execution->data[W::Status] == static_cast<double>(ctrl_swerve::ControlStatus::running);
  });
  EXPECT_FALSE(active);
  EXPECT_DOUBLE_EQ(execution->data[W::BodyVelocity + 2], 0.0);
}

TEST_F(SwerveControllerTest, EncoderReceiveAgeControlsObservedVelocityAcrossLifecycle)
{
  configure_finite_feedback_limits();
  ASSERT_FALSE(HasFatalFailure());
  sensor_msgs::msg::JointState::SharedPtr joints;
  auto subscription = client->create_subscription<sensor_msgs::msg::JointState>(
    "/rt_control/chassis/joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::SharedPtr message) { joints = message; });
  auto received = std::chrono::steady_clock::now();
  double angle = 0.0;
  const auto tick = [&](bool receive, bool moving) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto now = std::chrono::steady_clock::now();
    if (receive && now - received >= std::chrono::milliseconds(4)) {
      if (moving) {
        angle += 0.1 * std::chrono::duration<double>(now - received).count();
      }
      received = now;
    }
    const double age_ms = std::chrono::duration<double, std::milli>(now - received).count();
    for (std::size_t i = 0; i < 4; ++i) {
      state("e" + std::to_string(i) + "/position", angle);
      state("e" + std::to_string(i) + "/feedback_age_ms", age_ms);
    }
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.001));
  };
  spin_until([&]() {
    tick(true, false);
    return joints && std::isfinite(joints->velocity[0]);
  });
  ASSERT_TRUE(joints);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(joints->velocity[2 * i], 0.0);
  }
  spin_until([&]() {
    tick(true, true);
    return joints && std::isfinite(joints->velocity[0]) && joints->velocity[0] > 0.01;
  });
  EXPECT_GT(joints->velocity[0], 0.01);
  spin_until([&]() {
    tick(false, false);
    return joints && std::isnan(joints->velocity[0]);
  });
  EXPECT_TRUE(std::isnan(joints->velocity[0]));
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  for (std::size_t i = 0; i < 4; ++i) {
    state("e" + std::to_string(i) + "/position", 0.0);
    state("e" + std::to_string(i) + "/feedback_age_ms", 0.0);
  }
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  const auto reactivated = controller.get_node()->now().nanoseconds();
  joints.reset();
  spin_until([&]() {
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.001));
    return joints && rclcpp::Time(joints->header.stamp).nanoseconds() > reactivated;
  });
  EXPECT_TRUE(std::isnan(joints->velocity[0]));  // Cached zero-age samples remain unknown.
  angle = 0.0;
  received = std::chrono::steady_clock::now();
  spin_until([&]() {
    tick(true, false);
    return joints && std::isfinite(joints->velocity[0]);
  });
  EXPECT_DOUBLE_EQ(joints->velocity[0], 0.0);
}
