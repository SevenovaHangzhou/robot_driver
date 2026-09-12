#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "swerve_driver/swerve_controller.hpp"
#include "synthetic_parameters.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using Controller = swerve_driver::SwerveController;

class SwerveControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
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
    for (auto & handle : command_handles) {c_loaned.emplace_back(handle);}
    for (auto & handle : state_handles) {s_loaned.emplace_back(handle);}
    controller.assign_interfaces(std::move(c_loaned), std::move(s_loaned));
  }

  void SetUp() override
  {
    configure();
    if (HasFatalFailure()) {return;}
    const auto names = controller.state_interface_configuration().names;
    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i].find("status_word") != std::string::npos) {states[i] = 0x0027;}
      if (names[i].find("mode_of_operation_display") != std::string::npos) {
        states[i] = names[i][0] == 's' ? 8.0 : 9.0;
      }
    }
    bind();
    if (HasFatalFailure()) {return;}
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    executor.add_node(controller.get_node()->get_node_base_interface());
    client = std::make_shared<rclcpp::Node>("swerve_test_client");
    executor.add_node(client);
    publisher = client->create_publisher<geometry_msgs::msg::Twist>(
      "/swerve_test/cmd_vel", robot_interfaces_qos::control());
  }

  void TearDown() override {controller.on_deactivate(rclcpp_lifecycle::State{});}

  void spin_until(std::function<bool()> done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!done() && std::chrono::steady_clock::now() < deadline) {executor.spin_some();}
    ASSERT_TRUE(done());
  }

  void send(double vx, double vy = 0.0)
  {
    spin_until([&]() {return publisher->get_subscription_count() == 1;});
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    publisher->publish(msg);
    spin_until([&]() {
      controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.004));
      return std::abs(commands[1]) > 0.0 || std::abs(commands[0]) > 0.0;
    });
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
    for (size_t i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(commands[2 * i + 1], 0.0);}
  }
};

TEST_F(SwerveControllerTest, ClaimsEightMotionInterfacesAndNoControlWord)
{
  const auto names = controller.command_interface_configuration().names;
  EXPECT_EQ(names[0], "s0/position");
  EXPECT_EQ(names[1], "d0/velocity");
  for (const auto & name : names) {EXPECT_EQ(name.find("control_word"), std::string::npos);}
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

TEST_F(SwerveControllerTest, EncoderLossStopsWithoutReplayingOldCommandOnRecovery)
{
  send(1.0);
  state("e2/feedback_age_ms", 100.0);
  expect_stopped();
  state("e2/feedback_age_ms", 0.0);
  expect_stopped();
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
  EXPECT_TRUE(loader.createSharedInstance("swerve_driver/SwerveController"));
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
  spin_until([&]() {return publisher->get_subscription_count() == 1;});
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 1.0;
  publisher->publish(msg);
  ASSERT_TRUE(publisher->wait_for_all_acked(std::chrono::seconds(1)));
  controller.on_deactivate(rclcpp_lifecycle::State{});
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  for (int i = 0; i < 10; ++i) {executor.spin_some(); expect_stopped();}
}

TEST_F(SwerveControllerTest, LateralCommandTraversesInterfacesAndProducesMeasuredOdometry)
{
  nav_msgs::msg::Odometry::SharedPtr odometry;
  auto subscription = client->create_subscription<nav_msgs::msg::Odometry>(
    "/swerve_test/odom", robot_interfaces_qos::fast_state(),
    [&](nav_msgs::msg::Odometry::SharedPtr message) {odometry = message;});
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
  spin_until([&]() {return odometry && odometry->pose.pose.position.y > 0.1;});
  ASSERT_TRUE(odometry);
  EXPECT_NEAR(commands[0], swerve_driver::kPi / 2.0, 1e-9);
  EXPECT_GT(odometry->twist.twist.linear.y, 0.5);
  EXPECT_LT(std::abs(odometry->pose.pose.position.x), 0.1);
  EXPECT_EQ(odometry->header.frame_id, "odom");
  EXPECT_EQ(odometry->child_frame_id, "base_footprint");
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

TEST_F(SwerveControllerTest, LocalReceiveTimeoutStopsWithoutAnotherMessage)
{
  send(1.0);
  const auto expired = std::chrono::steady_clock::now() + std::chrono::milliseconds(550);
  spin_until([&]() {return std::chrono::steady_clock::now() >= expired;});
  expect_stopped();
}

TEST_F(SwerveControllerTest, OptionalImuPublishesMeasuredYawRateAndRejectsBadOrientation)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameters({
    {"imu_enabled", true}, {"imu_frame_id", "base_footprint"},
    {"imu_timeout", 0.2}, {"max_imu_yaw_step", 0.5}, {"quaternion_norm_tolerance", 0.05}});
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  nav_msgs::msg::Odometry::SharedPtr odometry;
  auto subscription = client->create_subscription<nav_msgs::msg::Odometry>(
    "/swerve_test/odom", robot_interfaces_qos::fast_state(),
    [&](nav_msgs::msg::Odometry::SharedPtr message) {odometry = message;});
  auto imu_publisher = client->create_publisher<sensor_msgs::msg::Imu>(
    "/swerve_test/imu", robot_interfaces_qos::fast_state());
  spin_until([&]() {return imu_publisher->get_subscription_count() == 1;});
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

TEST_F(SwerveControllerTest, FailedActivationCanBeCleanedUpSafely)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  bind();
  state("s0/status_word", 0x0040);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  controller.release_interfaces();
  EXPECT_EQ(controller.on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
}
