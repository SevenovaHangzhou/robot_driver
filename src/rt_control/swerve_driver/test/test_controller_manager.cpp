#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <future>
#include <iterator>
#include "controller_manager/controller_manager.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "synthetic_parameters.hpp"

class SwerveManagerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(SwerveManagerTest, SharesTheManagerAndReleasesMotionInterfacesOnDeactivate)
{
  std::ifstream input(SWERVE_MOCK_URDF);
  ASSERT_TRUE(input);
  const std::string urdf{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto hardware = std::make_unique<hardware_interface::ResourceManager>(urdf, true, true);
  auto * resources = hardware.get();
  auto options = controller_manager::get_cm_node_options();
  options.parameter_overrides({{"update_rate", 250}});
  controller_manager::ControllerManager manager(std::move(hardware), executor, "swerve_manager", "", options);
  auto controller = manager.load_controller("swerve_managed", "swerve_driver/SwerveController");
  ASSERT_TRUE(controller);
  controller->get_node()->set_parameters(synthetic_parameters());
  ASSERT_EQ(manager.configure_controller("swerve_managed"), controller_interface::return_type::OK);

  auto tick = [&]() {
      const auto time = manager.now();
      const auto period = rclcpp::Duration::from_seconds(0.004);
      manager.read(time, period);
      manager.update(time, period);
      manager.write(time, period);
      executor->spin_some();
    };
  auto change = [&](bool activate) {
      auto future = std::async(std::launch::async, [&]() {
          return manager.switch_controller(activate ? std::vector<std::string>{"swerve_managed"} : std::vector<std::string>{},
            activate ? std::vector<std::string>{} : std::vector<std::string>{"swerve_managed"},
            2, false, rclcpp::Duration::from_seconds(1.0));
        });
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
        std::chrono::steady_clock::now() < deadline) {tick();}
      EXPECT_EQ(future.get(), controller_interface::return_type::OK);
    };
  change(true);
  EXPECT_EQ(manager.get_update_rate(), 250U);
  EXPECT_TRUE(resources->command_interface_is_claimed("s0/position"));
  EXPECT_TRUE(resources->command_interface_is_claimed("d0/velocity"));
  EXPECT_FALSE(resources->command_interface_is_claimed("s0/control_word"));

  auto client = std::make_shared<rclcpp::Node>("swerve_manager_client");
  executor->add_node(client);
  auto publisher = client->create_publisher<geometry_msgs::msg::Twist>(
    "/swerve_managed/cmd_vel", robot_interfaces_qos::control());
  const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisher->get_subscription_count() != 1 && std::chrono::steady_clock::now() < discovery) {tick();}
  ASSERT_EQ(publisher->get_subscription_count(), 1U);
  auto velocity = resources->claim_state_interface("d0/velocity");
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  publisher->publish(command);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!(velocity.get_value() > 0.0) && std::chrono::steady_clock::now() < deadline) {tick();}
  EXPECT_GT(velocity.get_value(), 0.0);

  change(false);
  EXPECT_FALSE(resources->command_interface_is_claimed("s0/position"));
  EXPECT_FALSE(resources->command_interface_is_claimed("d0/velocity"));
  auto drive_command = resources->claim_command_interface("d0/velocity");
  EXPECT_DOUBLE_EQ(drive_command.get_value(), 0.0);
}
