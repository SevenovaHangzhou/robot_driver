#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_support.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <gtest/gtest.h>
#include <iterator>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

class GravityManagerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

namespace
{
std::string yaml_quote(const std::string & input)
{
  std::string output{"\""};
  for (const char value : input) {
    if (value == '\\' || value == '\"') {
      output.push_back('\\');
      output.push_back(value);
    } else if (value == '\n') {
      output += "\\n";
    } else if (value == '\r') {
      output += "\\r";
    } else {
      output.push_back(value);
    }
  }
  output.push_back('\"');
  return output;
}

std::filesystem::path write_parameters(const std::string & urdf)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("electri136-" + std::to_string(getpid()) + ".yaml");
  std::ofstream output(path);
  const auto hash = test_sha256(urdf);
  output
    << "whole_body_jtc:\n  ros__parameters:\n"
    "    joints: [joint1, joint2]\n    command_interfaces: [position]\n"
    "    state_interfaces: [position]\n    allow_partial_joints_goal: "
    "false\n"
    "    open_loop_control: false\n"
    "    set_last_command_interface_value_as_state_on_activation: false\n"
    "enable_manager:\n  ros__parameters:\n"
    "    managed_joints: [joint1, joint2]\n"
    "    enable_batch_joint_names: [joint1, joint2]\n"
    "    enable_batch_sizes: [2]\n"
    "    disable_terminal_policy: switch_on_disabled\n    enable_only: "
    "true\n    jtc_name: \"\"\n";
  const auto gravity = [&](const std::string & name, const bool active) {
      output << name
             << ":\n  ros__parameters:\n"
        "    joints: [joint1, joint2]\n    mode: "
             << (active ? "active" : "shadow")
             << "\n"
        "    scale: [1.0, 1.0]\n    max_effort_nm: [100.0, 100.0]\n"
        "    max_slew_nm_per_s: [100.0, 100.0]\n    robot_description: "
             << yaml_quote(urdf)
             << "\n"
        "    input_inertia_g_mm2: [1000.0, 2000.0]\n    reduction_ratio: "
        "[10.0, 20.0]\n"
        "    payload_frame: tool\n    model_validation.verified: "
             << (active ? "true" : "false")
             << "\n"
        "    model_validation.urdf_sha256: "
             << (active ? hash : "TBD")
             << "\n"
        "    model_validation.source: synthetic_fixture\n"
        "    effort_calibration.verified: ["
             << (active ? "true, true" : "false, false")
             << "]\n"
        "    effort_calibration.permille_per_newton_metre: [2.0, 3.0]\n"
        "    effort_calibration.source: [synthetic, synthetic]\n";
    };
  gravity("right_gravity_ff", true);
  gravity("left_gravity_shadow", false);
  return path;
}
} // namespace

TEST_F(
  GravityManagerTest,
  SharesIndependentCommandInterfacesWithJtcAndEnableManager) {
  std::ifstream input(GRAVITY_MOCK_URDF);
  ASSERT_TRUE(input);
  const std::string urdf{std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
  const auto parameter_file = write_parameters(urdf);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto hardware =
    std::make_unique<hardware_interface::ResourceManager>(urdf, true, true);
  auto * resources = hardware.get();
  auto options = controller_manager::get_cm_node_options();
  options.arguments({"--ros-args", "--params-file", parameter_file.string()});
  options.parameter_overrides({{"update_rate", 250}});
  controller_manager::ControllerManager manager(std::move(hardware), executor,
    "gravity_manager", "", options);
  for (const auto & controller : {"whole_body_jtc", "enable_manager",
      "right_gravity_ff", "left_gravity_shadow"})
  {
    const auto result = manager.set_parameter(
      rclcpp::Parameter(
        std::string(controller) + ".params_file", parameter_file.string()));
    ASSERT_TRUE(result.successful) << result.reason;
  }

  const std::vector<std::string> joints{"joint1", "joint2"};
  auto jtc = manager.load_controller(
    "whole_body_jtc",
    "joint_trajectory_controller/JointTrajectoryController");
  auto enable = manager.load_controller(
    "enable_manager", "enable_manager/EnableManagerController");
  auto active = manager.load_controller(
    "right_gravity_ff", "gravity_ff_controller/GravityFeedforwardController");
  auto shadow = manager.load_controller(
    "left_gravity_shadow",
    "gravity_ff_controller/GravityFeedforwardController");
  ASSERT_TRUE(jtc);
  ASSERT_TRUE(enable);
  ASSERT_TRUE(active);
  ASSERT_TRUE(shadow);
  ASSERT_EQ(
    manager.configure_controller("whole_body_jtc"),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    manager.configure_controller("enable_manager"),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    manager.configure_controller("right_gravity_ff"),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    manager.configure_controller("left_gravity_shadow"),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    shadow->command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);

  auto tick = [&]() {
      const auto time = manager.now();
      const auto period = rclcpp::Duration::from_seconds(0.004);
      manager.read(time, period);
      manager.update(time, period);
      manager.write(time, period);
      executor->spin_some();
    };
  auto change = [&](const std::vector<std::string> & activate,
      const std::vector<std::string> & deactivate) {
      auto future = std::async(
        std::launch::async, [&]() {
          return manager.switch_controller(
            activate, deactivate, 2, false,
            rclcpp::Duration::from_seconds(1.0));
        });
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (future.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready &&
        std::chrono::steady_clock::now() < deadline)
      {
        tick();
      }
      ASSERT_EQ(
        future.wait_for(std::chrono::seconds(0)),
        std::future_status::ready);
      EXPECT_EQ(future.get(), controller_interface::return_type::OK);
    };
  change(
    {"whole_body_jtc", "enable_manager", "right_gravity_ff",
      "left_gravity_shadow"},
    {});
  for (const auto & joint : joints) {
    EXPECT_TRUE(resources->command_interface_is_claimed(joint + "/position"));
    EXPECT_TRUE(
      resources->command_interface_is_claimed(joint + "/control_word"));
    EXPECT_TRUE(resources->command_interface_is_claimed(joint + "/effort"));
  }
  change({}, {"right_gravity_ff"});
  for (const auto & joint : joints) {
    EXPECT_TRUE(resources->command_interface_is_claimed(joint + "/position"));
    EXPECT_TRUE(
      resources->command_interface_is_claimed(joint + "/control_word"));
    EXPECT_FALSE(resources->command_interface_is_claimed(joint + "/effort"));
  }
  std::filesystem::remove(parameter_file);
}
