#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "rt_force_torque_broadcaster/force_torque_broadcaster.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace
{

using Broadcaster = rt_force_torque_broadcaster::ForceTorqueBroadcaster;
using CallbackReturn = controller_interface::CallbackReturn;

std::vector<rclcpp::Parameter> parameters(bool calibration_valid = true)
{
  return {
    {"sensor_name", "right_force_sensor"},
    {"frame_id", "right_ft_sensor_link"},
    {"value_interfaces", std::vector<std::string>{
        "right_force_sensor/channel_1_raw", "right_force_sensor/channel_2_raw",
        "right_force_sensor/channel_3_raw", "right_force_sensor/channel_4_raw",
        "right_force_sensor/channel_5_raw", "right_force_sensor/channel_6_raw"}},
    {"auxiliary_interfaces", std::vector<std::string>{
        "right_force_sensor/sample_code_1_raw", "right_force_sensor/sample_code_2_raw",
        "right_force_sensor/sample_code_3_raw", "right_force_sensor/sample_code_4_raw",
        "right_force_sensor/sample_code_5_raw", "right_force_sensor/sample_code_6_raw"}},
    {"scale_factors", std::vector<double>{0.1, 0.1, 0.1, 0.001, 0.001, 0.001}},
    {"validity_policy", "all_exact_in_range"},
    {"minimum_auxiliary_value", -999999},
    {"maximum_auxiliary_value", 999999},
    {"calibration_valid", calibration_valid},
    {"startup_id", "run-1"},
    {"snapshot_source", "preop_sdo"},
    {"decimals", std::vector<std::int64_t>{1, 1, 1, 3, 3, 3}},
    {"unit_codes", std::vector<std::int64_t>{5, 5, 5, 7, 7, 7}},
    {"wrench_topic", "/test/wrench"},
    {"raw_topic", "/test/raw"},
    {"calibration_topic", "/test/calibration"},
    {"diagnostic_name", "/robot/rt_control/x503b/right_force_sensor/calibration"},
    {"link_interface", "ethercat_master/link_up"},
    {"al_state_interface", "ethercat_slave_14/al_state"},
  };
}

class ForceTorqueBroadcasterTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  Broadcaster controller;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr client;
  std::array<double, 14> state_values{};
  std::vector<hardware_interface::StateInterface> state_handles;

  void configure(const bool calibration_valid = true)
  {
    ASSERT_EQ(controller.init("ft_test"), controller_interface::return_type::OK);
    controller.get_node()->set_parameters(parameters(calibration_valid));
    ASSERT_EQ(
      controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  void bind_and_activate()
  {
    const auto names = controller.state_interface_configuration().names;
    ASSERT_EQ(names.size(), state_values.size());
    state_handles.clear();
    for (std::size_t index = 0U; index < names.size(); ++index) {
      const auto separator = names[index].find_last_of('/');
      ASSERT_NE(separator, std::string::npos);
      state_handles.emplace_back(
        names[index].substr(0, separator), names[index].substr(separator + 1),
        &state_values[index]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned;
    for (auto & handle : state_handles) {
      loaned.emplace_back(handle);
    }
    controller.assign_interfaces({}, std::move(loaned));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  }

  void set_state(const std::string & name, const double value)
  {
    const auto names = controller.state_interface_configuration().names;
    const auto iterator = std::find(names.begin(), names.end(), name);
    ASSERT_NE(iterator, names.end());
    state_values[static_cast<std::size_t>(iterator - names.begin())] = value;
  }

  void set_valid_frame()
  {
    for (std::size_t index = 0U; index < 6U; ++index) {
      set_state(
        "right_force_sensor/channel_" + std::to_string(index + 1U) + "_raw",
        index < 3U ? 100.0 * static_cast<double>(index + 1U) :
        1000.0 * static_cast<double>(index - 2U));
      set_state(
        "right_force_sensor/sample_code_" + std::to_string(index + 1U) + "_raw",
        static_cast<double>(index));
    }
    set_state("ethercat_master/link_up", 1.0);
    set_state("ethercat_slave_14/al_state", 8.0);
  }

  void spin_until(const std::function<bool()> & complete)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!complete() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
    }
    ASSERT_TRUE(complete());
  }

  void update_once()
  {
    ASSERT_EQ(
      controller.update(
        controller.get_node()->now(), rclcpp::Duration::from_seconds(0.001)),
      controller_interface::return_type::OK);
  }

  void SetUp() override
  {
    configure();
    if (HasFatalFailure()) {
      return;
    }
    bind_and_activate();
    if (HasFatalFailure()) {
      return;
    }
    executor.add_node(controller.get_node()->get_node_base_interface());
    client = std::make_shared<rclcpp::Node>("ft_test_client");
    executor.add_node(client);
  }

  void TearDown() override
  {
    controller.on_deactivate(rclcpp_lifecycle::State{});
  }
};

TEST_F(ForceTorqueBroadcasterTest, ClaimsOnlyTheConfiguredStateInterfaces)
{
  EXPECT_EQ(
    controller.command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);
  const auto names = controller.state_interface_configuration().names;
  ASSERT_EQ(names.size(), 14U);
  EXPECT_EQ(names.front(), "right_force_sensor/channel_1_raw");
  EXPECT_EQ(names[11], "right_force_sensor/sample_code_6_raw");
  EXPECT_EQ(names[12], "ethercat_master/link_up");
  EXPECT_EQ(names[13], "ethercat_slave_14/al_state");
}

TEST_F(ForceTorqueBroadcasterTest, PublishesCompatibleRawWrenchAndCalibrationTopics)
{
  std_msgs::msg::Int32MultiArray::SharedPtr raw;
  geometry_msgs::msg::WrenchStamped::SharedPtr wrench;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr calibration;
  const auto raw_subscription = client->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/test/raw", robot_interfaces_qos::fast_state(),
    [&](std_msgs::msg::Int32MultiArray::SharedPtr message) {raw = message;});
  const auto wrench_subscription = client->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test/wrench", robot_interfaces_qos::fast_state(),
    [&](geometry_msgs::msg::WrenchStamped::SharedPtr message) {wrench = message;});
  const auto calibration_subscription =
    client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/test/calibration", robot_interfaces_qos::latched(),
    [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      calibration = message;
    });
  spin_until(
    [&]() {
      return raw_subscription->get_publisher_count() > 0U &&
      wrench_subscription->get_publisher_count() > 0U &&
      calibration_subscription->get_publisher_count() > 0U;
    });
  set_valid_frame();
  update_once();

  spin_until([&]() {return raw && wrench && calibration;});

  ASSERT_EQ(raw->data, (std::vector<std::int32_t>{100, 200, 300, 1000, 2000, 3000}));
  EXPECT_EQ(wrench->header.frame_id, "right_ft_sensor_link");
  EXPECT_DOUBLE_EQ(wrench->wrench.force.x, 10.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.y, 20.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.force.z, 30.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.x, 1.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.y, 2.0);
  EXPECT_DOUBLE_EQ(wrench->wrench.torque.z, 3.0);
  ASSERT_EQ(calibration->status.size(), 1U);
  EXPECT_EQ(
    calibration->status.front().name,
    "/robot/rt_control/x503b/right_force_sensor/calibration");
  EXPECT_EQ(calibration->status.front().hardware_id, "right_force_sensor");
  EXPECT_EQ(calibration->status.front().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ForceTorqueBroadcasterTest, RecoveredLinkNeverReusesInvalidatedCalibration)
{
  std::size_t raw_count{0U};
  std::size_t wrench_count{0U};
  const auto raw_subscription = client->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/test/raw", robot_interfaces_qos::fast_state(),
    [&](std_msgs::msg::Int32MultiArray::SharedPtr) {++raw_count;});
  const auto wrench_subscription = client->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test/wrench", robot_interfaces_qos::fast_state(),
    [&](geometry_msgs::msg::WrenchStamped::SharedPtr) {++wrench_count;});
  spin_until(
    [&]() {
      return raw_subscription->get_publisher_count() > 0U &&
      wrench_subscription->get_publisher_count() > 0U;
    });
  set_valid_frame();
  update_once();
  spin_until([&]() {return raw_count >= 1U && wrench_count >= 1U;});

  set_state("ethercat_slave_14/al_state", 2.0);
  update_once();
  spin_until([&]() {return raw_count >= 2U;});
  set_state("ethercat_slave_14/al_state", 8.0);
  update_once();
  spin_until([&]() {return raw_count >= 3U;});
  EXPECT_EQ(wrench_count, 1U);
}

TEST_F(ForceTorqueBroadcasterTest, InvalidCalibrationPublishesRawButNotWrench)
{
  controller.on_deactivate(rclcpp_lifecycle::State{});
  controller.get_node()->set_parameter({"calibration_valid", false});
  ASSERT_EQ(
    controller.on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  bind_and_activate();
  set_valid_frame();

  std_msgs::msg::Int32MultiArray::SharedPtr raw;
  geometry_msgs::msg::WrenchStamped::SharedPtr wrench;
  const auto raw_subscription = client->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/test/raw", robot_interfaces_qos::fast_state(),
    [&](std_msgs::msg::Int32MultiArray::SharedPtr message) {raw = message;});
  const auto wrench_subscription = client->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test/wrench", robot_interfaces_qos::fast_state(),
    [&](geometry_msgs::msg::WrenchStamped::SharedPtr message) {wrench = message;});

  spin_until(
    [&]() {
      return raw_subscription->get_publisher_count() > 0U &&
      wrench_subscription->get_publisher_count() > 0U;
    });
  update_once();
  spin_until([&]() {return raw != nullptr;});
  EXPECT_FALSE(wrench);
}

TEST_F(ForceTorqueBroadcasterTest, PluginIsDiscoverable)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(
    loader.createSharedInstance(
      "rt_force_torque_broadcaster/ForceTorqueBroadcaster"));
}

}  // namespace
