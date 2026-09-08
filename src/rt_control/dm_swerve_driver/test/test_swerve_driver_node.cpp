#include <gtest/gtest.h>

#include <memory>
#include <chrono>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dm_swerve_driver/swerve_driver_node.hpp"
#include "fake_can_transport.hpp"

namespace dm_swerve_driver {
namespace {

class SwerveDriverNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc{0};
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(SwerveDriverNodeTest, LifecycleStartsAndStopsInjectedControlLoop)
{
  test::FakeCanTransport * transport_observer{nullptr};
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [&](const DriverParameters &) {
      auto transport = std::make_unique<test::FakeCanTransport>();
      transport_observer = transport.get();
      return transport;
    });

  EXPECT_EQ(node->get_current_state().label(), "unconfigured");
  EXPECT_EQ(node->configure().label(), "inactive");
  EXPECT_EQ(node->activate().label(), "active");
  ASSERT_NE(transport_observer, nullptr);
  EXPECT_TRUE(transport_observer->is_open());
  EXPECT_TRUE(node->control_status().running);

  EXPECT_EQ(node->deactivate().label(), "inactive");
  EXPECT_FALSE(node->control_status().running);
}

TEST_F(SwerveDriverNodeTest, TriggerServicesControlAndMaintainLifecycleState)
{
  std::size_t transport_count{0U};
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [&](const DriverParameters &) {
      ++transport_count;
      return std::make_unique<test::FakeCanTransport>();
    });
  auto client_node = std::make_shared<rclcpp::Node>("swerve_service_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_EQ(node->configure().label(), "inactive");
  ASSERT_EQ(node->activate().label(), "active");

  const auto call = [&](const std::string & service_name) {
      auto client = client_node->create_client<std_srvs::srv::Trigger>(service_name);
      if (!client->wait_for_service(std::chrono::milliseconds{200})) {
        ADD_FAILURE() << "service unavailable: " << service_name;
        return std::make_shared<std_srvs::srv::Trigger::Response>();
      }
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (executor.spin_until_future_complete(future, std::chrono::seconds{1}) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        ADD_FAILURE() << "service call timed out: " << service_name;
        return std::make_shared<std_srvs::srv::Trigger::Response>();
      }
      return future.get();
    };

  EXPECT_TRUE(call("/swerve_driver/clear_faults")->success);
  EXPECT_FALSE(call("/swerve_driver/rezero_steering")->success);
  EXPECT_TRUE(call("/swerve_driver/disable")->success);
  EXPECT_FALSE(node->control_status().running);
  EXPECT_TRUE(call("/swerve_driver/rezero_steering")->success);
  EXPECT_TRUE(call("/swerve_driver/enable")->success);
  EXPECT_TRUE(node->control_status().running);
  EXPECT_GE(transport_count, 3U);

  EXPECT_EQ(node->deactivate().label(), "inactive");
  executor.remove_node(client_node);
  executor.remove_node(node->get_node_base_interface());
}

TEST_F(SwerveDriverNodeTest, EnableServiceContainsTransportFactoryExceptions)
{
  std::size_t transport_count{0U};
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [&](const DriverParameters &) -> std::unique_ptr<CanTransport> {
      ++transport_count;
      if (transport_count == 2U) {
        throw std::runtime_error{"injected factory failure"};
      }
      if (transport_count == 3U) {
        throw 7;
      }
      return std::make_unique<test::FakeCanTransport>();
    });
  auto client_node = std::make_shared<rclcpp::Node>("swerve_exception_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_EQ(node->configure().label(), "inactive");
  ASSERT_EQ(node->activate().label(), "active");

  const auto call = [&](const std::string & service_name) {
      auto client = client_node->create_client<std_srvs::srv::Trigger>(service_name);
      if (!client->wait_for_service(std::chrono::milliseconds{200})) {
        throw std::runtime_error{"service unavailable"};
      }
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (executor.spin_until_future_complete(future, std::chrono::seconds{1}) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error{"service timeout"};
      }
      return future.get();
    };

  ASSERT_TRUE(call("/swerve_driver/disable")->success);
  std::shared_ptr<std_srvs::srv::Trigger::Response> standard_failure;
  EXPECT_NO_THROW(standard_failure = call("/swerve_driver/enable"));
  ASSERT_NE(standard_failure, nullptr);
  EXPECT_FALSE(standard_failure->success);
  EXPECT_FALSE(standard_failure->message.empty());

  std::shared_ptr<std_srvs::srv::Trigger::Response> unknown_failure;
  EXPECT_NO_THROW(unknown_failure = call("/swerve_driver/enable"));
  ASSERT_NE(unknown_failure, nullptr);
  EXPECT_FALSE(unknown_failure->success);
  EXPECT_FALSE(node->control_status().running);

  EXPECT_EQ(node->deactivate().label(), "inactive");
  executor.remove_node(client_node);
  executor.remove_node(node->get_node_base_interface());
}

TEST_F(SwerveDriverNodeTest, RezeroServiceListsMotorThatFailsAcknowledgement)
{
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [](const DriverParameters &) {
      auto transport = std::make_unique<test::FakeCanTransport>();
      transport->set_dropped_mst_id(0x12U);
      return transport;
    });
  auto client_node = std::make_shared<rclcpp::Node>("swerve_rezero_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_EQ(node->configure().label(), "inactive");
  auto client = client_node->create_client<std_srvs::srv::Trigger>(
    "/swerve_driver/rezero_steering");
  ASSERT_TRUE(client->wait_for_service(std::chrono::milliseconds{200}));

  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds{1}),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto response = future.get();

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("ESC_ID 2"), std::string::npos);
  executor.remove_node(client_node);
  executor.remove_node(node->get_node_base_interface());
}

TEST_F(SwerveDriverNodeTest, StrictStartupFailureRejectsLifecycleActivation)
{
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [](const DriverParameters &) {
      auto transport = std::make_unique<test::FakeCanTransport>();
      transport->set_register_replies_enabled(false);
      return transport;
    });

  ASSERT_EQ(node->configure().label(), "inactive");
  EXPECT_NE(node->activate().label(), "active");
  EXPECT_FALSE(node->control_status().running);
}

TEST_F(SwerveDriverNodeTest, DisableEnableCannotBypassLatchedFault)
{
  std::size_t transport_count{0U};
  auto node = std::make_shared<SwerveDriverNode>(
    rclcpp::NodeOptions{},
    [&](const DriverParameters &) {
      auto transport = std::make_unique<test::FakeCanTransport>();
      if (++transport_count == 1U) {
        transport->override_next_error(0x11U, MotorError::over_current);
      }
      return transport;
    });
  auto client_node = std::make_shared<rclcpp::Node>("swerve_latch_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_EQ(node->configure().label(), "inactive");
  ASSERT_EQ(node->activate().label(), "active");

  for (int attempt{0}; attempt < 50 && !node->control_status().fault_latched; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  ASSERT_TRUE(node->control_status().fault_latched);

  const auto call = [&](const std::string & service_name) {
      auto client = client_node->create_client<std_srvs::srv::Trigger>(service_name);
      if (!client->wait_for_service(std::chrono::milliseconds{200})) {
        throw std::runtime_error{"service unavailable"};
      }
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (executor.spin_until_future_complete(future, std::chrono::seconds{1}) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error{"service timeout"};
      }
      return future.get();
    };

  ASSERT_TRUE(call("/swerve_driver/disable")->success);
  ASSERT_TRUE(call("/swerve_driver/enable")->success);
  EXPECT_TRUE(node->control_status().fault_latched);

  ASSERT_TRUE(call("/swerve_driver/clear_faults")->success);
  for (int attempt{0}; attempt < 50 && node->control_status().fault_latched; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  EXPECT_FALSE(node->control_status().fault_latched);

  EXPECT_EQ(node->deactivate().label(), "inactive");
  executor.remove_node(client_node);
  executor.remove_node(node->get_node_base_interface());
}

}  // namespace
}  // namespace dm_swerve_driver
