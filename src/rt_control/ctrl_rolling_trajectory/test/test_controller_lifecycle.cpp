#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rolling_test_joints.hpp"
#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"
#include "ctrl_rolling_trajectory/rolling_types.hpp"

namespace ctrl_rolling_trajectory
{
namespace
{

constexpr double kTakeoverTolerance = 0.125;

std::vector<rclcpp::Parameter> makeValidParameters()
{
  return testControllerParameters();
}

void replaceParameter(
  std::vector<rclcpp::Parameter> & parameters, const rclcpp::Parameter & replacement)
{
  const auto existing =
    std::find_if(parameters.begin(), parameters.end(), [&replacement](const auto & parameter) {
      return parameter.get_name() == replacement.get_name();
    });
  if (existing == parameters.end()) {
    parameters.push_back(replacement);
  } else {
    *existing = replacement;
  }
}

class ControllerLifecycleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      actual_positions_[axis] = static_cast<double>(axis) * 0.25;
      command_positions_[axis] = actual_positions_[axis] + 0.01;
      command_handles_.emplace_back(kTestJointNames[axis], "position", &command_positions_[axis]);
      state_handles_.emplace_back(kTestJointNames[axis], "position", &actual_positions_[axis]);
    }
    feedback_age_ms_ = 12.0;
    state_handles_.emplace_back("ethercat_domain", "process_data_age_ms", &feedback_age_ms_);

    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(makeValidParameters());
    controller_ = std::make_unique<RollingTrajectoryController>();
    ASSERT_EQ(
      controller_->init("test_rolling_controller", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
    controller_.reset();
  }

  void assignInterfaces(
    std::size_t command_count = kAxisCount, std::size_t state_count = kAxisCount + 1U)
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.reserve(command_count);
    states.reserve(state_count);
    for (std::size_t axis = 0U; axis < command_count; ++axis) {
      commands.emplace_back(command_handles_[axis]);
    }
    for (std::size_t axis = 0U; axis < state_count; ++axis) {
      states.emplace_back(state_handles_[axis]);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
  }

  control_msgs::srv::QueryTrajectoryState::Response::SharedPtr queryHeld(
    std::int32_t seconds = 0)
  {
    using Query = control_msgs::srv::QueryTrajectoryState;
    auto client_node = std::make_shared<rclcpp::Node>("hold_query_client");
    auto client = client_node->create_client<Query>("/test_rolling_controller/query_state");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller_->get_node()->get_node_base_interface());
    executor.add_node(client_node);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      ADD_FAILURE() << "hold query service unavailable";
      return nullptr;
    }
    auto request = std::make_shared<Query::Request>();
    request->time.sec = seconds;
    auto future = client->async_send_request(request);
    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      ADD_FAILURE() << "hold query timed out";
      return nullptr;
    }
    return future.get();
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(ControllerLifecycleTest, ClaimsFourteenPositionPairsAndTheSharedFeedbackAge)
{
  const auto command_configuration = controller_->command_interface_configuration();
  const auto state_configuration = controller_->state_interface_configuration();
  ASSERT_EQ(
    command_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(
    state_configuration.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(command_configuration.names.size(), kAxisCount);
  ASSERT_EQ(state_configuration.names.size(), kAxisCount + 1U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_EQ(command_configuration.names[axis], kTestJointNames[axis] + "/position");
    EXPECT_EQ(state_configuration.names[axis], kTestJointNames[axis] + "/position");
  }
  EXPECT_EQ(state_configuration.names.back(), "ethercat_domain/process_data_age_ms");
}

TEST_F(ControllerLifecycleTest, ConfigurationRequiresEveryParameterFromTheArtifact)
{
  RollingTrajectoryController implicit;
  ASSERT_EQ(implicit.init("implicit_rolling_controller"), controller_interface::return_type::OK);
  EXPECT_EQ(
    implicit.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, RuntimeParametersAreFrozenAfterConfigure)
{
  const auto result =
    controller_->get_node()->set_parameter(rclcpp::Parameter("buffer_capacity", std::int64_t{32}));
  EXPECT_FALSE(result.successful);
}

TEST_F(ControllerLifecycleTest, InvalidRuntimeParameterMatrixFailsConfigureWithoutClamping)
{
  std::vector<std::string> short_joints(kTestJointNames.begin(), kTestJointNames.end() - 1);
  std::vector<std::string> duplicate_joints(kTestJointNames.begin(), kTestJointNames.end());
  duplicate_joints.back() = duplicate_joints.front();
  const std::vector<rclcpp::Parameter> invalid_parameters = {
    rclcpp::Parameter("joints", short_joints),
    rclcpp::Parameter("joints", duplicate_joints),
    rclcpp::Parameter("joint_position_lower", std::vector<double>(kAxisCount, 10.0)),
    rclcpp::Parameter("joint_velocity_limits", std::vector<double>(kAxisCount - 1U, 1.0)),
    rclcpp::Parameter("joint_acceleration_limits", std::vector<double>(kAxisCount, 0.0)),
    rclcpp::Parameter("position_margin", -0.01),
    rclcpp::Parameter("position_margin", 10.0),
    rclcpp::Parameter("buffer_capacity", std::int64_t{1}),
    rclcpp::Parameter("buffer_capacity", std::int64_t{257}),
    rclcpp::Parameter("replace_lead_ms", std::int64_t{0}),
    rclcpp::Parameter("replace_lead_ms", std::int64_t{601}),
    rclcpp::Parameter("state_publish_period_ms", std::int64_t{0}),
    rclcpp::Parameter("nominal_controller_period_ms", std::int64_t{0}),
    rclcpp::Parameter("maximum_controller_period_ms", std::int64_t{3}),
    rclcpp::Parameter("update_timeout_ms", std::int64_t{7}),
    rclcpp::Parameter("one_cycle_detection_guard_ms", std::int64_t{0}),
    rclcpp::Parameter("feedback_age_limit_ms", std::numeric_limits<double>::quiet_NaN()),
    rclcpp::Parameter("takeover_tolerances", std::vector<double>(kAxisCount - 1U, 0.1))};

  for (std::size_t index = 0U; index < invalid_parameters.size(); ++index) {
    auto parameters = makeValidParameters();
    replaceParameter(parameters, invalid_parameters[index]);
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(parameters);
    RollingTrajectoryController invalid;
    ASSERT_EQ(
      invalid.init("invalid_runtime_configuration_" + std::to_string(index), "", options),
      controller_interface::return_type::OK)
      << invalid_parameters[index].get_name();
    EXPECT_EQ(
      invalid.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::ERROR)
      << invalid_parameters[index].get_name();
  }
}

TEST_F(ControllerLifecycleTest, ActivationRequiresEveryClaimedInterface)
{
  assignInterfaces(kAxisCount, kAxisCount);
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, FirstUpdatePreservesThePersistedSourceCommandExactly)
{
  assignInterfaces();
  const auto source_commands = command_positions_;
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(command_positions_, source_commands);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000)),
    controller_interface::return_type::OK);
  EXPECT_EQ(command_positions_, source_commands);
}

TEST_F(ControllerLifecycleTest, NonFiniteSourceCommandFailsWithoutPartialWrites)
{
  command_positions_[6] = std::numeric_limits<double>::quiet_NaN();
  assignInterfaces();
  const auto before = command_positions_;
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (axis == 6U) {
      EXPECT_TRUE(std::isnan(command_positions_[axis]));
    } else {
      EXPECT_DOUBLE_EQ(command_positions_[axis], before[axis]);
    }
  }
}

TEST_F(ControllerLifecycleTest, NonFiniteActualPositionFailsActivation)
{
  actual_positions_[4] = std::numeric_limits<double>::infinity();
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, ExactTakeoverTolerancePasses)
{
  command_positions_[3] = actual_positions_[3] + kTakeoverTolerance;
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(ControllerLifecycleTest, OneRepresentableStepPastTakeoverToleranceFails)
{
  command_positions_[3] = std::nextafter(
    actual_positions_[3] + kTakeoverTolerance, std::numeric_limits<double>::infinity());
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, DeactivationDoesNotRewriteTheHeldCommand)
{
  assignInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000)),
    controller_interface::return_type::OK);
  const auto held = command_positions_;
  EXPECT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(command_positions_, held);
}

TEST_F(ControllerLifecycleTest, ReadOnlyQueryReturnsSourceCommandAndNeverMeasuredPosition)
{
  const auto inactive = queryHeld();
  ASSERT_NE(inactive, nullptr);
  EXPECT_FALSE(inactive->success);
  assignInterfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto response = queryHeld();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_EQ(response->position.size(), kAxisCount);
  EXPECT_EQ(response->name, std::vector<std::string>(kTestJointNames.begin(), kTestJointNames.end()));
  EXPECT_EQ(response->velocity, std::vector<double>(kAxisCount, 0.0));
  EXPECT_EQ(response->acceleration, std::vector<double>(kAxisCount, 0.0));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_DOUBLE_EQ(response->position[axis], command_positions_[axis]);
    EXPECT_NE(response->position[axis], actual_positions_[axis]);
  }
  EXPECT_FALSE(queryHeld(1)->success);
  EXPECT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(queryHeld()->success);
}

TEST_F(ControllerLifecycleTest, HeldQueryRejectsStaleHardwareFeedback)
{
  assignInterfaces();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  feedback_age_ms_ = 1000.0;
  (void)controller_->update(rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000));
  const auto response = queryHeld();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_TRUE(response->position.empty());
}

}  // namespace
}  // namespace ctrl_rolling_trajectory
