#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>
#include "ctrl_stoppable_trajectory/controller.hpp"
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl/time.h"
#include "rclcpp_action/rclcpp_action.hpp"

thread_local bool count_allocations = false;
thread_local std::size_t allocations = 0;
void * operator new(std::size_t size)
{
  if (count_allocations) {
    ++allocations;
  }
  if (void * value = std::malloc(size ? size : 1)) {
    return value;
  }
  throw std::bad_alloc();
}
void * operator new[](std::size_t size)
{
  return ::operator new(size);
}
void operator delete(void * value) noexcept
{
  std::free(value);
}
void operator delete[](void * value) noexcept
{
  std::free(value);
}
void operator delete(void * value, std::size_t) noexcept
{
  std::free(value);
}
void operator delete[](void * value, std::size_t) noexcept
{
  std::free(value);
}

namespace
{
using Controller = ctrl_stoppable_trajectory::Controller;
using Stop = Controller::Stop;
using FJT = control_msgs::action::FollowJointTrajectory;
using Point = trajectory_msgs::msg::JointTrajectoryPoint;
using Msg = trajectory_msgs::msg::JointTrajectory;
using Return = controller_interface::return_type;
constexpr double rad = 0.017453292519943295;
constexpr std::int64_t epoch = 10000000000LL;
void require(bool value, const char * what)
{
  if (!value) {
    throw std::runtime_error(what);
  }
}

class Inspectable : public Controller
{
public:
  const Point & reference() const { return state_desired_; }
};

class StopController : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

struct Harness
{
  std::shared_ptr<Inspectable> controller = std::make_shared<Inspectable>();
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp_action::Client<FJT>::SharedPtr action;
  rclcpp::Client<Stop>::SharedPtr stop;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<std::string> names;
  std::vector<double> q, command;
  double age = 0.;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  std::string label;

  explicit Harness(bool open = true, bool missing_interface = false) : q(14, 0.), command(14, 0.)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    static unsigned serial = 0;
    label = "stop_test_" + std::to_string(serial++);
    for (int i = 0; i < 14; ++i) {
      names.push_back("axis_" + std::to_string(i));
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("joints", names),
       rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
       rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position"}),
       rclcpp::Parameter("use_sim_time", true), rclcpp::Parameter("open_loop_control", open),
       rclcpp::Parameter("action_name", "/" + label + "/fjt"),
       rclcpp::Parameter("bounded_stop_service", "/" + label + "/stop"),
       rclcpp::Parameter("set_last_command_interface_value_as_state_on_activation", false),
       rclcpp::Parameter("trajectory_start_consistency_check.enabled", true),
       rclcpp::Parameter(
         "trajectory_start_consistency_check.position_tolerances", std::vector<double>(14, rad)),
       rclcpp::Parameter(
         "trajectory_start_consistency_check.feedback_age_state_interface", "offline/age_ms"),
       rclcpp::Parameter("trajectory_start_consistency_check.max_feedback_age_ms", 500.),
       rclcpp::Parameter("joint_velocity_limits", std::vector<double>(14, 30. * rad)),
       rclcpp::Parameter("joint_acceleration_limits", std::vector<double>(14, 60. * rad)),
       rclcpp::Parameter("constraints.goal_time", 0.5),
       rclcpp::Parameter("constraints.stopped_velocity_tolerance", 0.01)});
    require(controller->init(label, "", options) == Return::OK, "init");
    set_time(0);
    require(
      controller->get_node()->configure().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      "configure");
    commands.reserve(14);
    states.reserve(15);
    for (std::size_t i = 0; i < names.size(); ++i) {
      commands.emplace_back(names[i], "position", &command[i]);
      if (!missing_interface || i != 13) {
        states.emplace_back(names[i], "position", &q[i]);
      }
    }
    states.emplace_back("offline", "age_ms", &age);
    assign();
    const auto active = controller->get_node()->activate().id();
    if (missing_interface) {
      require(active != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "missing axis activated");
      return;
    }
    require(active == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "activate");
    executor.add_node(controller->get_node()->get_node_base_interface());
    client_node = std::make_shared<rclcpp::Node>(label + "_client");
    executor.add_node(client_node);
    action = rclcpp_action::create_client<FJT>(client_node, "/" + label + "/fjt");
    stop = client_node->create_client<Stop>("/" + label + "/stop");
    require(action->wait_for_action_server(std::chrono::seconds(3)), "action discovery");
    require(stop->wait_for_service(std::chrono::seconds(3)), "stop discovery");
    require(tick(0) == Return::OK, "initial update");
  }
  void assign()
  {
    std::vector<hardware_interface::LoanedCommandInterface> cmd;
    std::vector<hardware_interface::LoanedStateInterface> state;
    for (auto & value : commands) {
      cmd.emplace_back(value);
    }
    for (auto & value : states) {
      state.emplace_back(value);
    }
    controller->assign_interfaces(std::move(cmd), std::move(state));
  }
  void set_time(std::int64_t t)
  {
    auto clock = controller->get_node()->get_clock()->get_clock_handle();
    require(rcl_enable_ros_time_override(clock) == RCL_RET_OK, "enable clock");
    require(rcl_set_ros_time_override(clock, epoch + t) == RCL_RET_OK, "set clock");
  }
  Return tick(std::int64_t t)
  {
    set_time(t);
    q = command;
    return controller->update(
      rclcpp::Time(epoch + t, RCL_ROS_TIME), rclcpp::Duration::from_seconds(.001));
  }
  template <class Future>
  auto wait(Future & future)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(
      future.wait_for(std::chrono::seconds(0)) == std::future_status::ready, "response timeout");
    return future.get();
  }
  void deliver()
  {
    // Give DDS a turn without advancing the controlled RT clock.
    for (int i = 0; i < 10; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  Msg motion(std::int64_t start = 0, double sign = 1.) const
  {
    Msg message;
    message.joint_names = names;
    message.header.stamp = rclcpp::Time(epoch + start, RCL_ROS_TIME);
    double p = 0., v = 0., a = 0., t = 0.;
    auto append = [&] {
      Point point;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      point.positions = command;
      point.velocities.resize(14);
      point.accelerations.resize(14);
      for (std::size_t i = 0; i < 14; ++i) {
        const double d = (i == 5 ? 1. : i == 2 ? -.5 : 0.) * sign;
        point.positions[i] += d * p * rad;
        point.velocities[i] = d * v * rad;
        point.accelerations[i] = d * a * rad;
      }
      message.points.push_back(point);
    };
    append();
    const double durations[] = {.25, .25, .25, 5.25, .25, .25, .25};
    const double jerks[] = {40., 0., -40., 0., -40., 0., 40.};
    for (int i = 0; i < 7; ++i) {
      const double dt = durations[i], j = jerks[i];
      p += v * dt + a * dt * dt / 2. + j * dt * dt * dt / 6.;
      v += a * dt + j * dt * dt / 2.;
      a += j * dt;
      t += dt;
      append();
    }
    return message;
  }
  auto send(const Msg & message)
  {
    FJT::Goal goal;
    goal.trajectory = message;
    auto future = action->async_send_goal(goal);
    return wait(future);
  }
  auto request(const rclcpp_action::GoalUUID & id)
  {
    auto value = std::make_shared<Stop::Request>();
    value->goal_id.uuid = id;
    value->velocity = 5. * rad;
    value->acceleration = 10. * rad;
    value->jerk = 40. * rad;
    return stop->async_send_request(value);
  }
  ~Harness()
  {
    if (
      controller->get_node()->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      controller->get_node()->deactivate();
    }
    if (client_node) {
      executor.remove_node(client_node);
      executor.remove_node(controller->get_node()->get_node_base_interface());
    }
    controller->get_node()->cleanup();
  }
};

Point sample(const Msg & message, std::int64_t t)
{
  joint_trajectory_controller::Trajectory trajectory(std::make_shared<Msg>(message));
  trajectory.set_point_before_trajectory_msg(
    rclcpp::Time(message.header.stamp), message.points.front());
  Point point;
  joint_trajectory_controller::TrajectoryPointConstIter begin, end;
  require(
    trajectory.sample(
      rclcpp::Time(epoch + t, RCL_ROS_TIME),
      joint_trajectory_controller::interpolation_methods::InterpolationMethod::
        VARIABLE_DEGREE_SPLINE,
      point, begin, end),
    "reference sample");
  return point;
}

TEST_F(StopController, DelayedServiceUsesActualUpdateAndPreservesQVA)
{
  for (bool open : {true, false})
    for (int delay : {5, 20, 50, 100})
      for (int queue : {0, 1, 30}) {
        SCOPED_TRACE(
          ::testing::Message() << "open=" << open << " delay=" << delay << " queue=" << queue);
        Harness h(open);
        const auto motion = h.motion();
        const auto goal = h.send(motion);
        ASSERT_TRUE(goal);
        for (std::int64_t t = 1000000; t <= 235000000; t += 1000000) {
          ASSERT_EQ(h.tick(t), Return::OK);
        }
        auto response = h.request(goal->get_goal_id());
        const std::int64_t receive = (235 + delay) * 1000000LL, apply = receive + queue * 1000000LL;
        for (std::int64_t t = 236000000; t < receive; t += 1000000) {
          ASSERT_EQ(h.tick(t), Return::OK);
        }
        h.deliver();
        ASSERT_EQ(h.tick(apply), Return::OK);
        const auto expected = sample(motion, apply);
        for (std::size_t i = 0; i < 14; ++i) {
          EXPECT_NEAR(h.controller->reference().positions[i], expected.positions[i], 1e-12);
          EXPECT_NEAR(h.controller->reference().velocities[i], expected.velocities[i], 1e-12);
          EXPECT_NEAR(h.controller->reference().accelerations[i], expected.accelerations[i], 1e-12);
        }
        EXPECT_EQ(response.wait_for(std::chrono::seconds(0)), std::future_status::timeout);
        auto previous = h.controller->reference();
        for (std::int64_t t = apply + 1000000; t < apply + 1500000000; t += 1000000) {
          allocations = 0;
          count_allocations = true;
          const auto updated = h.tick(t);
          count_allocations = false;
          ASSERT_EQ(updated, Return::OK);
          ASSERT_EQ(allocations, 0U);
          const auto now = h.controller->reference();
          for (std::size_t i = 0; i < 14; ++i) {
            EXPECT_LE(std::abs(now.velocities[i]), 5. * rad + 1e-9);
            EXPECT_LE(std::abs(now.accelerations[i]), 10. * rad + 1e-9);
            EXPECT_LE(
              std::abs(now.accelerations[i] - previous.accelerations[i]) / .001, 40. * rad + 1e-8);
          }
          previous = now;
        }
        const auto result = h.wait(response);
        EXPECT_EQ(result->status, Stop::Response::REFERENCE_STOPPED);
        EXPECT_EQ(rclcpp::Time(result->applied_at).nanoseconds(), epoch + apply);
        EXPECT_LE(h.command[5], motion.points.back().positions[5]);
        auto completed = h.action->async_get_result(goal);
        EXPECT_EQ(h.wait(completed).code, rclcpp_action::ResultCode::ABORTED);
        // A new normal goal starts from the retained stop position, not the old target.
        const auto held = h.command;
        const auto next_time = apply + 1600000000LL;
        h.set_time(next_time);
        ASSERT_TRUE(h.send(h.motion(next_time, -1.)));
        ASSERT_EQ(h.tick(next_time), Return::OK);
        EXPECT_EQ(h.command, held);
      }
}

TEST_F(StopController, StopsAtCruiseDecelerationAndEndpointInBothDirections)
{
  for (double sign : {1., -1.})
    for (std::int64_t apply :
         {0LL, 235000000LL, 750000000LL, 3500000000LL, 6250000000LL, 6749000000LL}) {
      SCOPED_TRACE(::testing::Message() << "direction=" << sign << " time=" << apply);
      Harness h;
      const auto motion = h.motion(0, sign);
      const auto goal = h.send(motion);
      ASSERT_TRUE(goal);
      for (std::int64_t t = 1000000; t < apply; t += 1000000) {
        ASSERT_EQ(h.tick(t), Return::OK);
      }
      auto response = h.request(goal->get_goal_id());
      h.deliver();
      ASSERT_EQ(h.tick(apply), Return::OK);
      const auto before = sample(motion, apply);
      EXPECT_NEAR(h.controller->reference().accelerations[5], before.accelerations[5], 1e-10);
      for (std::int64_t t = apply + 1000000; t <= apply + 1500000000; t += 1000000) {
        ASSERT_EQ(h.tick(t), Return::OK);
      }
      EXPECT_EQ(h.wait(response)->status, Stop::Response::REFERENCE_STOPPED);
      EXPECT_LE(sign * h.command[5], 30. * rad + 1e-10);
    }
}

TEST_F(StopController, StaleDuplicateAndCompetingGoal)
{
  Harness h;
  const auto goal = h.send(h.motion());
  ASSERT_TRUE(goal);
  h.tick(35000000);
  auto bad_id = goal->get_goal_id();
  bad_id[0] ^= 1;
  auto stale = h.request(bad_id);
  EXPECT_EQ(h.wait(stale)->status, Stop::Response::STALE_GOAL);
  auto first = h.request(goal->get_goal_id());
  h.deliver();
  auto duplicate = h.request(goal->get_goal_id());
  EXPECT_EQ(h.wait(duplicate)->status, Stop::Response::BUSY);
  EXPECT_FALSE(h.send(h.motion()));
  auto cancel = h.action->async_cancel_goal(goal);
  EXPECT_NE(h.wait(cancel)->return_code, 0);
  ASSERT_EQ(h.tick(50000000), Return::OK);
  for (std::int64_t t = 51000000; t < 1500000000; t += 1000000) {
    ASSERT_EQ(h.tick(t), Return::OK);
  }
  EXPECT_EQ(h.wait(first)->status, Stop::Response::REFERENCE_STOPPED);
  auto stale_after = h.request(goal->get_goal_id());
  EXPECT_EQ(h.wait(stale_after)->status, Stop::Response::STALE_GOAL);
}

TEST_F(StopController, FeedbackLossAndClockRegressionLatchFailure)
{
  for (bool stale : {true, false}) {
    Harness h;
    const auto goal = h.send(h.motion());
    ASSERT_TRUE(goal);
    h.tick(35000000);
    auto request = h.request(goal->get_goal_id());
    h.deliver();
    ASSERT_EQ(h.tick(50000000), Return::OK);
    const auto before = h.command;
    if (stale) {
      h.age = 501.;
    }
    EXPECT_EQ(h.tick(stale ? 51000000 : 49000000), Return::ERROR);
    EXPECT_EQ(h.command, before);
    EXPECT_EQ(h.wait(request)->status, Stop::Response::FAILED);
    EXPECT_FALSE(h.send(h.motion()));
    h.controller->get_node()->deactivate();
    h.deliver();
    h.age = 0.;
    h.assign();
    EXPECT_EQ(
      h.controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(h.tick(60000000), Return::OK);
    EXPECT_EQ(h.command, before);
  }
}

TEST_F(StopController, DeactivateWhileQueuedOrBrakingAndMissingInterface)
{
  for (bool braking : {false, true}) {
    Harness h;
    const auto goal = h.send(h.motion());
    ASSERT_TRUE(goal);
    h.tick(35000000);
    auto response = h.request(goal->get_goal_id());
    h.deliver();
    if (braking) {
      ASSERT_EQ(h.tick(50000000), Return::OK);
    }
    h.controller->get_node()->deactivate();
    EXPECT_EQ(h.wait(response)->status, Stop::Response::INACTIVE);
    auto inactive = h.request(goal->get_goal_id());
    EXPECT_EQ(h.wait(inactive)->status, Stop::Response::INACTIVE);
  }
  EXPECT_NO_THROW(Harness(true, true));
}

TEST_F(StopController, InvalidLimitsDoNotReplaceMotionAndErrorResolvesQueuedRequest)
{
  Harness h;
  const auto goal = h.send(h.motion());
  ASSERT_TRUE(goal);
  ASSERT_EQ(h.tick(35000000), Return::OK);
  auto invalid = std::make_shared<Stop::Request>();
  invalid->goal_id.uuid = goal->get_goal_id();
  invalid->velocity = 100.;
  invalid->acceleration = 100.;
  invalid->jerk = 40. * rad;
  auto rejected = h.stop->async_send_request(invalid);
  EXPECT_EQ(h.wait(rejected)->status, Stop::Response::INVALID);
  ASSERT_EQ(h.tick(40000000), Return::OK);
  auto request = h.request(goal->get_goal_id());
  h.deliver();
  EXPECT_EQ(
    h.controller->on_error(h.controller->get_node()->get_current_state()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(h.wait(request)->status, Stop::Response::INACTIVE);
  auto result = h.action->async_get_result(goal);
  EXPECT_EQ(h.wait(result).code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(StopController, PluginLoadsAndUnloadsWithGenericSystem)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const std::string urdf =
    "<robot name='offline'><ros2_control name='memory' type='system'>"
    "<hardware><plugin>mock_components/GenericSystem</plugin></hardware>"
    "<joint name='axis'><command_interface name='position'/><state_interface "
    "name='position'/></joint>"
    "</ros2_control></robot>";
  controller_manager::ControllerManager manager(
    std::make_unique<hardware_interface::ResourceManager>(urdf), executor, "stop_load_manager");
  ASSERT_TRUE(manager.load_controller("stop_load_only", "ctrl_stoppable_trajectory/Controller"));
  EXPECT_EQ(manager.unload_controller("stop_load_only"), Return::OK);
}
}  // namespace
