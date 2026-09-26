// All identities, limits, PDO bytes and plant motion in this file are synthetic.
// The controller and Kinco adapter under test are the actual production plugin classes.
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "robot_hw_ethercat/kinco_cyclic_mode_slave.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "swerve_driver/swerve_controller.hpp"
#include "synthetic_parameters.hpp"
#include "relative_move_fixture.hpp"

namespace
{
using Controller = swerve_driver::SwerveController;
using Runtime = swerve_driver::ChassisRuntime;
using Callback = controller_interface::CallbackReturn;
struct Drive
{
  robot_hw_ethercat::KincoCyclicModeSlave slave;
  std::vector<double> state = std::vector<double>(13, 0);
  std::vector<double> command{0.0, 0.0, 9.0, 15.0, 0.0, 0.0};
  std::array<std::array<uint8_t, 8>, 8> bytes{};
  int32_t actual{100}, velocity{0}, csp_residual{0};
  int8_t mode{9};
  bool refuse_mode{false}, skip_send{false}, complete{true};
  void setup()
  {
    std::unordered_map<std::string, std::string> params{
      {"slave_config", KINCO_FIXTURE}, {"allow_mock_profile", "true"}, {"mode_of_operation", "9"}};
    const std::vector<std::string> commands{"position", "velocity", "mode_of_operation",
      "control_word", "write_sequence", "write_mask"};
    const std::vector<std::string> states{"position", "velocity", "mode_of_operation_display",
      "status_word", "feedback_age_ms",
      "mode_switch_ready", "mode_request_ack", "command_fresh", "mode_request_error",
      "feedback_sequence", "sent_sequence", "feedback_sequence_at_send", "sent_velocity"};
    for (size_t i = 0; i < commands.size(); ++i) {
      params["command_interface/" + commands[i]] = std::to_string(i);
    }
    for (size_t i = 0; i < states.size(); ++i) {
      params["state_interface/" + states[i]] = std::to_string(i);
    }
    ASSERT_TRUE(slave.setupSlave(params, &state, &command)); slave.set_state_is_operational(true);
    read(); write(); command[1] = 0; read(); write();
  }
  void read()
  {
    EC_WRITE_U16(bytes[4].data(), 0x1027);
    EC_WRITE_S32(bytes[5].data(), actual); EC_WRITE_S32(bytes[6].data(), velocity); EC_WRITE_S8(
      bytes[7].data(), mode);
    slave.onPdoCycleRead(complete);
    for (size_t i = 4; i < 8; ++i) {slave.processData(i, bytes[i].data());}
  }
  void write()
  {
    slave.onPdoCycleStart(complete);
    for (size_t i = 0; i < 4; ++i) {slave.processData(i, bytes[i].data());}
    if (!skip_send) {slave.onPdoCycleSent();}
    if (!refuse_mode) {mode = EC_READ_S8(bytes[3].data());}
  }
  void plant(double dt)
  {
    if (mode == 9) {
      velocity = EC_READ_S32(bytes[2].data());
      actual += static_cast<int32_t>(std::llround(static_cast<double>(velocity) * 10.0 * dt));
    } else {
      const auto next = EC_READ_S32(bytes[1].data()) - csp_residual;
      velocity =
        static_cast<int32_t>(std::llround(static_cast<double>(next - actual) / (10.0 * dt)));
      actual = next;
    }
  }
};
std::vector<rclcpp::Parameter> runtime_parameters()
{
  auto params = synthetic_parameters();
  const auto c = swerve_driver::session_config();
  params.emplace_back("max_update_period", 0.1);
  params.emplace_back("max_wheel_speed", 0.3); // nav core m/s, adapter limit 4 rad/s
  params.emplace_back("relative.enabled", true); params.emplace_back(
    "relative.profile_verified",
    true);
  params.emplace_back("imu_enabled", true); params.emplace_back("imu_frame_id", "base_footprint");
  params.emplace_back("imu_timeout", 0.1); params.emplace_back("max_imu_yaw_step", 0.2);
  params.emplace_back("quaternion_norm_tolerance", 0.01);
#define FIELD(name) params.emplace_back("relative." #name, c.name)
  FIELD(max_steering_velocity); FIELD(max_steering_acceleration); FIELD(feedback_timeout); FIELD(
    imu_timeout);
  FIELD(imu_max_increment); FIELD(max_update_period); FIELD(stationary_wheel_velocity);
  FIELD(stationary_steering_velocity); FIELD(stationary_dwell); FIELD(alignment_tolerance); FIELD(
    alignment_dwell);
  FIELD(alignment_timeout); FIELD(steering_error); FIELD(steering_error_dwell); FIELD(
    encoder_difference);
  FIELD(position_tolerance); FIELD(velocity_tolerance); FIELD(settling_dwell); FIELD(
    settling_timeout);
  FIELD(slip_threshold); FIELD(slip_dwell); FIELD(pose_translation_tolerance); FIELD(
    pose_yaw_tolerance);
  FIELD(yaw_discrepancy); FIELD(switch_timeout); FIELD(stop_timeout); FIELD(max_goal_duration);
#undef FIELD
  params.emplace_back("relative.command_timeout", 0.5);
  params.emplace_back("relative.max_translation_velocity", 0.2);
  params.emplace_back("relative.max_translation_acceleration", 0.3);
  params.emplace_back("relative.max_yaw_velocity", 0.4);
  params.emplace_back("relative.max_yaw_acceleration", 0.5);
  params.emplace_back("relative.max_wheel_velocity", std::vector<double>(4, 3));
  params.emplace_back("relative.max_wheel_acceleration", std::vector<double>(4, 5));
  params.emplace_back("relative.seed_tolerance", std::vector<double>(4, 0.005));
  for (const auto * p : {"enabled", "csv", "csp"}) {
    params.emplace_back(std::string("relative.") + p + "_mask", 0x006f);
    params.emplace_back(std::string("relative.") + p + "_value", 0x0027);
  }
  return params;
}
class CyclicControllerTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  Controller controller;
  std::array<Drive, 4> drives;
  std::map<std::string, double> other;
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr client;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Client<Runtime::SetMode>::SharedPtr mode_client;
  rclcpp_action::Client<Runtime::Action>::SharedPtr action;
  rt_control_interfaces::msg::ChassisState::SharedPtr state;
  rclcpp::Subscription<rt_control_interfaces::msg::ChassisState>::SharedPtr state_sub;
  bool imu_valid{true}, running{false};
  controller_interface::return_type update_result{controller_interface::return_type::OK};
  std::chrono::steady_clock::time_point previous;
  double * pointer(const std::string & name, bool command)
  {
    const auto slash = name.find('/'); const auto field = name.substr(slash + 1);
    if (name[0] == 'd' && name[1] >= '0' && name[1] <= '3') {
      const std::vector<std::string> names = command ?
        std::vector<std::string>{"position", "velocity", "mode_of_operation", "control_word",
        "write_sequence", "write_mask"} :
      std::vector<std::string>{"position", "velocity", "mode_of_operation_display", "status_word",
        "feedback_age_ms", "mode_switch_ready",
        "mode_request_ack", "command_fresh", "mode_request_error", "feedback_sequence",
        "sent_sequence", "feedback_sequence_at_send", "sent_velocity"};
      const auto it = std::find(names.begin(), names.end(), field);
      if (it == names.end()) {throw std::runtime_error("unknown drive interface");}
      auto & drive = drives[static_cast<size_t>(name[1] - '0')];
      return &(command ? drive.command : drive.state)[static_cast<size_t>(it - names.begin())];
    }
    return &other[(command ? "command:" : "state:") + name];
  }
  void bind()
  {
    command_handles.clear(); state_handles.clear();
    for (const auto & name : controller.command_interface_configuration().names) {
      const auto slash = name.find('/');
      command_handles.emplace_back(
        name.substr(0, slash), name.substr(slash + 1), pointer(
          name,
          true));
      EXPECT_EQ(name.find("control_word"), std::string::npos);
    }
    for (const auto & name : controller.state_interface_configuration().names) {
      const auto slash = name.find('/');
      state_handles.emplace_back(
        name.substr(0, slash), name.substr(slash + 1),
        pointer(name, false));
      if (name[0] != 'd' && name.find("status_word") != std::string::npos) {
        *pointer(name, false) = 0x1027;
      }
      if (name[0] != 'd' && name.find("mode_of_operation_display") != std::string::npos) {
        *pointer(name, false) = 8;
      }
    }
    std::vector<hardware_interface::LoanedCommandInterface> c;
    std::vector<hardware_interface::LoanedStateInterface> s;
    for (auto & h : command_handles) {c.emplace_back(h);} for (auto & h : state_handles) {
      s.emplace_back(h);
    }
    controller.assign_interfaces(std::move(c), std::move(s));
  }
  void SetUp() override
  {
    for (auto & drive : drives) {drive.setup();}
    ASSERT_EQ(controller.init("cyclic_chassis"), controller_interface::return_type::OK);
    controller.get_node()->set_parameters(runtime_parameters());
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    bind(); ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
    running = true;
    executor.add_node(controller.get_node()->get_node_base_interface());
    client = std::make_shared<rclcpp::Node>("cyclic_client"); executor.add_node(client);
    velocity = client->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      robot_interfaces_qos::control());
    imu = client->create_publisher<sensor_msgs::msg::Imu>(
      "/cyclic_chassis/imu",
      robot_interfaces_qos::fast_state());
    mode_client = client->create_client<Runtime::SetMode>("/cyclic_chassis/set_mode");
    action = rclcpp_action::create_client<Runtime::Action>(client, "/cyclic_chassis/relative_move");
    state_sub = client->create_subscription<rt_control_interfaces::msg::ChassisState>(
      "/cyclic_chassis/state", 1,
      [this](rt_control_interfaces::msg::ChassisState::SharedPtr message) {state = message;});
    previous = std::chrono::steady_clock::now();
    until(
      [&]() {
        return state && state->mode == 1 && state->ready && velocity->get_subscription_count() == 1;
      });
  }
  void TearDown() override {if (running) {controller.on_deactivate(rclcpp_lifecycle::State{});}}
  void tick()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count(); previous = now;
    for (size_t i = 0; i < 4; ++i) {
      drives[i].plant(dt); drives[i].read();
      const auto joint = "s" + std::to_string(i), enc = "e" + std::to_string(i);
      const double target = other["command:" + joint + "/position"];
      other["state:" + joint + "/velocity"] = (target - other["state:" + joint + "/position"]) / dt;
      other["state:" + joint + "/position"] = other["state:" + enc + "/position"] = target;
    }
    sensor_msgs::msg::Imu message; message.header.stamp = client->now();
    message.header.frame_id = "base_footprint";
    message.orientation.w = imu_valid ? 1 : 0; imu->publish(message);
    executor.spin_some();
    update_result =
      controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(dt));
    for (auto & drive : drives) {drive.write();}
    executor.spin_some();
  }
  template<class F> void until(F done, double seconds = 3)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done() && std::chrono::steady_clock::now() < deadline) {tick();}
    ASSERT_TRUE(done()) << "phase=" << (state ? int(state->phase) : -1) << " fault=" <<
      (state ? state->fault_code : 999);
  }
  void command(double speed)
  {
    geometry_msgs::msg::Twist t; t.linear.x = speed; velocity->publish(t);
  }
  auto mode(uint8_t desired)
  {
    auto request = std::make_shared<Runtime::SetMode::Request>(); request->confirm = true;
    request->mode = desired;
    return mode_client->async_send_request(request);
  }
  auto goal(double dx = 0.03)
  {
    Runtime::Action::Goal g; g.dx = dx; g.max_duration = 10;
    g.corridor_token = "opaque/corridor-token";
    g.limits.max_translation_velocity = 0.1; g.limits.max_translation_acceleration = 0.2;
    g.limits.max_yaw_velocity = 0.3; g.limits.max_yaw_acceleration = 0.4;
    g.limits.max_wheel_velocity = 2; g.limits.max_wheel_acceleration = 3;
    return action->async_send_goal(g);
  }
  void operation()
  {
    auto future = mode(2); until(
      [&]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      });
    ASSERT_EQ(future.get()->code, Runtime::SetMode::Response::OK);
    until([&]() {return state && state->mode == 2 && state->ready && state->stationary;});
  }
};
TEST_F(CyclicControllerTest, FullNavigationStopSentPreloadRelativeHoldReturnAndFreshCommand)
{
  command(0.05); until([&]() {return drives[0].velocity > 0;});
  auto future = mode(2);
  bool saw_seed = false;
  until(
    [&]() {
      for (const auto & d : drives) {
        if (d.command[5] == 3 && EC_READ_S8(d.bytes[3].data()) == 9) {saw_seed = true;}
      }
      command(0.1); // Rejected during handoff; must not be replayed on return.
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  ASSERT_EQ(future.get()->code, Runtime::SetMode::Response::OK); EXPECT_TRUE(saw_seed);
  until([&]() {return state && state->ready && state->stationary;});
  auto accepted = goal(); until(
    [&]() {
      return accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  auto handle = accepted.get(); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  until([&]() {return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  const auto completed = result.get(); EXPECT_EQ(
    completed.code,
    rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(completed.result->corridor_token, "opaque/corridor-token");
  EXPECT_NEAR(completed.result->state.actual_dx, 0.03, 0.003);
  const auto held = drives[0].actual; for (int i = 0; i < 10; ++i) {
    tick();
  }
  EXPECT_EQ(drives[0].actual, held);
  auto back = mode(1); until(
    [&]() {
      return back.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  ASSERT_EQ(back.get()->code, Runtime::SetMode::Response::OK);
  for (int i = 0; i < 10; ++i) {
    tick(); EXPECT_EQ(drives[0].velocity, 0);
  }
  command(0.05); until([&]() {return drives[0].velocity > 0;});
}
TEST_F(CyclicControllerTest, ReverseHandoffPreservesHoldDespiteEncoderSettlingResidual)
{
  operation();
  std::array<int32_t, 4> held{};
  for (size_t i = 0; i < drives.size(); ++i) {
    held[i] = EC_READ_S32(drives[i].bytes[1].data());
    drives[i].csp_residual = 3; // 0.003 rad, within the synthetic 0.005 rad tolerance.
  }
  for (int i = 0; i < 20; ++i) {tick();}
  for (size_t i = 0; i < drives.size(); ++i) {
    ASSERT_EQ(drives[i].actual, held[i] - 3);
  }
  auto response = mode(1);
  until([&]() {
    for (size_t i = 0; i < drives.size(); ++i) {
      if ((static_cast<unsigned>(drives[i].command[5]) & 2U) != 0) {
        EXPECT_EQ(EC_READ_S32(drives[i].bytes[1].data()), held[i]);
      }
    }
    return response.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  });
  EXPECT_EQ(response.get()->code, Runtime::SetMode::Response::OK);
  for (const auto & drive : drives) {EXPECT_EQ(drive.mode, 9); EXPECT_EQ(drive.velocity, 0);}
}

TEST_F(CyclicControllerTest, PartialModeReadbackCannotReleaseRelativeMotion)
{
  drives[3].refuse_mode = true;
  auto future = mode(2); until(
    [&]() {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  EXPECT_EQ(future.get()->code, Runtime::SetMode::Response::SWITCH_FAILED);
  for (auto & d : drives) {
    EXPECT_EQ(d.actual, 100);
  }
  auto accepted = goal(); until(
    [&]() {
      return accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  EXPECT_FALSE(accepted.get());
}
TEST_F(CyclicControllerTest, MissingSentHookCannotBeAcknowledgedByAssignmentOrReadback)
{
  drives[2].skip_send = true;
  auto future = mode(2); until(
    [&]() {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  EXPECT_EQ(future.get()->code, Runtime::SetMode::Response::SWITCH_FAILED);
  for (auto & d : drives) {
    EXPECT_EQ(d.mode, 9); EXPECT_EQ(d.actual, 100);
  }
}
TEST_F(CyclicControllerTest, ImuLossDuringGoalStopsAndLatchesWithoutLosingToken)
{
  operation(); auto accepted = goal(0.1);
  until([&]() {return accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  auto handle = accepted.get(); ASSERT_TRUE(handle); auto result = action->async_get_result(handle);
  until([&]() {return drives[0].actual > 120;}); imu_valid = false;
  until([&]() {return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  const auto completed = result.get();
  EXPECT_EQ(completed.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(completed.result->state.fault_code, 1); EXPECT_FALSE(
    completed.result->state.estimate_valid);
  EXPECT_EQ(completed.result->corridor_token, "opaque/corridor-token");
  auto back = mode(1); until(
    [&]() {
      return back.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  EXPECT_EQ(back.get()->code, Runtime::SetMode::Response::FAULT_LATCHED);
}
TEST_F(CyclicControllerTest, ResetUsesCurrentCycleFaultInsteadOfPreviousHealthyFeedback)
{
  operation();
  auto reset = client->create_client<Runtime::Reset>("/cyclic_chassis/reset_fault");
  until([&]() {return reset->service_is_ready();});
  auto request = std::make_shared<Runtime::Reset::Request>();
  request->confirm = true;
  auto response = reset->async_send_request(request);
  // Dispatch the service against a healthy published snapshot, but do not run update yet.
  // The queued reset must be rechecked against the next PDO observation.
  executor.spin_some(std::chrono::milliseconds(2));
  drives[0].complete = false;
  until([&]() {return response.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  EXPECT_NE(response.get()->code, Runtime::Reset::Response::OK);
  EXPECT_EQ(update_result, controller_interface::return_type::ERROR);
  for (const auto & drive : drives) {
    EXPECT_EQ(drive.command[4], 0);
    EXPECT_EQ(drive.command[5], 0);
  }
}

TEST_F(CyclicControllerTest, CancelSettlesAndLifecycleAbortsOutstandingGoal)
{
  operation(); auto accepted = goal(0.1);
  until([&]() {return accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  auto handle = accepted.get(); ASSERT_TRUE(handle); auto result = action->async_get_result(handle);
  until([&]() {return drives[0].actual > 120;}); auto canceled = action->async_cancel_goal(handle);
  until([&]() {return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  until([&]() {return state && state->ready && state->stationary;});
  auto next = goal(); until(
    [&]() {
      return next.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
  auto next_handle = next.get(); ASSERT_TRUE(next_handle);
  auto next_result = action->async_get_result(next_handle);
  tick(); ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}), Callback::SUCCESS);
  running = false;
  executor.spin_some();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (next_result.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline) {executor.spin_some();}
  ASSERT_EQ(next_result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  EXPECT_EQ(next_result.get().code, rclcpp_action::ResultCode::ABORTED);
  for (auto & d : drives) {
    EXPECT_EQ(d.command[4], 0); EXPECT_EQ(d.command[5], 0);
  }
}
} // namespace
