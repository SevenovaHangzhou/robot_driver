#include <gtest/gtest.h>
#include <chrono>
#include <functional>
#include <thread>
#include "rclcpp/serialization.hpp"
#include "swerve_driver/relative_move_mock_node.hpp"

using Mock = swerve_driver::RelativeMoveMockNode;
using Action = Mock::Action;
using State = rt_control_interfaces::msg::ChassisState;
using MoveState = rt_control_interfaces::msg::ChassisMoveState;
using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;

namespace swerve_driver
{
class RelativeMoveMockTestAccess
{
public:
  static void pause_timer(Mock & node) {node.timer_->cancel();}
  static void dispatch_timer(Mock & node) {node.tick();}
  static bool pending_mode(const Mock & node) {return static_cast<bool>(node.mode_header_);}
  static ChassisMode backend_mode(const Mock & node) {return node.mock_mode_;}
  static MoveOutput output(const Mock & node) {return node.session_.output();}
  static MoveFeedback sample(const Mock & node) {return node.feedback_;}
  static MoveSnapshot state(const Mock & node) {return node.session_.state();}
  static uint16_t reset_preview(const Mock & node)
  {
    auto preview = node.session_;
    return preview.reset_fault(true);
  }
};
}
using Access = swerve_driver::RelativeMoveMockTestAccess;

class RelativeMoveMockTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<Mock> server;
  rclcpp::Node::SharedPtr client;
  rclcpp_action::Client<Action>::SharedPtr action;
  rclcpp::Client<Mock::SetMode>::SharedPtr mode;
  rclcpp::Client<Mock::ResetFault>::SharedPtr reset;
  rclcpp::Subscription<State>::SharedPtr state_subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
  State state;
  MoveState feedback;
  std::size_t feedback_count{0};
  rclcpp::NodeOptions options()
  {
    rclcpp::NodeOptions o;
    o.arguments({"--ros-args", "--params-file", SWERVE_RELATIVE_MOCK_CONFIG});
    return o;
  }
  bool spin(const std::function<bool()> & done, double seconds = 5)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done() && std::chrono::steady_clock::now() < deadline) {executor.spin_some();}
    return done();
  }
  template<class Future> bool wait(Future & future, double seconds = 5)
  {return spin([&]() {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;}, seconds);}
  void SetUp() override
  {
    server = std::make_shared<Mock>(options());
    client = std::make_shared<rclcpp::Node>("relative_move_test_client");
    executor.add_node(server); executor.add_node(client);
    action = rclcpp_action::create_client<Action>(client, "/chassis_relative_move_mock/relative_move");
    mode = client->create_client<Mock::SetMode>("/chassis_relative_move_mock/set_mode");
    reset = client->create_client<Mock::ResetFault>("/chassis_relative_move_mock/reset_fault");
    velocity = client->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    state_subscription = client->create_subscription<State>("/chassis_relative_move_mock/state", 1,
      [this](State::ConstSharedPtr msg) {state = *msg;});
    ASSERT_TRUE(spin([&]() {return state.ready && state.stationary && action->action_server_is_ready() && mode->service_is_ready();}));
  }
  void TearDown() override
  {
    executor.remove_node(server); executor.remove_node(client);
    server.reset(); client.reset();
  }
  Action::Goal goal(double dx = 0.2, double dy = 0, double yaw = 0)
  {
    Action::Goal g; g.corridor_token = "opaque/走廊 ?=a:b"; g.dx = dx; g.dy = dy; g.dyaw = yaw; g.max_duration = 20;
    g.limits.max_translation_velocity = 0.2; g.limits.max_translation_acceleration = 0.3;
    g.limits.max_yaw_velocity = 0.4; g.limits.max_yaw_acceleration = 0.5;
    g.limits.max_wheel_velocity = 3; g.limits.max_wheel_acceleration = 5;
    return g;
  }
  Mock::SetMode::Response::SharedPtr switch_mode(uint8_t value, bool confirm = true)
  {
    auto request = std::make_shared<Mock::SetMode::Request>(); request->mode = value; request->confirm = confirm;
    auto future = mode->async_send_request(request);
    if (!wait(future)) {ADD_FAILURE() << "mode response timeout"; return {};}
    return future.get();
  }
  void operation()
  {
    auto response = switch_mode(State::MODE_OPERATION);
    ASSERT_TRUE(response); ASSERT_EQ(response->code, Mock::SetMode::Response::OK);
    EXPECT_EQ(response->state.mode, State::MODE_OPERATION);
    ASSERT_TRUE(spin([&]() {return state.ready && state.mode == State::MODE_OPERATION;}));
  }
  ClientHandle::SharedPtr send(const Action::Goal & g)
  {
    typename rclcpp_action::Client<Action>::SendGoalOptions o;
    o.feedback_callback = [this, token = g.corridor_token](ClientHandle::SharedPtr, const std::shared_ptr<const Action::Feedback> f) {
      EXPECT_EQ(f->corridor_token, token); ++feedback_count; feedback = f->state;
    };
    auto future = action->async_send_goal(g, o);
    if (!wait(future)) {ADD_FAILURE() << "goal response timeout"; return {};}
    return future.get();
  }
  void inject(const std::string & name, bool value) {ASSERT_TRUE(server->set_parameter({name, value}).successful);}
  void restart_with_timeouts(double encoder, double imu, double cycle)
  {
    executor.remove_node(server); server.reset(); state = State{};
    auto o = options();
    o.parameter_overrides({{"feedback_timeout", encoder}, {"imu_timeout", imu},
      {"max_update_period", cycle}});
    server = std::make_shared<Mock>(o); executor.add_node(server);
    ASSERT_TRUE(spin([&]() {return state.ready && state.stationary &&
      mode->service_is_ready() && reset->service_is_ready();}));
  }

};

TEST_F(RelativeMoveMockTest, ExplicitMockAndCompleteConfigurationRequired)
{
  EXPECT_THROW(std::make_shared<Mock>(), std::invalid_argument);
  rclcpp::NodeOptions o; o.parameter_overrides({{"explicit_no_device_mock", true}, {"base_frame", "test"}});
  EXPECT_THROW(std::make_shared<Mock>(o), std::invalid_argument);
}
TEST_F(RelativeMoveMockTest, ModeAdmissionBusySuccessAndHold)
{
  EXPECT_FALSE(send(goal()));
  auto response = switch_mode(State::MODE_OPERATION, false);
  ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::INVALID_REQUEST);
  operation(); if (HasFatalFailure()) {return;}
  EXPECT_FALSE(send(goal(0.701)));
  auto handle = send(goal(0.1, -0.05, 0.1)); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  EXPECT_FALSE(send(goal()));
  response = switch_mode(State::MODE_NAVIGATION);
  ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::BUSY);
  ASSERT_TRUE(wait(result, 8));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.get().result->code, Action::Result::SUCCEEDED);
  EXPECT_EQ(result.get().result->corridor_token, goal().corridor_token);
  EXPECT_NEAR(result.get().result->state.actual_dx, 0.1, 0.001);
  EXPECT_NEAR(result.get().result->state.actual_dy, -0.05, 0.001);
  EXPECT_NEAR(result.get().result->state.actual_dyaw, 0.1, 0.001);
  EXPECT_TRUE(result.get().result->state.estimate_valid);
  ASSERT_TRUE(spin([&]() {return !state.goal_active && state.ready;}));
  EXPECT_EQ(state.active_goal_id.uuid, (std::array<uint8_t, 16>{}));
}
TEST_F(RelativeMoveMockTest, CancelAlignmentAndDrivePhases)
{
  operation(); if (HasFatalFailure()) {return;}
  for (double progress : {-1.0, 0.001, 0.5, 0.99}) {
    feedback = MoveState{};
    auto g = progress < 0 ? goal(0, 0.5) : goal(0.5);
    g.corridor_token = "cancel-phase/" + std::to_string(progress);
    auto handle = send(g); ASSERT_TRUE(handle);
    auto result = action->async_get_result(handle);
    ASSERT_TRUE(spin([&]() {return progress < 0 ? feedback.phase == State::PHASE_ALIGNING : feedback.progress >= progress;}, 8));
    auto cancel = action->async_cancel_goal(handle); ASSERT_TRUE(wait(cancel));
    ASSERT_FALSE(cancel.get()->goals_canceling.empty());
    ASSERT_TRUE(wait(result, 5));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(result.get().result->code, Action::Result::CANCELED);
    EXPECT_EQ(result.get().result->corridor_token, g.corridor_token);
    EXPECT_LE(result.get().result->state.progress, 1);
    if (progress < 0) {EXPECT_EQ(result.get().result->state.progress, 0);}
    ASSERT_TRUE(spin([&]() {return state.ready && !state.goal_active;}));
  }
}
TEST_F(RelativeMoveMockTest, StaleImuRejectsZeroAndLossAbortsThenExplicitReset)
{
  operation(); if (HasFatalFailure()) {return;}
  inject("mock_freeze_imu", true);
  ASSERT_TRUE(spin([&]() {return !state.imu_valid;}));
  EXPECT_FALSE(send(goal(0)));
  inject("mock_freeze_imu", false);
  ASSERT_TRUE(spin([&]() {return state.ready;}));
  auto g = goal(); g.corridor_token = "faulting IMU goal";
  auto handle = send(g); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.02;}));
  inject("mock_imu_valid", false);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->code, Action::Result::FAULTED);
  EXPECT_EQ(result.get().result->corridor_token, g.corridor_token);
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_IMU_LOST);
  EXPECT_FALSE(result.get().result->state.estimate_valid);
  EXPECT_EQ(result.get().result->state.quality_flags & MoveState::QUALITY_ESTIMATE_INVALID, MoveState::QUALITY_ESTIMATE_INVALID);
  inject("mock_imu_valid", true);
  ASSERT_TRUE(spin([&]() {return state.imu_valid && !state.goal_active && state.stationary;}));
  EXPECT_FALSE(send(goal()));
  auto request = std::make_shared<Mock::ResetFault::Request>(); request->confirm = true;
  auto reset_result = reset->async_send_request(request); ASSERT_TRUE(wait(reset_result));
  EXPECT_EQ(reset_result.get()->code, Mock::ResetFault::Response::OK);
  ASSERT_TRUE(spin([&]() {return state.ready;}));
  g = goal(0); g.corridor_token.clear();
  handle = send(g); ASSERT_TRUE(handle);
  result = action->async_get_result(handle); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.get().result->corridor_token.empty());
}
TEST_F(RelativeMoveMockTest, QualityMismatchDoesNotPreventEncoderSuccess)
{
  operation(); if (HasFatalFailure()) {return;}
  ASSERT_TRUE(server->set_parameter({"mock_imu_bias_rate", 0.03}).successful);
  auto handle = send(goal(0.1)); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_NONE);
  EXPECT_EQ(result.get().result->state.quality_flags & 3U, 3U);
}
TEST_F(RelativeMoveMockTest, SlipAndFaultDuringCancelAbort)
{
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal()); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.01;}));
  ASSERT_TRUE(server->set_parameter({"mock_wheel0_velocity_error", 5.0}).successful);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_SLIP);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}
TEST_F(RelativeMoveMockTest, DriveFaultDuringCancellationAndRestartDiscardWork)
{
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal()); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.01;}));
  auto cancel = action->async_cancel_goal(handle); ASSERT_TRUE(wait(cancel));
  inject("mock_drives_ok", false);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_DRIVE);
  EXPECT_EQ(result.get().result->corridor_token, goal().corridor_token);
  executor.remove_node(server); server.reset();
  state = State{}; feedback = MoveState{};
  server = std::make_shared<Mock>(options()); executor.add_node(server);
  ASSERT_TRUE(spin([&]() {return state.ready && state.mode == State::MODE_NAVIGATION;}));
  EXPECT_FALSE(state.goal_active); EXPECT_EQ(state.fault_code, State::FAULT_NONE);
  EXPECT_FALSE(send(goal()));
}
TEST_F(RelativeMoveMockTest, ModeReadbackTimeoutAndNoStaleVelocityReplay)
{
  geometry_msgs::msg::Twist command; command.linear.x = 0.2; velocity->publish(command);
  operation(); if (HasFatalFailure()) {return;}
  velocity->publish(command);
  auto response = switch_mode(State::MODE_NAVIGATION);
  ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::OK);
  ASSERT_TRUE(spin([&]() {return state.mode == State::MODE_NAVIGATION && state.ready;}));
  EXPECT_TRUE(state.stationary);
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal(0)); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle); ASSERT_TRUE(wait(result));
  EXPECT_DOUBLE_EQ(result.get().result->state.actual_dx, 0);
  ASSERT_TRUE(spin([&]() {return state.ready;}));
  inject("mock_refuse_mode_switch", true);
  response = switch_mode(State::MODE_NAVIGATION);
  ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::SWITCH_FAILED);
  EXPECT_EQ(response->state.fault_code, State::FAULT_MODE_SWITCH);
}
TEST_F(RelativeMoveMockTest, DeadlineWaitsForStopAndReturnsTimedOut)
{
  operation(); if (HasFatalFailure()) {return;}
  inject("mock_hold_position", true);
  auto g = goal(0.5); g.max_duration = 3.6; g.corridor_token = "deadline/opaque";
  auto handle = send(g); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return state.fault_code == State::FAULT_TIMEOUT;}));
  EXPECT_TRUE(state.goal_active);
  inject("mock_hold_position", false);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->code, Action::Result::TIMED_OUT);
  EXPECT_EQ(result.get().result->corridor_token, g.corridor_token);
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_TIMEOUT);
  ASSERT_TRUE(spin([&]() {return !state.goal_active && state.stationary;}));
  auto reset_request = std::make_shared<Mock::ResetFault::Request>(); reset_request->confirm = true;
  auto reset_result = reset->async_send_request(reset_request); ASSERT_TRUE(wait(reset_result));
  ASSERT_EQ(reset_result.get()->code, Mock::ResetFault::Response::OK);
  ASSERT_TRUE(spin([&]() {return state.ready;}));
  g = goal(0); g.corridor_token = "after deadline";
  handle = send(g); ASSERT_TRUE(handle);
  result = action->async_get_result(handle); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.get().result->corridor_token, g.corridor_token);
}
TEST_F(RelativeMoveMockTest, SteeringErrorAndFeedbackLossLatch)
{
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal()); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.02;}));
  ASSERT_TRUE(server->set_parameter({"mock_steering0_error", 0.3}).successful);
  ASSERT_TRUE(spin([&]() {return state.fault_code == State::FAULT_STEERING_ERROR;}));
  inject("mock_freeze_feedback", true);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->state.fault_code, State::FAULT_STEERING_ERROR);
  EXPECT_FALSE(result.get().result->state.estimate_valid);
}
TEST_F(RelativeMoveMockTest, RestartDuringActiveGoalNeverReplaysOrReportsSuccess)
{
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal()); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.02;}));
  executor.remove_node(server); server.reset();
  state = State{}; feedback = MoveState{};
  server = std::make_shared<Mock>(options()); executor.add_node(server);
  ASSERT_TRUE(spin([&]() {return state.ready && state.mode == State::MODE_NAVIGATION;}));
  EXPECT_FALSE(state.goal_active);
  if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    EXPECT_NE(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  }
  operation(); if (HasFatalFailure()) {return;}
  handle = send(goal(0)); ASSERT_TRUE(handle);
  auto new_result = action->async_get_result(handle); ASSERT_TRUE(wait(new_result));
  EXPECT_EQ(new_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_DOUBLE_EQ(new_result.get().result->state.actual_dx, 0);
}

TEST_F(RelativeMoveMockTest, ModeServiceRejectsElapsedEncoderAndCycleAgeBeforeTimer)
{
  for (bool cycle : {false, true}) {
    restart_with_timeouts(cycle ? 1.0 : 0.1, 1.0, cycle ? 0.1 : 1.0);
    if (HasFatalFailure()) {return;}
    Access::pause_timer(*server);
    const auto before = Access::output(*server);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    auto request = std::make_shared<Mock::SetMode::Request>();
    request->mode = State::MODE_OPERATION; request->confirm = true;
    auto result = mode->async_send_request(request);
    ASSERT_TRUE(spin([&]() {return Access::pending_mode(*server) ||
      result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;}));
    ASSERT_FALSE(Access::pending_mode(*server)) << "stale mode handoff was accepted";
    ASSERT_TRUE(wait(result));
    const auto response = result.get();
    EXPECT_EQ(response->code, Mock::SetMode::Response::NOT_READY);
    EXPECT_FALSE(response->state.ready);
    EXPECT_FALSE(response->state.stationary);
    EXPECT_EQ(Access::backend_mode(*server), swerve_driver::ChassisMode::navigation);
    EXPECT_EQ(Access::output(*server).wheel_position, before.wheel_position);
    EXPECT_EQ(Access::output(*server).requested_mode, before.requested_mode);
  }
}
TEST_F(RelativeMoveMockTest, ResetServiceRejectsElapsedEncoderImuAndCycleAgeBeforeTimer)
{
  for (int expired = 0; expired < 3; ++expired) {
    restart_with_timeouts(expired == 0 ? 0.1 : 1.0,
      expired == 1 ? 0.1 : 1.0, expired == 2 ? 0.1 : 1.0);
    if (HasFatalFailure()) {return;}
    inject("mock_refuse_mode_switch", true);
    auto failed_switch = switch_mode(State::MODE_OPERATION);
    ASSERT_TRUE(failed_switch);
    ASSERT_EQ(failed_switch->code, Mock::SetMode::Response::SWITCH_FAILED);
    ASSERT_EQ(Access::reset_preview(*server), Mock::ResetFault::Response::OK);
    ASSERT_TRUE(Access::output(*server).inhibited);
    const auto fault = Access::state(*server).fault;
    Access::pause_timer(*server);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    auto request = std::make_shared<Mock::ResetFault::Request>(); request->confirm = true;
    auto result = reset->async_send_request(request); ASSERT_TRUE(wait(result));
    const auto response = result.get();
    EXPECT_EQ(response->code, Mock::ResetFault::Response::CAUSE_PRESENT);
    EXPECT_FALSE(response->state.ready);
    if (expired != 1) {EXPECT_FALSE(response->state.stationary);}
    if (expired != 0) {EXPECT_FALSE(response->state.imu_valid);}
    EXPECT_EQ(Access::state(*server).fault, fault);
    EXPECT_TRUE(Access::output(*server).inhibited);
  }
}
TEST_F(RelativeMoveMockTest, OverdueTimerFaultsBeforeApplyingPendingMode)
{
  Access::pause_timer(*server);
  auto request = std::make_shared<Mock::SetMode::Request>();
  request->mode = State::MODE_OPERATION; request->confirm = true;
  auto result = mode->async_send_request(request);
  ASSERT_TRUE(spin([&]() {return Access::pending_mode(*server);}));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  Access::dispatch_timer(*server);
  EXPECT_EQ(Access::backend_mode(*server), swerve_driver::ChassisMode::navigation);
  EXPECT_EQ(Access::sample(*server).drive_mode, (std::array<int, 4>{9, 9, 9, 9}));
  EXPECT_EQ(Access::state(*server).fault, swerve_driver::MoveFault::execution);
  EXPECT_TRUE(Access::output(*server).inhibited);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get()->code, Mock::SetMode::Response::SWITCH_FAILED);
}
TEST_F(RelativeMoveMockTest, OverdueTimerFaultsBeforeApplyingPendingMotionOutput)
{
  operation(); if (HasFatalFailure()) {return;}
  auto handle = send(goal()); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  ASSERT_TRUE(spin([&]() {return feedback.progress > 0.02;}));
  Access::pause_timer(*server);
  const auto before = Access::sample(*server);
  ASSERT_NE(before.positions[0].drive_position_rad, Access::output(*server).wheel_position[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  Access::dispatch_timer(*server);
  const auto after = Access::sample(*server);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(after.positions[i].drive_position_rad, before.positions[i].drive_position_rad);
    EXPECT_EQ(after.positions[i].steering_motor_rad, before.positions[i].steering_motor_rad);
  }
  EXPECT_EQ(after.imu_yaw, before.imu_yaw);
  EXPECT_EQ(Access::state(*server).fault, swerve_driver::MoveFault::execution);
  EXPECT_TRUE(Access::output(*server).inhibited);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(RelativeMoveMockTest, CorridorTokenSerializationPreservesOpaqueString)
{
  // All three generated wire payloads, including empty, Unicode and long opaque data.
  for (const auto & token : {std::string{}, std::string{"opaque/走廊 ?=a:b"}, std::string(4096, 'x')}) {
    Action::Goal g; g.corridor_token = token;
    Action::Feedback f; f.corridor_token = token;
    Action::Result r; r.corridor_token = token;
    const auto round_trip = [&token](const auto & input) {
      using Message = std::decay_t<decltype(input)>;
      rclcpp::Serialization<Message> serializer;
      rclcpp::SerializedMessage wire;
      serializer.serialize_message(&input, &wire);
      Message output; serializer.deserialize_message(&wire, &output);
      EXPECT_EQ(output.corridor_token, token);
    };
    round_trip(g); round_trip(f); round_trip(r);
  }
}
TEST_F(RelativeMoveMockTest, CorridorTokenIsolatedAcrossRejectedAndSubsequentGoals)
{
  operation(); if (HasFatalFailure()) {return;}
  for (const auto & token : {std::string{"first opaque token"}, std::string{}, std::string{"third/新"}}) {
    auto g = goal(0.05); g.corridor_token = token;
    const auto count_before = feedback_count;
    auto handle = send(g); ASSERT_TRUE(handle);
    auto result = action->async_get_result(handle);
    auto rejected = goal(0); rejected.corridor_token = "must never replace accepted token";
    EXPECT_FALSE(send(rejected));
    ASSERT_TRUE(wait(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(result.get().result->corridor_token, token);
    EXPECT_GT(feedback_count, count_before);
    ASSERT_TRUE(spin([&]() {return state.ready && !state.goal_active;}));
    rejected.dx = 0.701;
    EXPECT_FALSE(send(rejected));
  }
}
TEST_F(RelativeMoveMockTest, CorridorTokenEchoedWhenAcceptedGoalCannotStart)
{
  operation(); if (HasFatalFailure()) {return;}
  Access::pause_timer(*server);
  auto g = goal(); g.corridor_token = "accepted before delayed timer";
  auto handle = send(g); ASSERT_TRUE(handle);
  auto result = action->async_get_result(handle);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  Access::dispatch_timer(*server);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->code, Action::Result::FAULTED);
  EXPECT_EQ(result.get().result->corridor_token, g.corridor_token);
}
TEST_F(RelativeMoveMockTest, StaleInvalidServiceRequestsReturnInvalidRequest)
{
  Access::pause_timer(*server);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  for (uint8_t value : {uint8_t{0}, uint8_t{1}, uint8_t{2}, uint8_t{255}}) {
    for (bool confirm : {false, true}) {
      if (confirm && (value == State::MODE_NAVIGATION || value == State::MODE_OPERATION)) {continue;}
      auto response = switch_mode(value, confirm);
      ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::INVALID_REQUEST);
      EXPECT_FALSE(Access::pending_mode(*server));
    }
  }
  auto request = std::make_shared<Mock::ResetFault::Request>(); request->confirm = false;
  auto result = reset->async_send_request(request); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get()->code, Mock::ResetFault::Response::INVALID_REQUEST);
  EXPECT_EQ(Access::backend_mode(*server), swerve_driver::ChassisMode::navigation);
}
TEST_F(RelativeMoveMockTest, BusyServicePrecedenceSurvivesInvalidAndStaleRequests)
{
  Access::pause_timer(*server);
  auto request = std::make_shared<Mock::SetMode::Request>();
  request->mode = State::MODE_OPERATION; request->confirm = true;
  auto pending = mode->async_send_request(request);
  ASSERT_TRUE(spin([&]() {return Access::pending_mode(*server);}));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto response = switch_mode(255, false);
  ASSERT_TRUE(response); EXPECT_EQ(response->code, Mock::SetMode::Response::BUSY);
  auto reset_request = std::make_shared<Mock::ResetFault::Request>(); reset_request->confirm = false;
  auto reset_result = reset->async_send_request(reset_request); ASSERT_TRUE(wait(reset_result));
  EXPECT_EQ(reset_result.get()->code, Mock::ResetFault::Response::BUSY);
  Access::dispatch_timer(*server);
  ASSERT_TRUE(wait(pending));
  EXPECT_EQ(pending.get()->code, Mock::SetMode::Response::SWITCH_FAILED);
}
