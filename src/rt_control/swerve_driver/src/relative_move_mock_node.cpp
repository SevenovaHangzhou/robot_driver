#include "swerve_driver/relative_move_mock_node.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swerve_driver
{
namespace
{
rcl_interfaces::msg::ParameterDescriptor immutable()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return descriptor;
}
static_assert(static_cast<uint8_t>(MovePhase::fault) == rt_control_interfaces::msg::ChassisState::PHASE_FAULT);
static_assert(static_cast<uint16_t>(MoveFault::execution) == rt_control_interfaces::msg::ChassisState::FAULT_EXECUTION);
static_assert(static_cast<uint16_t>(MoveResult::timed_out) == RelativeMoveMockNode::Action::Result::TIMED_OUT);
}
double RelativeMoveMockNode::required(const std::string & name)
{
  const double value = declare_parameter<double>(name, kRelativeMoveRequired, immutable());
  if (!std::isfinite(value)) {throw std::invalid_argument("required finite parameter: " + name);}
  return value;
}
std::array<double, 4> RelativeMoveMockNode::required_array(const std::string & name)
{
  const auto values = declare_parameter<std::vector<double>>(name, std::vector<double>{}, immutable());
  if (values.size() != 4) {throw std::invalid_argument("required four-element parameter: " + name);}
  std::array<double, 4> result{};
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(values[i])) {throw std::invalid_argument("nonfinite parameter: " + name);}
    result[i] = values[i];
  }
  return result;
}
RelativeMoveMockNode::RelativeMoveMockNode(const rclcpp::NodeOptions & options)
: Node("chassis_relative_move_mock", options)
{
  if (!declare_parameter<bool>("explicit_no_device_mock", false, immutable())) {
    throw std::invalid_argument("explicit_no_device_mock=true required; no real backend exists");
  }
  if (declare_parameter<std::string>("base_frame", "", immutable()).empty()) {
    throw std::invalid_argument("base_frame is required");
  }
  auto & m = config_.motion;
  m.max_translation_velocity_mps = required("max_translation_velocity");
  m.max_translation_acceleration_mps2 = required("max_translation_acceleration");
  m.max_yaw_velocity_radps = required("max_yaw_velocity");
  m.max_yaw_acceleration_radps2 = required("max_yaw_acceleration");
  const auto x = required_array("module_x"), y = required_array("module_y");
  const auto radius = required_array("wheel_radius");
  const auto minimum = required_array("steering_min"), maximum = required_array("steering_max");
  const auto margin = required_array("steering_margin"), tolerance = required_array("steering_measurement_tolerance");
  const auto velocity = required_array("max_wheel_velocity"), acceleration = required_array("max_wheel_acceleration");
  const auto initial_steering = required_array("mock_initial_steering");
  const auto initial_wheel = required_array("mock_initial_wheel_position");
  for (std::size_t i = 0; i < 4; ++i) {
    locations_[i] = {x[i], y[i]};
    m.modules[i] = {locations_[i], radius[i], {minimum[i], maximum[i], margin[i], tolerance[i]}, velocity[i], acceleration[i]};
    if (!steering_angle_within_limits(initial_steering[i], m.modules[i].steering_limits)) {
      throw std::invalid_argument("mock initial steering is outside configured limits");
    }
    feedback_.positions[i] = {initial_steering[i], initial_steering[i], initial_wheel[i]};
    mock_previous_[i] = {initial_wheel[i] * radius[i], initial_steering[i], true};
  }
#define REQUIRED(field) config_.field = required(#field)
  REQUIRED(max_steering_velocity); REQUIRED(max_steering_acceleration);
  REQUIRED(feedback_timeout); REQUIRED(imu_timeout); REQUIRED(imu_max_increment);
  REQUIRED(max_update_period); REQUIRED(stationary_wheel_velocity); REQUIRED(stationary_steering_velocity);
  REQUIRED(stationary_dwell); REQUIRED(alignment_tolerance); REQUIRED(alignment_dwell);
  REQUIRED(alignment_timeout); REQUIRED(steering_error); REQUIRED(steering_error_dwell);
  REQUIRED(encoder_difference); REQUIRED(position_tolerance); REQUIRED(velocity_tolerance);
  REQUIRED(settling_dwell); REQUIRED(settling_timeout); REQUIRED(slip_threshold); REQUIRED(slip_dwell);
  REQUIRED(pose_translation_tolerance); REQUIRED(pose_yaw_tolerance); REQUIRED(yaw_discrepancy);
  REQUIRED(switch_timeout); REQUIRED(stop_timeout); REQUIRED(max_goal_duration);
#undef REQUIRED
  period_ = required("mock_update_period");
  if (period_ <= 0 || period_ > config_.max_update_period || !session_.configure(config_)) {
    throw std::invalid_argument("invalid session/mock timing configuration");
  }
  // Fault injection is confined to this no-device process, never a controller parameter.
  declare_parameter<bool>("mock_imu_valid", true);
  declare_parameter<bool>("mock_bus_ok", true);
  declare_parameter<bool>("mock_drives_ok", true);
  declare_parameter<bool>("mock_freeze_feedback", false);
  declare_parameter<bool>("mock_hold_position", false);
  declare_parameter<bool>("mock_freeze_imu", false);
  declare_parameter<bool>("mock_refuse_mode_switch", false);
  declare_parameter<double>("mock_imu_bias_rate", 0.0);
  declare_parameter<double>("mock_wheel0_velocity_error", 0.0);
  declare_parameter<double>("mock_steering0_error", 0.0);
  feedback_.age = feedback_.imu_age = feedback_.imu_yaw = 0;
  feedback_.bus_ok = feedback_.drives_ok = feedback_.imu_valid = true;
  feedback_.steering_mode.fill(8); feedback_.drive_mode.fill(9);

  state_publisher_ = create_publisher<rt_control_interfaces::msg::ChassisState>("~/state", rclcpp::QoS(1).reliable().durability_volatile());
  // Tier A mock intentionally has no NAVIGATION velocity execution. Never stores Twist,
  // so OPERATION/switch/restart cannot replay a stale command on return to NAVIGATION.
  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).durability_volatile(),
    [](geometry_msgs::msg::Twist::ConstSharedPtr) {});
  mode_service_ = create_service<SetMode>("~/set_mode",
    [this](const std::shared_ptr<rmw_request_id_t> header, const SetMode::Request::SharedPtr req) {
      const auto receipt = effective_feedback(std::chrono::steady_clock::now());
      SetMode::Response response;
      response.code = (reserved_ || mode_header_) ? SetMode::Response::BUSY :
        (!req->confirm || (req->mode != static_cast<uint8_t>(ChassisMode::navigation) &&
        req->mode != static_cast<uint8_t>(ChassisMode::operation))) ? SetMode::Response::INVALID_REQUEST :
        !receipt.encoders_fresh ? SetMode::Response::NOT_READY :
        session_.set_mode(static_cast<ChassisMode>(req->mode), req->confirm);
      if (response.code != SetMode::Response::OK) {
        response.state = chassis_state(); mode_service_->send_response(*header, response); return;
      }
      mode_header_ = header; // capacity one, replied only after simulated per-drive readback
    });
  reset_service_ = create_service<ResetFault>("~/reset_fault",
    [this](ResetFault::Request::ConstSharedPtr req, ResetFault::Response::SharedPtr response) {
      const auto receipt = effective_feedback(std::chrono::steady_clock::now());
      response->code = (reserved_ || mode_header_) ? ResetFault::Response::BUSY :
        !req->confirm ? ResetFault::Response::INVALID_REQUEST :
        (!receipt.encoders_fresh || !receipt.imu_fresh) ? ResetFault::Response::CAUSE_PRESENT :
        session_.reset_fault(req->confirm);
      response->state = chassis_state();
    });
  action_ = rclcpp_action::create_server<Action>(this, "~/relative_move",
    [this](const rclcpp_action::GoalUUID & id, std::shared_ptr<const Action::Goal> goal) {
      if (reserved_ || mode_header_) {return rclcpp_action::GoalResponse::REJECT;}
      const auto receipt = effective_feedback(std::chrono::steady_clock::now());
      if (!receipt.encoders_fresh || !receipt.imu_fresh) {return rclcpp_action::GoalResponse::REJECT;}
      // Receipt checks expire cached evidence without advancing stationary dwell or execution.
      auto preview = session_;
      const auto request = convert(*goal);
      if (!preview.start(request)) {return rclcpp_action::GoalResponse::REJECT;}
      pending_goal_ = request; reserved_ = true; started_ = false; goal_id_ = id;
      cancel_pending_ = false; accepted_at_ = std::chrono::steady_clock::now();
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<Handle> handle) {
      if (!reserved_ || handle->get_goal_id() != goal_id_) {return rclcpp_action::CancelResponse::REJECT;}
      cancel_pending_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<Handle> handle) {handle_ = handle;});
  previous_tick_ = std::chrono::steady_clock::now();
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_)),
    [this]() {tick();});
}
RelativeMoveMockNode::ReceiptFeedback RelativeMoveMockNode::effective_feedback(
  std::chrono::steady_clock::time_point receipt) const
{
  ReceiptFeedback result{};
  result.feedback = feedback_;
  result.elapsed = std::chrono::duration<double>(receipt - previous_tick_).count();
  result.feedback.age += result.elapsed;
  result.feedback.imu_age += result.elapsed;
  result.cycle_fresh = std::isfinite(result.elapsed) && result.elapsed > 0 &&
    result.elapsed <= config_.max_update_period;
  result.encoders_fresh = result.cycle_fresh && std::isfinite(result.feedback.age) &&
    result.feedback.age >= 0 && result.feedback.age <= config_.feedback_timeout;
  result.imu_fresh = result.cycle_fresh && result.feedback.imu_valid &&
    std::isfinite(result.feedback.imu_yaw) && std::isfinite(result.feedback.imu_age) &&
    result.feedback.imu_age >= 0 && result.feedback.imu_age <= config_.imu_timeout;
  return result;
}
MoveRequest RelativeMoveMockNode::convert(const Action::Goal & goal) const
{
  MoveRequest request;
  request.goal = {goal.dx, goal.dy, goal.dyaw}; request.max_duration = goal.max_duration;
  request.limits = config_.motion;
  request.limits.max_translation_velocity_mps = goal.limits.max_translation_velocity;
  request.limits.max_translation_acceleration_mps2 = goal.limits.max_translation_acceleration;
  request.limits.max_yaw_velocity_radps = goal.limits.max_yaw_velocity;
  request.limits.max_yaw_acceleration_radps2 = goal.limits.max_yaw_acceleration;
  for (auto & m : request.limits.modules) {
    m.max_wheel_velocity_radps = goal.limits.max_wheel_velocity;
    m.max_wheel_acceleration_radps2 = goal.limits.max_wheel_acceleration;
  }
  return request;
}
void RelativeMoveMockNode::simulate(double dt)
{
  const auto & out = session_.output();
  if (session_.state().phase == MovePhase::switching && !get_parameter("mock_refuse_mode_switch").as_bool()) {
    mock_mode_ = out.requested_mode;
  }
  if (get_parameter("mock_freeze_feedback").as_bool()) {feedback_.age += dt;}
  else {
    feedback_.age = 0;
    std::array<SwerveModulePosition, 4> positions{};
    for (std::size_t i = 0; i < 4; ++i) {
      if (!out.inhibited && !get_parameter("mock_hold_position").as_bool()) {
        feedback_.positions[i] = {out.steering_position[i], out.steering_position[i], out.wheel_position[i]};
        feedback_.wheel_velocity[i] = out.wheel_velocity[i];
        feedback_.steering_velocity[i] = out.steering_velocity[i];
      }
      positions[i] = {feedback_.positions[i].drive_position_rad * config_.motion.modules[i].wheel_radius_m,
        feedback_.positions[i].steering_measured_rad, true};
    }
    const auto delta = wheel_chassis_delta_from_position_deltas(mock_previous_, positions, locations_);
    if (delta) {mock_yaw_ += delta->dtheta_rad;}
    mock_previous_ = positions;
    feedback_.positions[0].steering_measured_rad += get_parameter("mock_steering0_error").as_double();
    feedback_.wheel_velocity[0] += get_parameter("mock_wheel0_velocity_error").as_double();
  }
  mock_yaw_ += dt * get_parameter("mock_imu_bias_rate").as_double();
  if (get_parameter("mock_freeze_imu").as_bool()) {feedback_.imu_age += dt;}
  else {feedback_.imu_age = 0; feedback_.imu_yaw = mock_yaw_;}
  feedback_.imu_valid = get_parameter("mock_imu_valid").as_bool();
  feedback_.bus_ok = get_parameter("mock_bus_ok").as_bool();
  feedback_.drives_ok = get_parameter("mock_drives_ok").as_bool();
  feedback_.steering_mode.fill(8);
  feedback_.drive_mode.fill(mock_mode_ == ChassisMode::operation ? 8 : 9);
}
rt_control_interfaces::msg::ChassisState RelativeMoveMockNode::chassis_state() const
{
  rt_control_interfaces::msg::ChassisState msg;
  const auto & s = session_.state();
  const auto receipt = effective_feedback(std::chrono::steady_clock::now());
  msg.stamp = now(); msg.mode = static_cast<uint8_t>(s.mode); msg.phase = static_cast<uint8_t>(s.phase);
  msg.fault_code = static_cast<uint16_t>(s.fault);
  msg.ready = s.ready && receipt.encoders_fresh && receipt.imu_fresh && !reserved_ && !mode_header_;
  msg.stationary = s.stationary && receipt.encoders_fresh;
  msg.imu_valid = s.imu_valid && receipt.imu_fresh; msg.goal_active = reserved_;
  if (reserved_) {msg.active_goal_id.uuid = goal_id_;}
  return msg;
}
rt_control_interfaces::msg::ChassisMoveState RelativeMoveMockNode::move_state() const
{
  rt_control_interfaces::msg::ChassisMoveState msg;
  const auto & s = session_.state();
  msg.stamp = estimate_stamp_; msg.execution_start = execution_start_;
  msg.phase = static_cast<uint8_t>(s.phase); msg.fault_code = static_cast<uint16_t>(s.fault);
  msg.progress = s.progress; msg.actual_dx = s.actual.x_m; msg.actual_dy = s.actual.y_m;
  msg.actual_dyaw = s.actual.heading_rad; msg.imu_yaw = s.imu_yaw; msg.wheel_yaw = s.wheel_yaw;
  msg.yaw_difference = s.imu_yaw - s.wheel_yaw; msg.estimate_valid = s.estimate_valid;
  msg.quality_flags = s.quality;
  return msg;
}
void RelativeMoveMockNode::tick()
{
  const auto current = std::chrono::steady_clock::now();
  const auto receipt = effective_feedback(current);
  const double dt = receipt.elapsed;
  previous_tick_ = current;
  // Detect missed update deadlines before applying any pending mode or plant output.
  // Keep the old observations with their true age for the core's timing-fault path.
  if (receipt.cycle_fresh) {simulate(dt);}
  else {feedback_ = receipt.feedback;}
  // Fixed-size core call. Everything above/below (ROS parameters, allocations, publish)
  // is mock adapter work, never part of the core or a production control-loop claim.
  if (cancel_pending_ && session_.state().active) {session_.cancel(); cancel_pending_ = false;}
  session_.update(dt, feedback_);
  if (mode_header_ && session_.state().phase != MovePhase::switching) {
    SetMode::Response response;
    response.code = session_.state().fault == MoveFault::none ? SetMode::Response::OK : SetMode::Response::SWITCH_FAILED;
    response.state = chassis_state(); mode_service_->send_response(*mode_header_, response); mode_header_.reset();
  }
  if (handle_ && !started_) {
    execution_start_ = now(); estimate_stamp_ = execution_start_; started_ = true;
    pending_goal_.max_duration -= std::chrono::duration<double>(current - accepted_at_).count();
    if (!session_.start(pending_goal_)) {
      session_.reject_execution();
      auto result = std::make_shared<Action::Result>();
      result->code = Action::Result::FAULTED;
      result->corridor_token = handle_->get_goal()->corridor_token;
      result->state.phase = static_cast<uint8_t>(MovePhase::fault);
      result->state.fault_code = static_cast<uint16_t>(session_.state().fault);
      result->state.quality_flags = 4U;
      handle_->abort(result); handle_.reset(); reserved_ = false; cancel_pending_ = false;
      execution_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    } else if (cancel_pending_) {session_.cancel(); cancel_pending_ = false;}
  }
  if (session_.state().measurement_sequence != estimate_sequence_) {
    estimate_sequence_ = session_.state().measurement_sequence; estimate_stamp_ = now();
  }
  if (handle_) {
    auto feedback = std::make_shared<Action::Feedback>(); feedback->state = move_state();
    // Correlation stays on the accepted ROS handle, outside the fixed-size session.
    feedback->corridor_token = handle_->get_goal()->corridor_token;
    handle_->publish_feedback(feedback);
    if (!session_.state().active) {
      auto result = std::make_shared<Action::Result>(); result->state = move_state();
      result->corridor_token = handle_->get_goal()->corridor_token;
      result->code = static_cast<uint16_t>(session_.state().result);
      if (session_.state().result == MoveResult::succeeded) {handle_->succeed(result);}
      else if (session_.state().result == MoveResult::canceled) {handle_->canceled(result);}
      else {handle_->abort(result);}
      handle_.reset(); reserved_ = false; cancel_pending_ = false; goal_id_ = {};
      execution_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }
  state_publisher_->publish(chassis_state());
}
}  // namespace swerve_driver
