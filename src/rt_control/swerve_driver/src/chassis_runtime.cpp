#include "swerve_driver/chassis_runtime.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace swerve_driver
{
namespace
{
double monotonic_now()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
constexpr const char * session_parameters[] = {
  "max_steering_velocity", "max_steering_acceleration", "feedback_timeout", "imu_timeout",
  "imu_max_increment", "max_update_period", "stationary_wheel_velocity",
  "stationary_steering_velocity",
  "stationary_dwell", "alignment_tolerance", "alignment_dwell", "alignment_timeout",
  "steering_error",
  "steering_error_dwell", "encoder_difference", "position_tolerance", "velocity_tolerance",
  "settling_dwell", "settling_timeout", "slip_threshold", "slip_dwell",
  "pose_translation_tolerance",
  "pose_yaw_tolerance", "yaw_discrepancy", "switch_timeout", "stop_timeout", "max_goal_duration"};
}
void ChassisRuntime::declare_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & n)
{
  n->declare_parameter<bool>("relative.enabled", false);
  n->declare_parameter<bool>("relative.profile_verified", false);
  for (const auto * name : session_parameters) {
    n->declare_parameter<double>(std::string("relative.") + name, kRelativeMoveRequired);
  }
  for (const auto * name :
    {"max_translation_velocity", "max_translation_acceleration", "max_yaw_velocity",
      "max_yaw_acceleration", "command_timeout"})
  {
    n->declare_parameter<double>(std::string("relative.") + name, kRelativeMoveRequired);
  }
  for (const auto * name : {"max_wheel_velocity", "max_wheel_acceleration", "seed_tolerance"}) {
    n->declare_parameter<std::vector<double>>(
      std::string("relative.") + name,
      std::vector<double>{});
  }
  for (const auto * name :
    {"enabled_mask", "enabled_value", "csv_mask", "csv_value", "csp_mask", "csp_value"})
  {
    n->declare_parameter<int>(std::string("relative.") + name, 0);
  }
}
ChassisRuntime::ChassisRuntime(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & n,
  const CoreConfig & core)
: node_(n)
{
  if (!n->get_parameter("relative.profile_verified").as_bool() ||
    !n->get_parameter("imu_enabled").as_bool())
  {
    throw std::invalid_argument(
            "relative mode requires verified profile/calibration and base-frame IMU");
  }
  auto number = [&n](const char * name) {
      return n->get_parameter(std::string("relative.") + name).as_double();
    };
  auto array = [&n](const char * name) {
      const auto values = n->get_parameter(std::string("relative.") + name).as_double_array();
      if (values.size() != 4) {throw std::invalid_argument("relative per-wheel values required");}
      HandoffWheels result{}; std::copy(values.begin(), values.end(), result.begin());
      return result;
    };
#define REQUIRED(field) config_.field = number(#field)
  REQUIRED(max_steering_velocity); REQUIRED(max_steering_acceleration); REQUIRED(feedback_timeout);
  REQUIRED(imu_timeout); REQUIRED(imu_max_increment); REQUIRED(max_update_period);
  REQUIRED(stationary_wheel_velocity); REQUIRED(stationary_steering_velocity); REQUIRED(
    stationary_dwell);
  REQUIRED(alignment_tolerance); REQUIRED(alignment_dwell); REQUIRED(alignment_timeout);
  REQUIRED(steering_error); REQUIRED(steering_error_dwell); REQUIRED(encoder_difference);
  REQUIRED(position_tolerance); REQUIRED(velocity_tolerance); REQUIRED(settling_dwell);
  REQUIRED(settling_timeout); REQUIRED(slip_threshold); REQUIRED(slip_dwell);
  REQUIRED(pose_translation_tolerance); REQUIRED(pose_yaw_tolerance); REQUIRED(yaw_discrepancy);
  REQUIRED(switch_timeout); REQUIRED(stop_timeout); REQUIRED(max_goal_duration);
#undef REQUIRED
  auto & m = config_.motion;
  m.max_translation_velocity_mps = number("max_translation_velocity");
  m.max_translation_acceleration_mps2 = number("max_translation_acceleration");
  m.max_yaw_velocity_radps = number("max_yaw_velocity");
  m.max_yaw_acceleration_radps2 = number("max_yaw_acceleration");
  handoff_config_.max_wheel_velocity = array("max_wheel_velocity");
  handoff_config_.max_wheel_acceleration = array("max_wheel_acceleration");
  handoff_config_.position_tolerance = array("seed_tolerance");
  for (size_t i = 0; i < 4; ++i) {
    m.modules[i] = {core.locations[i], core.wheel_radius[i],
      {core.steering_min[i], core.steering_max[i], core.steering_limit_margin[i],
        core.steering_limit_tolerance[i]},
      handoff_config_.max_wheel_velocity[i], handoff_config_.max_wheel_acceleration[i]};
  }
#define COPY(field) handoff_config_.field = config_.field
  COPY(stationary_wheel_velocity); COPY(stationary_steering_velocity); COPY(stationary_dwell);
  COPY(feedback_timeout); COPY(max_update_period); COPY(stop_timeout); COPY(switch_timeout);
#undef COPY
  handoff_config_.command_timeout = number("command_timeout");
  auto predicate = [&n](const char * mask, const char * value) {
      const auto a = n->get_parameter(std::string("relative.") + mask).as_int();
      const auto b = n->get_parameter(std::string("relative.") + value).as_int();
      if (a <= 0 || a > 65535 || b < 0 || b > 65535) {
        throw std::invalid_argument("status predicates required");
      }
      return HandoffStatusPredicate{static_cast<uint16_t>(a), static_cast<uint16_t>(b)};
    };
  handoff_config_.enabled = predicate("enabled_mask", "enabled_value");
  handoff_config_.csv_ready = predicate("csv_mask", "csv_value");
  handoff_config_.csp_ready = predicate("csp_mask", "csp_value");
  ChassisModeHandoff check;
  if (!session_.configure(config_) || !check.configure(handoff_config_, 1)) {
    throw std::invalid_argument("invalid required relative/handoff limits");
  }
}
bool ChassisRuntime::activate(double now, const ChassisRuntimeFeedback & f, uint64_t lifetime)
{
  std::lock_guard<std::mutex> lock(endpoint_mutex_);
  if (active_ || lifetime == 0 || lifetime > 8000000000000000ULL) {return false;}
  lifetime_ = wire_base_ = lifetime;
  handoff_ = ChassisModeHandoff{};
  session_ = RelativeMoveSession{};
  session_live_ = false;
  Request discarded; while (requests_.pop(discarded)) {}
  drain_snapshots(); latest_ = Snapshot{};
  pending_mode_id_ = rt_response_id_ = rt_goal_id_ = 0;
  execution_ns_ = estimate_ns_ = 0; last_measurement_ = 0;
  cancel_id_.store(0); navigation_generation_.store(0);
  if (!session_.configure(config_) || !handoff_.configure(handoff_config_, lifetime) ||
    !handoff_.start_navigation(now, f.drives, f.sent_velocity)) {return false;}
  for (size_t i = 0; i < 4; ++i) {
    steering_[i] = f.move.positions[i].steering_motor_rad;
  }
  state_publisher_ = node_->create_publisher<rt_control_interfaces::msg::ChassisState>(
    "~/state", rclcpp::QoS(
      1));
  mode_service_ = node_->create_service<SetMode>(
    "~/set_mode",
    [this, weak = weak_from_this(), lifetime](std::shared_ptr<rmw_request_id_t> header,
    SetMode::Request::SharedPtr req) {
      const auto keep_alive = weak.lock(); if (!keep_alive) {return;}
      std::lock_guard<std::mutex> guard(endpoint_mutex_); drain_snapshots();
      SetMode::Response response;
      response.code = (reserved_ || mode_header_ || reset_header_) ? SetMode::Response::BUSY :
      (!req->confirm || (req->mode != 1 && req->mode != 2)) ? SetMode::Response::INVALID_REQUEST :
      !active_ || lifetime != lifetime_ || !snapshot_fresh() ? SetMode::Response::NOT_READY :
      (latest_.cause != HandoffCause::none || latest_.move.fault != MoveFault::none) ? SetMode::Response::FAULT_LATCHED :
      SetMode::Response::OK;
      if (response.code == SetMode::Response::OK) {
        Request request; request.kind = RequestKind::mode; request.lifetime = lifetime;
        request.id = ++request_id_; request.mode = static_cast<ChassisMode>(req->mode); request.receipt = monotonic_now();
        if (requests_.push(request)) {mode_header_ = header; return;}
        response.code = SetMode::Response::BUSY;
      }
      response.state = chassis_state(); mode_service_->send_response(*header, response);
    });
  reset_service_ = node_->create_service<Reset>(
    "~/reset_fault",
    [this, weak = weak_from_this(), lifetime](std::shared_ptr<rmw_request_id_t> header,
    Reset::Request::SharedPtr req) {
      const auto keep_alive = weak.lock(); if (!keep_alive) {return;}
      std::lock_guard<std::mutex> guard(endpoint_mutex_); drain_snapshots();
      Reset::Response response;
      response.code = reserved_ || mode_header_ || reset_header_ ? Reset::Response::BUSY :
      !req->confirm ? Reset::Response::INVALID_REQUEST :
      !active_ || lifetime != lifetime_ || !snapshot_fresh() ? Reset::Response::CAUSE_PRESENT : Reset::Response::OK;
      if (response.code == Reset::Response::OK) {
        Request request; request.kind = RequestKind::reset; request.lifetime = lifetime; request.id = ++request_id_;
        if (requests_.push(request)) {reset_header_ = header; return;}
        response.code = Reset::Response::BUSY;
      }
      response.state = chassis_state(); reset_service_->send_response(*header, response);
    });
  action_ = rclcpp_action::create_server<Action>(
    node_, "~/relative_move",
    [this, weak = weak_from_this(), lifetime](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal) {
      const auto keep_alive = weak.lock(); if (!keep_alive) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      std::lock_guard<std::mutex> guard(endpoint_mutex_); drain_snapshots();
      if (!active_ || lifetime != lifetime_ || reserved_ || mode_header_ || reset_header_ ||
      !snapshot_fresh() || latest_.phase != HandoffPhase::operation || !latest_.move.ready ||
      !latest_.move.stationary || !latest_.move.imu_valid)
      {
        return rclcpp_action::GoalResponse::REJECT;
      }
      Request request; request.kind = RequestKind::goal; request.lifetime = lifetime;
      request.id = ++request_id_; request.goal = convert(*goal); request.receipt = monotonic_now();
      const double age = request.receipt - latest_.time;
      auto preview = latest_.admission;
      if (latest_.feedback_age + age > config_.feedback_timeout ||
      latest_.imu_age + age > config_.imu_timeout || !preview.start(request.goal) ||
      !requests_.push(request)) {return rclcpp_action::GoalResponse::REJECT;}
      goal_id_ = request.id; reserved_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this, weak = weak_from_this(), lifetime](std::shared_ptr<Handle> handle) {
      const auto keep_alive = weak.lock(); if (!keep_alive) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      std::lock_guard<std::mutex> guard(endpoint_mutex_);
      if (!active_ || lifetime != lifetime_ || !handle_ ||
      handle_->get_goal_id() != handle->get_goal_id())
      {
        return rclcpp_action::CancelResponse::REJECT;
      }
      cancel_id_.store(goal_id_); return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this, weak = weak_from_this(), lifetime](std::shared_ptr<Handle> handle) {
      const auto keep_alive = weak.lock();
      if (!keep_alive) {
        auto result = std::make_shared<Action::Result>(); result->code = Action::Result::FAULTED;
        result->corridor_token = handle->get_goal()->corridor_token; handle->abort(result); return;
      }
      std::lock_guard<std::mutex> guard(endpoint_mutex_);
      if (active_ && lifetime == lifetime_) {handle_ = handle;} else {
        auto result = std::make_shared<Action::Result>(); result->code = Action::Result::FAULTED;
        result->corridor_token = handle->get_goal()->corridor_token; handle->abort(result);
      }
    });
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(20), [this,
    weak = weak_from_this()]() {
      const auto keep_alive = weak.lock(); if (keep_alive) {
        publish();
      }
    });
  active_ = true; return true;
}
ChassisRuntimeOutput ChassisRuntime::update(
  double now, double dt, int64_t ros_ns,
  const ChassisRuntimeFeedback & f, const ControlOutput & nav,
  double receipt, uint64_t generation, bool valid) noexcept
{
  Request request;
  uint64_t reset_request_id = 0;
  if (requests_.pop(request) && request.lifetime == lifetime_) {
    if (request.kind == RequestKind::mode) {
      uint16_t code = SetMode::Response::OK;
      if (handoff_.cause() != HandoffCause::none || session_.state().fault != MoveFault::none) {
        code = SetMode::Response::FAULT_LATCHED;
      } else if (request.mode == ChassisMode::operation && handoff_.navigation_open()) {
        if (!handoff_.begin_operation(now)) {code = SetMode::Response::NOT_READY;}
      } else if (request.mode == ChassisMode::navigation && handoff_.operation_ready()) {
        if (session_.state().active) {
          code = SetMode::Response::BUSY;
        } else if (!handoff_.begin_navigation(now, session_.output().wheel_position)) {
          code = SetMode::Response::NOT_STATIONARY;
        }
      } else if (request.mode == ChassisMode::operation && handoff_.operation_ready()) {
        if (!session_.state().stationary) {code = SetMode::Response::NOT_STATIONARY;}
      } else if (request.mode == ChassisMode::navigation && handoff_.navigation_open()) {
        if (!handoff_.reconfirm_navigation(now)) {code = SetMode::Response::NOT_STATIONARY;}
      } else {code = SetMode::Response::BUSY;}
      if (code == SetMode::Response::OK) {
        pending_mode_id_ = request.id; pending_mode_ = request.mode;
      } else {rt_response_id_ = request.id; rt_response_code_ = code;}
    } else if (request.kind == RequestKind::reset) {
      // Judge reset only after this cycle's feedback and timing have been processed.
      // A cached healthy sample must not clear a latch before a new fault is observed.
      reset_request_id = request.id;
    } else {
      rt_goal_id_ = request.id; execution_ns_ = estimate_ns_ = ros_ns;
      request.goal.max_duration -= now - request.receipt;
      if (!handoff_.operation_ready() || !session_.start(request.goal)) {
        session_.reject_execution();
      }
    }
  }
  if (handoff_.navigation_open()) {
    steering_ = nav.steering_position;
    if (valid) {
      const HandoffNavigationCommand command{nav.drive_velocity, generation, receipt};
      if (!handoff_.accept_navigation(command, now)) {
        (void)handoff_.refine_navigation(command, now);
      }
    } else {handoff_.discard_navigation();}
  }
  handoff_.update(now, f.drives, f.ack);
  if (handoff_.operation_ready()) {
    if (!session_live_) {
      // This executor lifetime starts only after the exact preload, send and all-four
      // readback gate. Returning to CSV is permitted only without a session fault.
      session_ = RelativeMoveSession{}; (void)session_.configure(config_);
      if (!session_.enter_confirmed_operation(
          f.move, handoff_.output().wheel_position,
          steering_))
      {
        session_.reject_execution();
      }
      session_live_ = true;
    }
    if (rt_goal_id_ != 0 && cancel_id_.load() == rt_goal_id_) {session_.cancel();}
    session_.update(dt, f.move);
  } else if (handoff_.navigation_open()) {session_live_ = false;}
  if (handoff_.cause() != HandoffCause::none && session_.state().active) {
    auto lost = f.move; lost.bus_ok = false; session_.update(dt, lost);
  }
  if (reset_request_id != 0) {
    rt_response_id_ = reset_request_id;
    rt_response_code_ = handoff_.cause() != HandoffCause::none ?
      Reset::Response::RESTART_REQUIRED :
      session_live_ && handoff_.operation_ready() ?
      session_.reset_fault(true) : Reset::Response::CAUSE_PRESENT;
  }
  if (pending_mode_id_ != 0) {
    const bool done = pending_mode_ == ChassisMode::operation ?
      handoff_.operation_ready() &&
      session_.state().mode == ChassisMode::operation : handoff_.navigation_open();
    if (done || handoff_.cause() != HandoffCause::none ||
      session_.state().fault != MoveFault::none)
    {
      rt_response_id_ = pending_mode_id_; pending_mode_id_ = 0;
      rt_response_code_ = done ? SetMode::Response::OK : SetMode::Response::SWITCH_FAILED;
    }
  }
  if (session_.state().measurement_sequence != last_measurement_) {
    last_measurement_ = session_.state().measurement_sequence; estimate_ns_ = ros_ns;
  }
  Snapshot snapshot{session_.state(), handoff_.phase(), handoff_.cause(), now,
    rt_response_id_, rt_goal_id_, rt_response_code_, ros_ns, execution_ns_, estimate_ns_,
    f.move.age, f.move.imu_age, session_};
  (void)snapshots_.push(snapshot);
  navigation_generation_.store(handoff_.navigation_open() ? handoff_.generation() : 0);
  ChassisRuntimeOutput result{handoff_.output(), steering_};
  if (handoff_.operation_ready()) {
    const auto & move = session_.output();
    // During stationary session initialization retain the coordinator's frozen seed.
    if (session_.state().mode == ChassisMode::operation) {
      result.drive.inhibited = move.inhibited && (session_.state().active ||
        !f.move.bus_ok || !f.move.drives_ok || session_.state().fault == MoveFault::none);
      result.drive.write_position = !move.inhibited;
      result.drive.wheel_position = move.wheel_position;
      result.steering = move.steering_position;
      steering_ = result.steering;
    }
    result.drive.write_sequence = 0; // Executor writes cannot acknowledge coordinator work.
  }
  return result;
}
MoveRequest ChassisRuntime::convert(const Action::Goal & g) const
{
  MoveRequest r{{g.dx, g.dy, g.dyaw}, config_.motion, g.max_duration};
  r.limits.max_translation_velocity_mps = g.limits.max_translation_velocity;
  r.limits.max_translation_acceleration_mps2 = g.limits.max_translation_acceleration;
  r.limits.max_yaw_velocity_radps = g.limits.max_yaw_velocity;
  r.limits.max_yaw_acceleration_radps2 = g.limits.max_yaw_acceleration;
  for (auto & m : r.limits.modules) {
    m.max_wheel_velocity_radps = g.limits.max_wheel_velocity;
    m.max_wheel_acceleration_radps2 = g.limits.max_wheel_acceleration;
  }
  return r;
}
void ChassisRuntime::drain_snapshots()
{
  Snapshot s;
  for (size_t i = 0; i < 8 && snapshots_.pop(s); ++i) {
    latest_ = s;
  }
}
bool ChassisRuntime::snapshot_fresh() const
{
  const double age = monotonic_now() - latest_.time;
  return latest_.time > 0 && age >= 0 && age <= config_.max_update_period;
}
rt_control_interfaces::msg::ChassisState ChassisRuntime::chassis_state() const
{
  rt_control_interfaces::msg::ChassisState out;
  const auto & s = latest_.move;
  out.stamp = rclcpp::Time(latest_.ros_ns, RCL_ROS_TIME);
  const bool nav = latest_.phase == HandoffPhase::navigation;
  const bool op = latest_.phase == HandoffPhase::operation;
  out.mode =
    static_cast<uint8_t>(nav ? ChassisMode::navigation : op ? ChassisMode::operation : ChassisMode
    ::unknown);
  out.phase = static_cast<uint8_t>(op ? s.phase : nav ? MovePhase::idle : MovePhase::switching);
  out.fault_code =
    static_cast<uint16_t>(latest_.cause != HandoffCause::none ? MoveFault::mode_switch : s.fault);
  if (out.fault_code != 0) {out.phase = static_cast<uint8_t>(MovePhase::fault);}
  out.ready = active_ && snapshot_fresh() && (nav || (op && s.ready)) && !reserved_ &&
    !mode_header_ && out.fault_code == 0;
  out.stationary = s.stationary && snapshot_fresh();
  out.imu_valid = s.imu_valid && snapshot_fresh();
  out.goal_active = reserved_; if (handle_) {out.active_goal_id.uuid = handle_->get_goal_id();}
  return out;
}
rt_control_interfaces::msg::ChassisMoveState ChassisRuntime::move_state() const
{
  rt_control_interfaces::msg::ChassisMoveState out;
  const auto & s = latest_.move;
  out.stamp = rclcpp::Time(latest_.estimate_ns, RCL_ROS_TIME); out.execution_start = rclcpp::Time(
    latest_.execution_ns, RCL_ROS_TIME);
  out.phase = static_cast<uint8_t>(s.phase); out.fault_code = static_cast<uint16_t>(s.fault);
  out.progress = s.progress; out.actual_dx = s.actual.x_m; out.actual_dy = s.actual.y_m;
  out.actual_dyaw = s.actual.heading_rad; out.imu_yaw = s.imu_yaw; out.wheel_yaw = s.wheel_yaw;
  out.yaw_difference = s.imu_yaw - s.wheel_yaw; out.estimate_valid = s.estimate_valid;
  out.quality_flags = s.quality;
  return out;
}
void ChassisRuntime::complete_goal(bool deactivated)
{
  if (!handle_) {return;}
  auto result = std::make_shared<Action::Result>(); result->state = move_state();
  result->corridor_token = handle_->get_goal()->corridor_token;
  result->code =
    static_cast<uint16_t>(deactivated ||
    latest_.move.result == MoveResult::none ? MoveResult::faulted : latest_.move.result);
  if (deactivated) {
    result->state.estimate_valid = false; result->state.quality_flags |= 4U;
    result->state.phase = static_cast<uint8_t>(MovePhase::fault);
    result->state.fault_code = static_cast<uint16_t>(MoveFault::execution);
  }
  if (result->code == Action::Result::SUCCEEDED) {
    handle_->succeed(result);
  } else if (result->code == Action::Result::CANCELED) {handle_->canceled(result);} else {
    handle_->abort(result);
  }
  handle_.reset(); reserved_ = false;
}
void ChassisRuntime::publish()
{
  std::lock_guard<std::mutex> guard(endpoint_mutex_);
  if (!active_) {return;}
  drain_snapshots();
  if (latest_.response_id == request_id_) {
    if (mode_header_) {
      SetMode::Response response; response.code = latest_.response_code;
      response.state = chassis_state();
      mode_service_->send_response(*mode_header_, response); mode_header_.reset();
    }
    if (reset_header_) {
      Reset::Response response; response.code = latest_.response_code;
      response.state = chassis_state();
      reset_service_->send_response(*reset_header_, response); reset_header_.reset();
    }
  }
  if (handle_ && latest_.goal_id == goal_id_) {
    auto feedback = std::make_shared<Action::Feedback>(); feedback->state = move_state();
    feedback->corridor_token = handle_->get_goal()->corridor_token; handle_->publish_feedback(
      feedback);
    if (!latest_.move.active) {complete_goal(false);}
  }
  state_publisher_->publish(chassis_state());
}
void ChassisRuntime::deactivate()
{
  std::lock_guard<std::mutex> guard(endpoint_mutex_);
  active_ = false; navigation_generation_.store(0); handoff_.deactivate(); drain_snapshots();
  complete_goal(true); reserved_ = false;
  if (mode_header_) {
    SetMode::Response response; response.code = SetMode::Response::SWITCH_FAILED;
    response.state = chassis_state(); mode_service_->send_response(*mode_header_, response);
    mode_header_.reset();
  }
  if (reset_header_) {
    Reset::Response response; response.code = Reset::Response::RESTART_REQUIRED;
    response.state = chassis_state(); reset_service_->send_response(*reset_header_, response);
    reset_header_.reset();
  }
  timer_.reset(); action_.reset(); mode_service_.reset(); reset_service_.reset();
  state_publisher_.reset();
}
}  // namespace swerve_driver
