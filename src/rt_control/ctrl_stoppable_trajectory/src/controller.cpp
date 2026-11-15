#include "ctrl_stoppable_trajectory/controller.hpp"

#include <algorithm>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/create_server.hpp"
#include "robot_interfaces_qos/profiles.hpp"

namespace ctrl_stoppable_trajectory
{
Controller::CallbackReturn Controller::on_init()
{
  const auto result = Base::on_init();
  if (result == CallbackReturn::SUCCESS) {
    auto_declare<std::string>("bounded_stop_service", "");
  }
  return result;
}

Controller::CallbackReturn Controller::on_configure(const rclcpp_lifecycle::State & state)
{
  const auto result = Base::on_configure(state);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }
  const auto endpoint = get_node()->get_parameter("bounded_stop_service").as_string();
  // This extension writes CSP position references. Other command modes remain on JTC.
  if (
    endpoint.empty() || params_.command_interfaces != std::vector<std::string>{"position"} ||
    !params_.trajectory_start_consistency_check.enabled ||
    params_.joint_velocity_limits.size() != dof_ ||
    params_.joint_acceleration_limits.size() != dof_ ||
    std::any_of(joints_angle_wraparound_.begin(), joints_angle_wraparound_.end(), [](bool wraps) {
      return wraps;
    })) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Bounded stop requires an explicit service, position commands, model limits, bounded joints "
      "and feedback age checking");
    return CallbackReturn::ERROR;
  }
  request_.origin.resize(dof_);
  request_.direction.resize(dof_);
  cut_positions_.resize(dof_);
  stop_direction_.resize(dof_);
  // Replace callback bindings, not the public FJT schema. All callbacks use the
  // node's mutually-exclusive group, including the stop slot producer/timer.
  action_server_.reset();
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    params_.action_name.empty() ? std::string("~/follow_joint_trajectory") : params_.action_name,
    [this](const auto & id, auto goal) {
      return phase_.load(std::memory_order_acquire) == Phase::idle
               ? goal_received_callback(id, goal)
               : rclcpp_action::GoalResponse::REJECT;
    },
    [this](auto goal) {
      return phase_.load(std::memory_order_acquire) == Phase::idle
               ? goal_cancelled_callback(goal)
               : rclcpp_action::CancelResponse::REJECT;
    },
    [this](auto goal) {
      if (phase_.load(std::memory_order_acquire) != Phase::idle) {
        auto response = std::make_shared<FollowJTrajAction::Result>();
        response->error_code = FollowJTrajAction::Result::INVALID_GOAL;
        response->error_string = "Bounded stop is pending";
        goal->abort(response);
        return;
      }
      goal_accepted_callback(goal);
      normal_generation_.fetch_add(1, std::memory_order_release);
    });
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      params_.trajectory_topic.empty() ? std::string("~/joint_trajectory")
                                       : params_.trajectory_topic,
      robot_interfaces_qos::control(),
      [this](std::shared_ptr<trajectory_msgs::msg::JointTrajectory> message) {
        if (phase_.load(std::memory_order_acquire) != Phase::idle) {
          return;
        }
        topic_callback(message);
        // Only leave our hold if the base controller actually admitted the input.
        if (*traj_msg_external_point_ptr_.readFromNonRT() == message) {
          normal_generation_.fetch_add(1, std::memory_order_release);
        }
      });
  query_state_srv_.reset();
  query_state_srv_ = get_node()->create_service<control_msgs::srv::QueryTrajectoryState>(
    "~/query_state",
    [this](
      std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
      std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response) {
      if (own_reference_.load(std::memory_order_acquire)) {
        response->success = false;
      } else {
        query_state_service(request, response);
      }
    });
  stop_service_ = get_node()->create_service<Stop>(
    endpoint,
    [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Stop::Request> request) {
      receive_stop(std::move(header), std::move(request));
    });
  completion_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(), [this] { finish_request(); });
  return CallbackReturn::SUCCESS;
}

Controller::CallbackReturn Controller::on_activate(const rclcpp_lifecycle::State & state)
{
  // Never erase a response that the NRT completion timer has not delivered yet.
  if (response_pending_.load(std::memory_order_acquire)) {
    return CallbackReturn::ERROR;
  }
  const auto result = Base::on_activate(state);
  if (result == CallbackReturn::SUCCESS) {
    receiver_.clear();
    owns_reference_ = detached_goal_ = false;
    own_reference_.store(false, std::memory_order_release);
    phase_.store(Phase::idle, std::memory_order_release);
    active_.store(true, std::memory_order_release);
  }
  return result;
}

void Controller::invalidate_request()
{
  active_.store(false, std::memory_order_release);
  receiver_.clear();
  const auto phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::preparing || phase == Phase::queued || phase == Phase::braking) {
    request_.result = Stop::Response::INACTIVE;
    phase_.store(Phase::fault, std::memory_order_release);
  }
}

Controller::CallbackReturn Controller::on_deactivate(const rclcpp_lifecycle::State & state)
{
  invalidate_request();
  return Base::on_deactivate(state);
}

Controller::CallbackReturn Controller::on_cleanup(const rclcpp_lifecycle::State & state)
{
  invalidate_request();
  finish_request();
  completion_timer_.reset();
  stop_service_.reset();
  return Base::on_cleanup(state);
}

Controller::CallbackReturn Controller::on_error(const rclcpp_lifecycle::State & state)
{
  invalidate_request();
  return Base::on_error(state);
}

Controller::CallbackReturn Controller::on_shutdown(const rclcpp_lifecycle::State & state)
{
  invalidate_request();
  finish_request();
  return Base::on_shutdown(state);
}

controller_interface::return_type Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  using Return = controller_interface::return_type;
  if (!active_.load(std::memory_order_acquire)) {
    return Return::OK;
  }
  const auto phase = phase_.load(std::memory_order_acquire);
  if (phase == Phase::fault) {
    return Return::ERROR;
  }
  if (
    owns_reference_ && stopped_generation_ == normal_generation_.load(std::memory_order_acquire)) {
    return update_stop(time);
  }
  owns_reference_ = false;
  own_reference_.store(false, std::memory_order_release);
  const auto result = Base::update(time, period);
  if (result == Return::OK && phase == Phase::queued) {
    begin_stop(time);
  }
  return phase_.load(std::memory_order_acquire) == Phase::fault ? Return::ERROR : result;
}

bool Controller::feedback_fresh() const
{
  if (!has_feedback_age_state_interface_.load(std::memory_order_acquire)) {
    return false;
  }
  const double age = state_interfaces_[feedback_age_state_interface_index_].get_value();
  return std::isfinite(age) && age >= 0. &&
         age <= params_.trajectory_start_consistency_check.max_feedback_age_ms;
}

void Controller::begin_stop(const rclcpp::Time & time)
{
  const auto goal = *rt_active_goal_.readFromRT();
  if (!goal || goal->gh_->get_goal_id() != request_.id || rt_is_holding_) {
    request_.result = Stop::Response::STALE_GOAL;
    phase_.store(Phase::rejected, std::memory_order_release);
    return;
  }
  request_.result = Stop::Response::FAILED;
  const auto lead = request_.lead;
  const double direction = request_.direction[lead];
  const execution_stop::State current{
    (state_desired_.positions[lead] - request_.origin[lead]) / direction,
    state_desired_.velocities[lead] / direction, state_desired_.accelerations[lead] / direction};
  bool linear = true;
  for (std::size_t i = 0; i < dof_; ++i) {
    const double d = request_.direction[i];
    // Encoding roundoff only; tracking tolerances remain the configured JTC values.
    linear &=
      std::abs(state_desired_.positions[i] - request_.origin[i] - d * current.position) <= 1e-8;
    linear &= std::abs(state_desired_.velocities[i] - d * current.velocity) <= 1e-8;
    linear &= std::abs(state_desired_.accelerations[i] - d * current.acceleration) <= 1e-8;
  }
  if (
    !linear || !feedback_fresh() ||
    receiver_.apply(
      request_.id, goal->gh_->get_goal_id(), true, current, request_.limits, request_.length,
      time.nanoseconds()) != execution_stop::Result::accepted) {
    phase_.store(Phase::fault, std::memory_order_release);
    return;
  }
  std::copy(
    state_desired_.positions.begin(), state_desired_.positions.end(), cut_positions_.begin());
  std::copy(request_.direction.begin(), request_.direction.end(), stop_direction_.begin());
  stop_id_ = request_.id;
  cut_distance_ = current.position;
  request_.applied = time;
  request_.duration = receiver_.profile().duration();
  previous_tick_ = time.nanoseconds();
  stopped_generation_ = normal_generation_.load(std::memory_order_acquire);
  owns_reference_ = detached_goal_ = true;
  own_reference_.store(true, std::memory_order_release);
  phase_.store(Phase::braking, std::memory_order_release);
}

controller_interface::return_type Controller::update_stop(const rclcpp::Time & time)
{
  using Return = controller_interface::return_type;
  execution_stop::State reference{};
  if (
    time.nanoseconds() < previous_tick_ || !feedback_fresh() ||
    !receiver_.sample(stop_id_, true, time.nanoseconds(), reference)) {
    request_.result = Stop::Response::FAILED;
    phase_.store(Phase::fault, std::memory_order_release);
    return Return::ERROR;
  }
  previous_tick_ = time.nanoseconds();
  read_state_from_state_interfaces(state_current_);
  const auto tolerance = active_tolerances_.readFromRT();
  const double elapsed = static_cast<double>(time.nanoseconds() - receiver_.started_ns()) / 1e9;
  const bool at_rest = elapsed >= receiver_.profile().duration();
  bool tracking = true, reached = true;
  for (std::size_t i = 0; i < dof_; ++i) {
    const double d = stop_direction_[i];
    state_desired_.positions[i] = cut_positions_[i] + d * (reference.position - cut_distance_);
    state_desired_.velocities[i] = d * reference.velocity;
    state_desired_.accelerations[i] = d * reference.acceleration;
    state_error_.positions[i] = state_desired_.positions[i] - state_current_.positions[i];
    if (has_velocity_state_interface_) {
      state_error_.velocities[i] = state_desired_.velocities[i] - state_current_.velocities[i];
      tracking &= std::isfinite(state_current_.velocities[i]);
    }
    if (has_acceleration_state_interface_) {
      state_error_.accelerations[i] =
        state_desired_.accelerations[i] - state_current_.accelerations[i];
      tracking &= std::isfinite(state_current_.accelerations[i]);
    }
    tracking &= joint_trajectory_controller::check_state_tolerance_per_joint(
      state_error_, i, tolerance->state_tolerance[i], false);
    reached &= joint_trajectory_controller::check_state_tolerance_per_joint(
      state_error_, i, tolerance->goal_state_tolerance[i], false);
    tracking &= std::isfinite(state_current_.positions[i]);
  }
  const bool expired = at_rest && !reached && tolerance->goal_time_tolerance > 0. &&
                       elapsed > receiver_.profile().duration() + tolerance->goal_time_tolerance;
  if (!tracking || expired) {
    request_.result = Stop::Response::FAILED;
    phase_.store(Phase::fault, std::memory_order_release);
    return Return::ERROR;
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    joint_command_interface_[0][i].get().set_value(state_desired_.positions[i]);
  }
  last_commanded_state_ = state_desired_;
  state_desired_.time_from_start = rclcpp::Duration::from_seconds(elapsed);
  publish_stop_state();
  if (at_rest && reached && phase_.load(std::memory_order_acquire) == Phase::braking) {
    request_.result = Stop::Response::REFERENCE_STOPPED;
    phase_.store(Phase::complete, std::memory_order_release);
  }
  return Return::OK;
}

void Controller::publish_stop_state()
{
  admission_snapshot_sequence_.fetch_add(1, std::memory_order_acq_rel);
  for (std::size_t i = 0; i < dof_; ++i) {
    admission_position_snapshot_[i].store(state_current_.positions[i], std::memory_order_relaxed);
    const auto & splice = params_.open_loop_control ? last_commanded_state_ : state_current_;
    admission_splice_position_snapshot_[i].store(splice.positions[i], std::memory_order_relaxed);
    admission_splice_velocity_snapshot_[i].store(
      splice.velocities.empty() ? 0. : splice.velocities[i], std::memory_order_relaxed);
  }
  admission_feedback_age_ms_.store(
    state_interfaces_[feedback_age_state_interface_index_].get_value(), std::memory_order_relaxed);
  admission_snapshot_sequence_.fetch_add(1, std::memory_order_release);
  publish_state(state_desired_, state_current_, state_error_);
}
void Controller::receive_stop(
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Stop::Request> request)
{
  Stop::Response response;
  const auto goal = *rt_active_goal_.readFromNonRT();
  if (!active_.load(std::memory_order_acquire)) {
    response.status = Stop::Response::INACTIVE;
  } else if (phase_.load(std::memory_order_acquire) != Phase::idle) {
    response.status = Stop::Response::BUSY;
  } else if (!goal || goal->gh_->get_goal_id() != request->goal_id.uuid) {
    response.status = Stop::Response::STALE_GOAL;
  } else if (
    !std::isfinite(request->velocity) || request->velocity <= 0. ||
    !std::isfinite(request->acceleration) || request->acceleration <= 0. ||
    !std::isfinite(request->jerk) || request->jerk <= 0.) {
    response.status = Stop::Response::INVALID;
  } else {
    auto expected = Phase::idle;
    if (!phase_.compare_exchange_strong(expected, Phase::preparing, std::memory_order_acq_rel)) {
      response.status = Stop::Response::BUSY;
      stop_service_->send_response(*header, response);
      return;
    }
    bool supported = prepare_line(*goal->gh_->get_goal());
    for (std::size_t i = 0; supported && i < dof_; ++i) {
      const double d = std::abs(request_.direction[i]);
      supported &= request->velocity * d <= params_.joint_velocity_limits[i] + 1e-10;
      supported &= request->acceleration * d <= params_.joint_acceleration_limits[i] + 1e-10;
    }
    if (!supported) {
      expected = Phase::preparing;
      phase_.compare_exchange_strong(expected, Phase::idle, std::memory_order_acq_rel);
      response.status = Stop::Response::INVALID;
      stop_service_->send_response(*header, response);
      return;
    }
    request_.id = request->goal_id.uuid;
    request_.limits = {request->velocity, request->acceleration, request->jerk};
    request_.goal = goal;
    request_.header = std::move(header);
    response_pending_.store(true, std::memory_order_release);
    request_.duration = 0.;
    request_.applied = rclcpp::Time(0, 0, RCL_ROS_TIME);
    detached_goal_ = false;
    expected = Phase::preparing;
    phase_.compare_exchange_strong(expected, Phase::queued, std::memory_order_acq_rel);
    return;
  }
  response.message = "Stop request rejected before execution; no stop was applied";
  stop_service_->send_response(*header, response);
}

bool Controller::prepare_line(const FollowJTrajAction::Goal & goal)
{
  const auto & trajectory = goal.trajectory;
  if (trajectory.joint_names.size() != dof_ || trajectory.points.size() < 2) {
    return false;
  }
  std::vector<std::size_t> index(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    const auto found =
      std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), params_.joints[i]);
    if (found == trajectory.joint_names.end()) {
      return false;
    }
    index[i] = static_cast<std::size_t>(found - trajectory.joint_names.begin());
  }
  request_.length = 0.;
  request_.lead = 0;
  for (const auto & point : trajectory.points) {
    if (
      point.positions.size() != dof_ || point.velocities.size() != dof_ ||
      point.accelerations.size() != dof_) {
      return false;
    }
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    request_.origin[i] = trajectory.points.front().positions[index[i]];
    request_.direction[i] = trajectory.points.back().positions[index[i]] - request_.origin[i];
    if (std::abs(request_.direction[i]) > request_.length) {
      request_.length = std::abs(request_.direction[i]);
      request_.lead = i;
    }
  }
  if (request_.length <= 1e-10) {
    return false;
  }
  for (double & value : request_.direction) {
    value /= request_.length;
  }
  const auto lead = index[request_.lead];
  const double sign = request_.direction[request_.lead];
  double previous = -1e-8;
  for (const auto & point : trajectory.points) {
    const double s = (point.positions[lead] - request_.origin[request_.lead]) / sign;
    const double v = point.velocities[lead] / sign, a = point.accelerations[lead] / sign;
    if (
      !std::isfinite(s) || !std::isfinite(v) || !std::isfinite(a) || s < previous - 1e-8 ||
      s > request_.length + 1e-8 || v < -1e-8) {
      return false;
    }
    previous = s;
    for (std::size_t i = 0; i < dof_; ++i) {
      const double d = request_.direction[i];
      // Negated comparisons also reject NaN. This verifies the path, not tracking.
      if (
        !(std::abs(point.positions[index[i]] - request_.origin[i] - d * s) <= 1e-8) ||
        !(std::abs(point.velocities[index[i]] - d * v) <= 1e-8) ||
        !(std::abs(point.accelerations[index[i]] - d * a) <= 1e-8)) {
        return false;
      }
    }
  }
  return true;
}

void Controller::finish_request()
{
  auto phase = phase_.load(std::memory_order_acquire);
  if (!active_.load(std::memory_order_acquire) && phase == Phase::queued) {
    request_.result = Stop::Response::INACTIVE;
    phase_.store(Phase::fault, std::memory_order_release);
    phase = Phase::fault;
  }
  if (
    (phase != Phase::complete && phase != Phase::rejected && phase != Phase::fault) ||
    !request_.header) {
    return;
  }
  if (
    (detached_goal_ || phase == Phase::fault) && request_.goal && request_.goal->gh_->is_active()) {
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = phase == Phase::complete
                             ? "Original target interrupted by bounded stop"
                             : "Bounded stop failed or controller deactivated";
    request_.goal->setAborted(result);
    request_.goal->runNonRealtime();
    const auto current = *rt_active_goal_.readFromNonRT();
    if (current == request_.goal) {
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      rt_has_pending_goal_ = false;
    }
  }
  Stop::Response response;
  response.status = request_.result;
  response.message = response.status == Stop::Response::REFERENCE_STOPPED
                       ? "Reference stopped within configured tracking tolerances; physical "
                         "standstill is not certified"
                       : "Stop did not complete; inspect controller state before further motion";
  response.applied_at = request_.applied;
  response.braking_duration = request_.duration;
  stop_service_->send_response(*request_.header, response);
  request_.header.reset();
  request_.goal.reset();
  response_pending_.store(false, std::memory_order_release);
  if (phase != Phase::fault) {
    // A feedback fault can arrive while this NRT callback sends the response.
    // Never clear a newer RT fault when releasing the completed request slot.
    phase_.compare_exchange_strong(phase, Phase::idle, std::memory_order_acq_rel);
  }
}
}  // namespace ctrl_stoppable_trajectory

PLUGINLIB_EXPORT_CLASS(
  ctrl_stoppable_trajectory::Controller, controller_interface::ControllerInterface)
