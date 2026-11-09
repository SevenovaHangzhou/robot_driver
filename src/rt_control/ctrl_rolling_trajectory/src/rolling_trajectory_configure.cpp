#include "ctrl_rolling_trajectory/rolling_trajectory_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/subscription_options.hpp"
#include "robot_interfaces_qos/profiles.hpp"

namespace ctrl_rolling_trajectory
{

namespace
{

constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000U;

// Every value comes from the composed artifact (motion policy and robot model); a missing one
// leaves its sentinel here and is rejected at configure.
constexpr std::array<std::string_view, 21> kManagedParameterNames = {
  "joints",
  "joint_position_lower",
  "joint_position_upper",
  "joint_velocity_limits",
  "joint_acceleration_limits",
  "position_margin",
  "buffer_capacity",
  "max_horizon_ms",
  "update_timeout_ms",
  "replace_lead_ms",
  "state_publish_period_ms",
  "nominal_controller_period_ms",
  "maximum_controller_period_ms",
  "one_cycle_detection_guard_ms",
  "stop_time_growth_guard_ms",
  "non_rt_to_rt_visibility_guard_ms",
  "period_quantization_guard_ms",
  "feedback_age_limit_ms",
  "takeover_tolerances",
  "splice_position_tolerances",
  "splice_velocity_tolerances"};

bool isManagedParameter(std::string_view name) noexcept
{
  return std::find(kManagedParameterNames.begin(), kManagedParameterNames.end(), name) !=
         kManagedParameterNames.end();
}

bool millisecondsToNanoseconds(std::int64_t milliseconds, std::uint64_t & nanoseconds) noexcept
{
  if (milliseconds <= 0) {
    return false;
  }
  const auto value = static_cast<std::uint64_t>(milliseconds);
  if (value > std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerMillisecond) {
    return false;
  }
  nanoseconds = value * kNanosecondsPerMillisecond;
  return true;
}

}  // namespace

controller_interface::CallbackReturn RollingTrajectoryController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  for (const char * name :
       {"joint_position_lower", "joint_position_upper", "joint_velocity_limits",
        "joint_acceleration_limits", "takeover_tolerances", "splice_position_tolerances",
        "splice_velocity_tolerances"}) {
    auto_declare<std::vector<double>>(name, {});
  }
  for (const char * name :
       {"buffer_capacity", "max_horizon_ms", "update_timeout_ms", "replace_lead_ms",
        "state_publish_period_ms", "nominal_controller_period_ms", "maximum_controller_period_ms",
        "one_cycle_detection_guard_ms", "stop_time_growth_guard_ms",
        "non_rt_to_rt_visibility_guard_ms", "period_quantization_guard_ms"}) {
    auto_declare<std::int64_t>(name, 0);
  }
  auto_declare<double>("position_margin", -1.0);
  auto_declare<double>("feedback_age_limit_ms", 0.0);
  parameters_frozen_.store(false, std::memory_order_relaxed);
  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      if (!parameters_frozen_.load(std::memory_order_acquire)) {
        return result;
      }
      for (const auto & parameter : parameters) {
        if (isManagedParameter(parameter.get_name())) {
          result.successful = false;
          result.reason = "rolling runtime parameters are frozen after configure";
          break;
        }
      }
      return result;
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

bool RollingTrajectoryController::loadRuntimeConfiguration(std::string & error)
{
  RuntimeConfiguration candidate;
  if (!parseJointNames(
        get_node()->get_parameter("joints").as_string_array(), candidate.joints, error)) {
    return false;
  }
  const std::int64_t capacity = get_node()->get_parameter("buffer_capacity").as_int();
  if (capacity < 2 || capacity > static_cast<std::int64_t>(kTransportMaxPoints)) {
    error = "buffer_capacity must be in [2, 256]";
    return false;
  }
  candidate.buffer_capacity = static_cast<std::size_t>(capacity);

  const auto read_duration = [this, &error](const char * name, std::uint64_t & destination) {
    if (!millisecondsToNanoseconds(get_node()->get_parameter(name).as_int(), destination)) {
      error = std::string(name) + " must be a positive, representable integer millisecond value";
      return false;
    }
    return true;
  };
  if (
    !read_duration("max_horizon_ms", candidate.max_horizon_ns) ||
    !read_duration("update_timeout_ms", candidate.update_timeout_ns) ||
    !read_duration("replace_lead_ms", candidate.replace_lead_ns) ||
    !read_duration("state_publish_period_ms", candidate.state_publish_period_ns) ||
    !read_duration("nominal_controller_period_ms", candidate.nominal_controller_period_ns) ||
    !read_duration("maximum_controller_period_ms", candidate.maximum_controller_period_ns) ||
    !read_duration(
      "one_cycle_detection_guard_ms", candidate.scheduling_guard.one_cycle_detection_ns) ||
    !read_duration("stop_time_growth_guard_ms", candidate.scheduling_guard.stop_time_growth_ns) ||
    !read_duration(
      "non_rt_to_rt_visibility_guard_ms", candidate.scheduling_guard.non_rt_to_rt_visibility_ns) ||
    !read_duration(
      "period_quantization_guard_ms", candidate.scheduling_guard.period_quantization_ns)) {
    return false;
  }
  if (
    candidate.maximum_controller_period_ns < candidate.nominal_controller_period_ns ||
    candidate.update_timeout_ns < candidate.maximum_controller_period_ns ||
    candidate.replace_lead_ns > candidate.max_horizon_ns) {
    error = "controller periods, update timeout, replace lead, and horizon are inconsistent";
    return false;
  }

  candidate.feedback_age_limit_ms = get_node()->get_parameter("feedback_age_limit_ms").as_double();
  if (!std::isfinite(candidate.feedback_age_limit_ms) || candidate.feedback_age_limit_ms <= 0.0) {
    error = "feedback_age_limit_ms must be finite and positive";
    return false;
  }

  const auto read_axis_values = [this, &error](
                                  const char * name, std::array<double, kAxisCount> & destination) {
    const std::vector<double> values = get_node()->get_parameter(name).as_double_array();
    if (values.size() != kAxisCount) {
      error = std::string(name) + " must contain exactly 14 values in protocol axis order";
      return false;
    }
    std::copy(values.begin(), values.end(), destination.begin());
    return true;
  };
  if (
    !read_axis_values("takeover_tolerances", candidate.takeover_tolerances) ||
    !read_axis_values("splice_position_tolerances", candidate.splice_position_tolerances) ||
    !read_axis_values("splice_velocity_tolerances", candidate.splice_velocity_tolerances)) {
    return false;
  }

  // Limits of the robot model; the brake decelerates at the acceleration limit (user 2026-11-05).
  std::array<double, kAxisCount> lower{};
  std::array<double, kAxisCount> upper{};
  std::array<double, kAxisCount> velocity{};
  std::array<double, kAxisCount> acceleration{};
  if (
    !read_axis_values("joint_position_lower", lower) ||
    !read_axis_values("joint_position_upper", upper) ||
    !read_axis_values("joint_velocity_limits", velocity) ||
    !read_axis_values("joint_acceleration_limits", acceleration)) {
    return false;
  }
  const double margin = get_node()->get_parameter("position_margin").as_double();
  DynamicEnvelope envelope;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    envelope.axes[axis] = AxisEnvelope{lower[axis],
                                       upper[axis],
                                       velocity[axis],
                                       velocity[axis],
                                       acceleration[axis],
                                       acceleration[axis],
                                       acceleration[axis],
                                       acceleration[axis],
                                       margin,
                                       margin};
  }

  BufferConfiguration buffer_configuration;
  buffer_configuration.capacity = candidate.buffer_capacity;
  buffer_configuration.max_horizon_ns = candidate.max_horizon_ns;
  buffer_configuration.splice_position_tolerance = candidate.splice_position_tolerances;
  buffer_configuration.splice_velocity_tolerance = candidate.splice_velocity_tolerances;
  StopTrajectory stop_trajectory;
  if (!buffer_.configure(buffer_configuration, envelope) || !stop_trajectory.configure(envelope)) {
    error = "buffer configuration or robot-model limits are invalid";
    return false;
  }
  envelope_ = envelope;
  rt_stop_trajectory_ = stop_trajectory;
  runtime_configuration_ = candidate;
  return true;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  active_.store(false, std::memory_order_release);
  std::string configuration_error;
  if (!loadRuntimeConfiguration(configuration_error)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid rolling runtime configuration: %s",
      configuration_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_publisher_ = get_node()->create_publisher<StateMessage>(
    "/rt/arms/rolling/state", robot_interfaces_qos::rolling_state());
  state_timer_ = get_node()->create_wall_timer(
    std::chrono::nanoseconds(runtime_configuration_.state_publish_period_ns),
    [this]() { (void)publishPublicState(); }, callback_group_);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  batch_subscription_ = get_node()->create_subscription<BatchMessage>(
    "/rt/arms/rolling/target_batch", robot_interfaces_qos::rolling_command(),
    std::bind(&RollingTrajectoryController::handleBatch, this, std::placeholders::_1),
    subscription_options);

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_positions_[axis].store(0.0, std::memory_order_relaxed);
    desired_positions_[axis].store(0.0, std::memory_order_relaxed);
  }
  observed_feedback_age_ms_.store(0.0, std::memory_order_relaxed);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_relaxed);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_relaxed);
  execution_time_ns_.store(0U, std::memory_order_relaxed);
  active_generation_.store(0U, std::memory_order_relaxed);
  buffered_until_ns_.store(0U, std::memory_order_relaxed);
  replaceable_from_ns_.store(0U, std::memory_order_relaxed);
  prime_start_ns_.store(0, std::memory_order_relaxed);
  session_epoch_.store(0U, std::memory_order_relaxed);
  session_publication_floor_.store(1U, std::memory_order_relaxed);
  initial_handoff_gate_.clear(std::memory_order_relaxed);
  configured_ = true;
  parameters_frozen_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ctrl_rolling_trajectory
