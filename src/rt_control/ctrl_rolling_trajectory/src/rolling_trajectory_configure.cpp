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
#include "ctrl_rolling_trajectory/envelope_loader.hpp"

namespace ctrl_rolling_trajectory
{

namespace
{

constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000U;
constexpr std::int64_t kDefaultBufferCapacity = 64;
constexpr std::int64_t kDefaultMaxHorizonMs = 600;
constexpr std::int64_t kDefaultRequiredInitialHorizonMs = 500;
constexpr std::int64_t kDefaultUpdateTimeoutMs = 200;
constexpr std::int64_t kDefaultReplaceLeadMs = 16;
constexpr std::int64_t kDefaultStatePublishPeriodMs = 20;
constexpr std::int64_t kDefaultPrimeTimeoutMs = 100;
constexpr std::int64_t kDefaultNominalControllerPeriodMs = 4;
constexpr std::int64_t kDefaultMaximumControllerPeriodMs = 8;
constexpr std::int64_t kDefaultSchedulingGuardMs = 4;
constexpr double kDefaultOpenFeedbackAgeLimitMs = 500.0;

constexpr std::array<std::string_view, 24> kManagedParameterNames = {
  "configuration_source",
  "joints",
  "allow_test_only_configuration",
  "allow_provisional_configuration",
  "envelope_file",
  "limits_version",
  "axis_set_hash",
  "buffer_capacity",
  "max_horizon_ms",
  "required_initial_horizon_ms",
  "update_timeout_ms",
  "replace_lead_ms",
  "state_publish_period_ms",
  "prime_timeout_ms",
  "nominal_controller_period_ms",
  "maximum_controller_period_ms",
  "one_cycle_detection_guard_ms",
  "stop_time_growth_guard_ms",
  "non_rt_to_rt_visibility_guard_ms",
  "period_quantization_guard_ms",
  "open_feedback_age_limit_ms",
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

constexpr std::array<std::uint8_t, 32> kTestOnlyLimitsVersion = {
  0xa1U, 0xc3U, 0x34U, 0x03U, 0xa1U, 0x37U, 0x0eU, 0xf8U, 0x21U, 0x05U, 0xc8U,
  0x48U, 0x8bU, 0x58U, 0x68U, 0x74U, 0xacU, 0xa7U, 0x7cU, 0xa3U, 0x29U, 0x6eU,
  0xeaU, 0xceU, 0xbeU, 0x53U, 0xd2U, 0x41U, 0x3bU, 0xc1U, 0x12U, 0x3bU};

DynamicEnvelope makeTestOnlyEnvelope() noexcept
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kTestOnly;
  envelope.limits_version = kTestOnlyLimitsVersion;
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -10.0;
    axis.position_upper = 10.0;
    axis.velocity_positive = 10.0;
    axis.velocity_negative = 10.0;
    axis.acceleration_positive = 10.0;
    axis.acceleration_negative = 10.0;
    axis.stop_acceleration_positive = 10.0;
    axis.stop_acceleration_negative = 10.0;
    axis.position_margin_lower = 0.01;
    axis.position_margin_upper = 0.01;
  }
  return envelope;
}

}  // namespace

controller_interface::CallbackReturn RollingTrajectoryController::on_init()
{
  auto_declare<std::string>("configuration_source", "unconfigured");
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<bool>("allow_test_only_configuration", false);
  auto_declare<bool>("allow_provisional_configuration", false);
  auto_declare<std::string>("envelope_file", "");
  auto_declare<std::string>("limits_version", "");
  auto_declare<std::string>("axis_set_hash", "");
  auto_declare<std::int64_t>("buffer_capacity", kDefaultBufferCapacity);
  auto_declare<std::int64_t>("max_horizon_ms", kDefaultMaxHorizonMs);
  auto_declare<std::int64_t>("required_initial_horizon_ms", kDefaultRequiredInitialHorizonMs);
  auto_declare<std::int64_t>("update_timeout_ms", kDefaultUpdateTimeoutMs);
  auto_declare<std::int64_t>("replace_lead_ms", kDefaultReplaceLeadMs);
  auto_declare<std::int64_t>("state_publish_period_ms", kDefaultStatePublishPeriodMs);
  auto_declare<std::int64_t>("prime_timeout_ms", kDefaultPrimeTimeoutMs);
  auto_declare<std::int64_t>("nominal_controller_period_ms", kDefaultNominalControllerPeriodMs);
  auto_declare<std::int64_t>("maximum_controller_period_ms", kDefaultMaximumControllerPeriodMs);
  auto_declare<std::int64_t>("one_cycle_detection_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<std::int64_t>("stop_time_growth_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<std::int64_t>("non_rt_to_rt_visibility_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<std::int64_t>("period_quantization_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<double>("open_feedback_age_limit_ms", kDefaultOpenFeedbackAgeLimitMs);
  auto_declare<std::vector<double>>("takeover_tolerances", {});
  auto_declare<std::vector<double>>("splice_position_tolerances", {});
  auto_declare<std::vector<double>>("splice_velocity_tolerances", {});
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
    !read_duration("required_initial_horizon_ms", candidate.required_initial_horizon_ns) ||
    !read_duration("update_timeout_ms", candidate.update_timeout_ns) ||
    !read_duration("replace_lead_ms", candidate.replace_lead_ns) ||
    !read_duration("state_publish_period_ms", candidate.state_publish_period_ns) ||
    !read_duration("prime_timeout_ms", candidate.prime_timeout_ns) ||
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

  if (candidate.required_initial_horizon_ns > candidate.max_horizon_ns) {
    error = "required_initial_horizon_ms must not exceed max_horizon_ms";
    return false;
  }
  constexpr std::uint64_t kMinimumUpdateTimeoutNs = 100'000'000U;
  constexpr std::uint64_t kMaximumUpdateTimeoutNs = 500'000'000U;
  if (
    candidate.update_timeout_ns < kMinimumUpdateTimeoutNs ||
    candidate.update_timeout_ns > kMaximumUpdateTimeoutNs) {
    error = "update_timeout_ms must be in [100, 500]";
    return false;
  }
  if (
    candidate.maximum_controller_period_ns < candidate.nominal_controller_period_ns ||
    candidate.update_timeout_ns < candidate.maximum_controller_period_ns ||
    candidate.prime_timeout_ns < candidate.maximum_controller_period_ns ||
    candidate.replace_lead_ns > candidate.max_horizon_ns) {
    error = "controller periods, timeouts, replace lead, and horizon are inconsistent";
    return false;
  }

  std::uint64_t total_guard_ns = 0U;
  for (const std::uint64_t guard_ns :
       {candidate.scheduling_guard.one_cycle_detection_ns,
        candidate.scheduling_guard.stop_time_growth_ns,
        candidate.scheduling_guard.non_rt_to_rt_visibility_ns,
        candidate.scheduling_guard.period_quantization_ns}) {
    if (!addTime(total_guard_ns, guard_ns, total_guard_ns)) {
      error = "scheduling guard sum overflows";
      return false;
    }
  }
  if (total_guard_ns > candidate.required_initial_horizon_ns) {
    error = "scheduling guard sum must not exceed required_initial_horizon_ms";
    return false;
  }

  candidate.open_feedback_age_limit_ms =
    get_node()->get_parameter("open_feedback_age_limit_ms").as_double();
  if (
    !std::isfinite(candidate.open_feedback_age_limit_ms) ||
    candidate.open_feedback_age_limit_ms <= 0.0) {
    error = "open_feedback_age_limit_ms must be finite and positive";
    return false;
  }

  const auto read_axis_values = [this, &error](
                                  const char * name, std::array<double, kAxisCount> & destination) {
    const std::vector<double> values = get_node()->get_parameter(name).as_double_array();
    if (values.size() != kAxisCount) {
      error = std::string(name) + " must contain exactly 14 values in protocol axis order";
      return false;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      if (!std::isfinite(values[axis]) || values[axis] < 0.0) {
        error = std::string(name) + " contains a non-finite or negative value";
        return false;
      }
      destination[axis] = values[axis];
    }
    return true;
  };
  if (
    !read_axis_values("takeover_tolerances", candidate.takeover_tolerances) ||
    !read_axis_values("splice_position_tolerances", candidate.splice_position_tolerances) ||
    !read_axis_values("splice_velocity_tolerances", candidate.splice_velocity_tolerances)) {
    return false;
  }

  runtime_configuration_ = candidate;
  return true;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  active_.store(false, std::memory_order_release);
  const std::string source = get_node()->get_parameter("configuration_source").as_string();
  const bool allow_test_only = get_node()->get_parameter("allow_test_only_configuration").as_bool();
  const bool allow_provisional =
    get_node()->get_parameter("allow_provisional_configuration").as_bool();
  const std::string envelope_file = get_node()->get_parameter("envelope_file").as_string();
  const std::string limits_version = get_node()->get_parameter("limits_version").as_string();
  if (
    (source == "test_only" && !allow_test_only) ||
    (source == "provisional" && !allow_provisional) ||
    (source != "test_only" && source != "provisional")) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller configuration source '%s' lacks its explicit authority", source.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string configuration_error;
  if (!loadRuntimeConfiguration(configuration_error)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid rolling runtime configuration: %s",
      configuration_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!parseSha256Hex(
        get_node()->get_parameter("axis_set_hash").as_string(), axis_set_hash_,
        configuration_error)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid rolling axis_set_hash: %s", configuration_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  BufferConfiguration buffer_configuration;
  buffer_configuration.capacity = runtime_configuration_.buffer_capacity;
  buffer_configuration.required_initial_horizon_ns =
    runtime_configuration_.required_initial_horizon_ns;
  buffer_configuration.max_horizon_ns = runtime_configuration_.max_horizon_ns;
  buffer_configuration.splice_position_tolerance =
    runtime_configuration_.splice_position_tolerances;
  buffer_configuration.splice_velocity_tolerance =
    runtime_configuration_.splice_velocity_tolerances;
  if (source == "test_only") {
    if (!envelope_file.empty() || !limits_version.empty()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Test-only rolling configuration must not provide an envelope file or limits_version");
      return controller_interface::CallbackReturn::ERROR;
    }
    envelope_ = makeTestOnlyEnvelope();
  } else {
    std::string envelope_error;
    std::array<std::uint8_t, 32> version{};
    if (
      !parseSha256Hex(limits_version, version, envelope_error) ||
      !loadProvisionalEnvelope(
        envelope_file, runtime_configuration_.joints, version, envelope_, envelope_error)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to load provisional rolling envelope: %s",
        envelope_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Using ESTIMATED_NOT_MEASURED provisional rolling envelope '%s'; "
      "this does not authorize hardware motion or replace bench measurement",
      envelope_file.c_str());
  }
  if (
    !buffer_.configure(buffer_configuration) ||
    !buffer_.configureLimits(envelope_, allow_test_only, allow_provisional)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling buffer envelope");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(envelope_, allow_test_only, allow_provisional)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }

  callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_publisher_ = get_node()->create_publisher<StateMessage>(
    "/rt/arms/rolling/state", robot_interfaces_qos::rolling_state());
  state_timer_ = get_node()->create_wall_timer(
    std::chrono::nanoseconds(runtime_configuration_.state_publish_period_ns),
    [this]() { (void)publishPublicState(); }, callback_group_);
  open_service_ = get_node()->create_service<OpenService>(
    "/rt/arms/rolling/open_session",
    std::bind(
      &RollingTrajectoryController::handleOpen, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  close_service_ = get_node()->create_service<CloseService>(
    "/rt/arms/rolling/close_session",
    std::bind(
      &RollingTrajectoryController::handleClose, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  update_subscription_ = get_node()->create_subscription<BatchMessage>(
    "/rt/arms/rolling/target_batch", robot_interfaces_qos::rolling_command(),
    std::bind(&RollingTrajectoryController::handleUpdate, this, std::placeholders::_1),
    subscription_options);
  mode_result_subscription_ = get_node()->create_subscription<ModeResultMessage>(
    "/rt/internal/joint_control/mode_result", robot_interfaces_qos::state(),
    std::bind(&RollingTrajectoryController::handleModeResult, this, std::placeholders::_1),
    subscription_options);

  hold_positions_.fill(0.0);
  last_mode_result_ = ModeResultMessage{};
  last_mode_result_sequence_ = 0U;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_positions_[axis].store(0.0, std::memory_order_relaxed);
    desired_positions_[axis].store(0.0, std::memory_order_relaxed);
    desired_velocities_[axis].store(0.0, std::memory_order_relaxed);
  }
  observed_feedback_age_ms_.store(0.0, std::memory_order_relaxed);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_relaxed);
  stop_reason_.store(static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_relaxed);
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_relaxed);
  execution_time_ns_.store(0U, std::memory_order_relaxed);
  active_generation_.store(0U, std::memory_order_relaxed);
  buffered_until_ns_.store(0U, std::memory_order_relaxed);
  replaceable_from_ns_.store(0U, std::memory_order_relaxed);
  prime_start_time_ns_.store(0U, std::memory_order_relaxed);
  session_epoch_.store(0U, std::memory_order_relaxed);
  session_publication_floor_.store(1U, std::memory_order_relaxed);
  rt_state_version_.store(0U, std::memory_order_relaxed);
  rt_state_session_epoch_.store(0U, std::memory_order_relaxed);
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_relaxed);
  timeout_count_.store(0U, std::memory_order_relaxed);
  invariant_failure_count_.store(0U, std::memory_order_relaxed);
  has_accepted_update_.store(false, std::memory_order_relaxed);
  initial_handoff_gate_.clear(std::memory_order_relaxed);
  configured_ = true;
  parameters_frozen_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ctrl_rolling_trajectory
