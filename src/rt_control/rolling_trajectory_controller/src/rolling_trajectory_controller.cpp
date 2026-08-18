#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_options.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "rolling_trajectory_controller/envelope_loader.hpp"
#include "rolling_trajectory_controller/service_error.hpp"

namespace rolling_trajectory_controller
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

constexpr std::array<std::string_view, 21> kManagedParameterNames = {
  "configuration_source",
  "allow_test_only_configuration",
  "allow_provisional_configuration",
  "envelope_file",
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

constexpr std::array<std::uint8_t, 32> kAxisSetHash = {
  0x25U, 0xc6U, 0xe8U, 0x2bU, 0xf5U, 0x05U, 0xcaU, 0x9eU,
  0xb9U, 0x9dU, 0xb1U, 0xc6U, 0x45U, 0xabU, 0x75U, 0xd7U,
  0xecU, 0xdeU, 0x01U, 0x53U, 0xfaU, 0xafU, 0x6aU, 0x74U,
  0x92U, 0xc6U, 0x21U, 0x0cU, 0x4dU, 0x36U, 0x25U, 0x26U};

constexpr std::array<std::uint8_t, 32> kTestOnlyLimitsVersion = {
  0xa1U, 0xc3U, 0x34U, 0x03U, 0xa1U, 0x37U, 0x0eU, 0xf8U,
  0x21U, 0x05U, 0xc8U, 0x48U, 0x8bU, 0x58U, 0x68U, 0x74U,
  0xacU, 0xa7U, 0x7cU, 0xa3U, 0x29U, 0x6eU, 0xeaU, 0xceU,
  0xbeU, 0x53U, 0xd2U, 0x41U, 0x3bU, 0xc1U, 0x12U, 0x3bU};

std::uint64_t splitMix64(std::uint64_t & state) noexcept
{
  state += 0x9e3779b97f4a7c15ULL;
  std::uint64_t value = state;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

class BlockingFlagGuard
{
public:
  explicit BlockingFlagGuard(std::atomic_flag & flag) noexcept
  : flag_(flag)
  {
    while (flag_.test_and_set(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  }

  ~BlockingFlagGuard() noexcept
  {
    flag_.clear(std::memory_order_release);
  }

  BlockingFlagGuard(const BlockingFlagGuard &) = delete;
  BlockingFlagGuard & operator=(const BlockingFlagGuard &) = delete;

private:
  std::atomic_flag & flag_;
};

class OptionalRtFlagGuard
{
public:
  OptionalRtFlagGuard(std::atomic_flag & flag, bool required) noexcept
  : flag_(flag), required_(required), owns_(!required || !flag_.test_and_set(
        std::memory_order_acquire))
  {
  }

  ~OptionalRtFlagGuard() noexcept
  {
    if (required_ && owns_) {
      flag_.clear(std::memory_order_release);
    }
  }

  OptionalRtFlagGuard(const OptionalRtFlagGuard &) = delete;
  OptionalRtFlagGuard & operator=(const OptionalRtFlagGuard &) = delete;

  [[nodiscard]] bool mayProceed() const noexcept
  {
    return owns_;
  }

private:
  std::atomic_flag & flag_;
  bool required_{false};
  bool owns_{false};
};

class RtStateVersionGuard
{
public:
  explicit RtStateVersionGuard(std::atomic<std::uint64_t> & version) noexcept
  : version_(version)
  {
    version_.fetch_add(1U, std::memory_order_acq_rel);
  }

  ~RtStateVersionGuard() noexcept
  {
    version_.fetch_add(1U, std::memory_order_release);
  }

  RtStateVersionGuard(const RtStateVersionGuard &) = delete;
  RtStateVersionGuard & operator=(const RtStateVersionGuard &) = delete;

private:
  std::atomic<std::uint64_t> & version_;
};

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
  auto_declare<bool>("allow_test_only_configuration", false);
  auto_declare<bool>("allow_provisional_configuration", false);
  auto_declare<std::string>("envelope_file", "");
  auto_declare<std::int64_t>("buffer_capacity", kDefaultBufferCapacity);
  auto_declare<std::int64_t>("max_horizon_ms", kDefaultMaxHorizonMs);
  auto_declare<std::int64_t>(
    "required_initial_horizon_ms", kDefaultRequiredInitialHorizonMs);
  auto_declare<std::int64_t>("update_timeout_ms", kDefaultUpdateTimeoutMs);
  auto_declare<std::int64_t>("replace_lead_ms", kDefaultReplaceLeadMs);
  auto_declare<std::int64_t>("state_publish_period_ms", kDefaultStatePublishPeriodMs);
  auto_declare<std::int64_t>("prime_timeout_ms", kDefaultPrimeTimeoutMs);
  auto_declare<std::int64_t>(
    "nominal_controller_period_ms", kDefaultNominalControllerPeriodMs);
  auto_declare<std::int64_t>(
    "maximum_controller_period_ms", kDefaultMaximumControllerPeriodMs);
  auto_declare<std::int64_t>("one_cycle_detection_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<std::int64_t>("stop_time_growth_guard_ms", kDefaultSchedulingGuardMs);
  auto_declare<std::int64_t>(
    "non_rt_to_rt_visibility_guard_ms", kDefaultSchedulingGuardMs);
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
  const std::int64_t capacity = get_node()->get_parameter("buffer_capacity").as_int();
  if (capacity < 2 || capacity > static_cast<std::int64_t>(kTransportMaxPoints)) {
    error = "buffer_capacity must be in [2, 256]";
    return false;
  }
  candidate.buffer_capacity = static_cast<std::size_t>(capacity);

  const auto read_duration = [this, &error](
    const char * name, std::uint64_t & destination) {
      if (!millisecondsToNanoseconds(get_node()->get_parameter(name).as_int(), destination)) {
        error = std::string(name) + " must be a positive, representable integer millisecond value";
        return false;
      }
      return true;
    };
  if (
    !read_duration("max_horizon_ms", candidate.max_horizon_ns) ||
    !read_duration(
      "required_initial_horizon_ms", candidate.required_initial_horizon_ns) ||
    !read_duration("update_timeout_ms", candidate.update_timeout_ns) ||
    !read_duration("replace_lead_ms", candidate.replace_lead_ns) ||
    !read_duration("state_publish_period_ms", candidate.state_publish_period_ns) ||
    !read_duration("prime_timeout_ms", candidate.prime_timeout_ns) ||
    !read_duration(
      "nominal_controller_period_ms", candidate.nominal_controller_period_ns) ||
    !read_duration(
      "maximum_controller_period_ms", candidate.maximum_controller_period_ns) ||
    !read_duration(
      "one_cycle_detection_guard_ms", candidate.scheduling_guard.one_cycle_detection_ns) ||
    !read_duration(
      "stop_time_growth_guard_ms", candidate.scheduling_guard.stop_time_growth_ns) ||
    !read_duration(
      "non_rt_to_rt_visibility_guard_ms",
      candidate.scheduling_guard.non_rt_to_rt_visibility_ns) ||
    !read_duration(
      "period_quantization_guard_ms",
      candidate.scheduling_guard.period_quantization_ns))
  {
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
    candidate.update_timeout_ns > kMaximumUpdateTimeoutNs)
  {
    error = "update_timeout_ms must be in [100, 500]";
    return false;
  }
  if (
    candidate.maximum_controller_period_ns < candidate.nominal_controller_period_ns ||
    candidate.update_timeout_ns < candidate.maximum_controller_period_ns ||
    candidate.prime_timeout_ns < candidate.maximum_controller_period_ns ||
    candidate.replace_lead_ns > candidate.max_horizon_ns)
  {
    error = "controller periods, timeouts, replace lead, and horizon are inconsistent";
    return false;
  }

  std::uint64_t total_guard_ns = 0U;
  for (const std::uint64_t guard_ns : {
      candidate.scheduling_guard.one_cycle_detection_ns,
      candidate.scheduling_guard.stop_time_growth_ns,
      candidate.scheduling_guard.non_rt_to_rt_visibility_ns,
      candidate.scheduling_guard.period_quantization_ns})
  {
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
    candidate.open_feedback_age_limit_ms <= 0.0)
  {
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
    !read_axis_values(
      "splice_position_tolerances", candidate.splice_position_tolerances) ||
    !read_axis_values(
      "splice_velocity_tolerances", candidate.splice_velocity_tolerances))
  {
    return false;
  }

  runtime_configuration_ = candidate;
  return true;
}

controller_interface::InterfaceConfiguration
RollingTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/position");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RollingTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(kAxisCount + 1U);
  for (const char * joint : kJointNames) {
    configuration.names.emplace_back(std::string(joint) + "/position");
  }
  configuration.names.emplace_back(kFeedbackAgeInterface);
  return configuration;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  active_.store(false, std::memory_order_release);
  const std::string source = get_node()->get_parameter("configuration_source").as_string();
  const bool allow_test_only =
    get_node()->get_parameter("allow_test_only_configuration").as_bool();
  const bool allow_provisional =
    get_node()->get_parameter("allow_provisional_configuration").as_bool();
  const std::string envelope_file = get_node()->get_parameter("envelope_file").as_string();
  if (
    (source == "test_only" && !allow_test_only) ||
    (source == "provisional" && !allow_provisional) ||
    (source != "test_only" && source != "provisional"))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller configuration source '%s' lacks its explicit authority",
      source.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string configuration_error;
  if (!loadRuntimeConfiguration(configuration_error)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Invalid rolling runtime configuration: %s",
      configuration_error.c_str());
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
    if (!envelope_file.empty()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Test-only rolling configuration must not provide an envelope file");
      return controller_interface::CallbackReturn::ERROR;
    }
    envelope_ = makeTestOnlyEnvelope();
  } else {
    std::string envelope_error;
    if (!loadProvisionalEnvelope(envelope_file, envelope_, envelope_error)) {
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
    !buffer_.configureLimits(envelope_, allow_test_only, allow_provisional))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling buffer envelope");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(envelope_, allow_test_only, allow_provisional)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }

  callback_group_ =
    get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_publisher_ = get_node()->create_publisher<StateMessage>(
    "/rt/rolling_joint_control/state", robot_interfaces_qos::rolling_state());
  state_timer_ = get_node()->create_wall_timer(
    std::chrono::nanoseconds(runtime_configuration_.state_publish_period_ns),
    [this]() {(void)publishPublicState();}, callback_group_);
  state_timer_->cancel();
  open_service_ = get_node()->create_service<OpenService>(
    "/rt/rolling_joint_control/open",
    std::bind(
      &RollingTrajectoryController::handleOpen, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  close_service_ = get_node()->create_service<CloseService>(
    "/rt/rolling_joint_control/close",
    std::bind(
      &RollingTrajectoryController::handleClose, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  update_subscription_ = get_node()->create_subscription<BatchMessage>(
    "/rt/rolling_joint_control/update", robot_interfaces_qos::rolling_command(),
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
    observed_position_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
    desired_position_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
    desired_velocity_bits_[axis].store(encodeDouble(0.0), std::memory_order_relaxed);
  }
  feedback_age_bits_.store(encodeDouble(0.0), std::memory_order_relaxed);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_relaxed);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_relaxed);
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

controller_interface::CallbackReturn RollingTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  if (
    !configured_ || command_interfaces_.size() != kAxisCount ||
    state_interfaces_.size() != kAxisCount + 1U)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rolling controller requires 14 position command/state pairs and feedback age");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::array<double, kAxisCount> source_commands{};
  std::array<double, kAxisCount> actual_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const std::string expected_name = std::string(kJointNames[axis]) + "/position";
    if (
      command_interfaces_[axis].get_name() != expected_name ||
      state_interfaces_[axis].get_name() != expected_name)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unexpected position interface order at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    const double source_command = command_interfaces_[axis].get_value();
    const double actual_position = state_interfaces_[axis].get_value();
    if (
      !std::isfinite(source_command) || !std::isfinite(actual_position) ||
      std::abs(source_command - actual_position) >
      runtime_configuration_.takeover_tolerances[axis])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unsafe rolling-controller takeover at axis %zu", axis);
      return controller_interface::CallbackReturn::ERROR;
    }
    source_commands[axis] = source_command;
    actual_positions[axis] = actual_position;
  }
  if (state_interfaces_[kAxisCount].get_name() != kFeedbackAgeInterface) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing shared feedback-age state interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  const double feedback_age_ms = state_interfaces_[kAxisCount].get_value();

  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (
      buffer_.sessionState() == SessionState::kTerminated &&
      !buffer_.resetTerminated())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    if (buffer_.sessionState() != SessionState::kNone) {
      RCLCPP_ERROR(get_node()->get_logger(), "Rolling session still exists during activation");
      return controller_interface::CallbackReturn::ERROR;
    }
    controller_boot_id_ = generateIdentifier();
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    last_reject_code_ = RejectCode::kNone;
    last_rejected_sequence_ = 0U;
    accepted_count_ = 0U;
    rejected_count_ = 0U;
    superseded_pending_count_ = 0U;
    timeout_count_.store(0U, std::memory_order_release);
    invariant_failure_count_.store(0U, std::memory_order_release);
    last_service_error_ = ServiceResultMessage::NONE;
    state_sequence_ = 0U;
    stop_requested_.store(false, std::memory_order_release);
  }

  hold_positions_ = source_commands;
  rt_active_trajectory_ = TrajectoryImage{};
  rt_sampling_cursor_.reset();
  rt_identity_ = SessionIdentity{};
  rt_desired_ = JointPoint{};
  rt_desired_.positions = source_commands;
  rt_desired_.velocities.fill(0.0);
  rt_stop_trajectory_ = StopTrajectory{};
  if (!rt_stop_trajectory_.configure(
      envelope_, envelope_.source == LimitsSource::kTestOnly,
      envelope_.source == LimitsSource::kProvisional))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset rolling stop trajectory");
    return controller_interface::CallbackReturn::ERROR;
  }
  rt_observed_session_epoch_ = 0U;
  rt_publication_floor_ = 1U;
  rt_consumed_publication_sequence_ = 0U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kNone), std::memory_order_release);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  prime_start_time_ns_.store(0U, std::memory_order_release);
  session_epoch_.store(0U, std::memory_order_release);
  session_publication_floor_.store(1U, std::memory_order_release);
  rt_state_version_.store(0U, std::memory_order_release);
  rt_state_session_epoch_.store(0U, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  initial_handoff_gate_.clear(std::memory_order_release);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_position_bits_[axis].store(
      encodeDouble(actual_positions[axis]), std::memory_order_release);
    desired_position_bits_[axis].store(
      encodeDouble(source_commands[axis]), std::memory_order_release);
    desired_velocity_bits_[axis].store(encodeDouble(0.0), std::memory_order_release);
  }
  feedback_age_bits_.store(encodeDouble(feedback_age_ms), std::memory_order_release);
  active_.store(true, std::memory_order_release);
  if (state_timer_) {
    state_timer_->reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RollingTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    active_.store(false, std::memory_order_release);
    buffer_.terminateSession();
    controller_boot_id_ = Identifier{};
    open_cache_ = OpenCacheEntry{};
    clearActiveCloseCache();
    finalize_cache_ = CloseCacheEntry{};
    last_mode_result_ = ModeResultMessage{};
    last_mode_result_sequence_ = 0U;
    stop_requested_.store(false, std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kControllerDeactivated),
      std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kTerminated),
      std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    const std::uint64_t new_epoch = session_epoch_.fetch_add(
      1U, std::memory_order_acq_rel) + 1U;
    rt_state_session_epoch_.store(new_epoch, std::memory_order_release);
    rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
    const SessionIdentity invalid_identity{};
    const TrajectoryImage invalid_trajectory{};
    (void)snapshot_exchange_.publish(invalid_identity, invalid_trajectory, steadyNowNs());
  }
  (void)publishPublicState();
  if (state_timer_) {
    state_timer_->cancel();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RollingTrajectoryController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_.load(std::memory_order_acquire)) {
    return controller_interface::return_type::OK;
  }
  RtStateVersionGuard state_version(rt_state_version_);

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    observed_position_bits_[axis].store(
      encodeDouble(state_interfaces_[axis].get_value()), std::memory_order_release);
  }
  feedback_age_bits_.store(
    encodeDouble(state_interfaces_[kAxisCount].get_value()), std::memory_order_release);

  const std::int64_t period_ns = period.nanoseconds();
  SessionState state = SessionState::kNone;
  bool coherent_session = false;
  constexpr std::size_t kMaximumEpochReadAttempts = 3U;
  for (std::size_t attempt = 0U; attempt < kMaximumEpochReadAttempts; ++attempt) {
    const std::uint64_t epoch_before = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before != rt_observed_session_epoch_ && !resetRtEpoch(epoch_before)) {
      return controller_interface::return_type::ERROR;
    }
    state = static_cast<SessionState>(
      rt_session_state_.load(std::memory_order_acquire));
    const std::uint64_t epoch_after = session_epoch_.load(std::memory_order_acquire);
    if (epoch_before == epoch_after) {
      coherent_session = true;
      break;
    }
  }
  if (!coherent_session) {
    return writeRtDesired() ?
           controller_interface::return_type::OK :
           controller_interface::return_type::ERROR;
  }

  bool update_succeeded = true;
  if (state == SessionState::kPriming) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(
        stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else if (
      elapsedExceeds(
        steadyNowNs(), prime_start_time_ns_.load(std::memory_order_acquire),
        runtime_configuration_.prime_timeout_ns))
    {
      update_succeeded = beginRtStop(StopReason::kPrimeTimeout);
    } else {
      OptionalRtFlagGuard handoff(initial_handoff_gate_, true);
      if (handoff.mayProceed()) {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(true);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        }
      }
    }
  } else if (state == SessionState::kRunning) {
    if (stop_requested_.load(std::memory_order_acquire)) {
      StopReason reason = static_cast<StopReason>(
        stop_reason_.load(std::memory_order_acquire));
      if (reason == StopReason::kNone) {
        reason = StopReason::kGracefulClose;
      }
      update_succeeded = beginRtStop(reason);
    } else {
      const bool initial_handoff =
        execution_time_ns_.load(std::memory_order_relaxed) == 0U;
      OptionalRtFlagGuard handoff(initial_handoff_gate_, initial_handoff);
      if (!handoff.mayProceed()) {
        update_succeeded = true;
      } else {
        const SnapshotConsumeResult consumed = consumeLatestRtTrajectory(false);
        if (consumed == SnapshotConsumeResult::kInvalid) {
          update_succeeded = beginRtStop(StopReason::kInternalInvariant);
        } else if (!validPeriod(period_ns)) {
          update_succeeded = beginRtStop(StopReason::kClockAnomaly);
        } else if (
          elapsedExceeds(
            steadyNowNs(), rt_last_accepted_arrival_ns_,
            runtime_configuration_.update_timeout_ns))
        {
          update_succeeded = beginRtStop(StopReason::kUpdateTimeout);
        } else {
          std::uint64_t stop_duration_ns = 0U;
          const std::uint64_t execution_ns =
            execution_time_ns_.load(std::memory_order_relaxed);
          const std::uint64_t buffered_until_ns =
            buffered_until_ns_.load(std::memory_order_relaxed);
          if (!calculateStopDurationNs(rt_desired_, stop_duration_ns)) {
            rt_invariant_stage_.store(
              static_cast<std::uint8_t>(RtInvariantStage::kStopDuration),
              std::memory_order_release);
            update_succeeded = beginRtStop(StopReason::kInternalInvariant);
          } else if (!hasSufficientStoppingHorizon(
              execution_ns, buffered_until_ns, stop_duration_ns,
              runtime_configuration_.scheduling_guard))
          {
            update_succeeded = beginRtStop(StopReason::kLowWater);
          } else {
            std::uint64_t next_execution_ns = 0U;
            JointPoint candidate;
            const std::uint64_t increment_ns = static_cast<std::uint64_t>(period_ns);
            if (
              !addTime(execution_ns, increment_ns, next_execution_ns) ||
              !sampleTrajectoryImageMonotonic(
                rt_active_trajectory_, next_execution_ns,
                rt_sampling_cursor_, candidate) ||
              !desiredIsValid(candidate))
            {
              rt_invariant_stage_.store(
                static_cast<std::uint8_t>(RtInvariantStage::kTrajectorySample),
                std::memory_order_release);
              update_succeeded = beginRtStop(StopReason::kInternalInvariant);
            } else {
              rt_desired_ = candidate;
              execution_time_ns_.store(next_execution_ns, std::memory_order_release);
              update_succeeded = updateRtReplaceableBoundary();
              if (!update_succeeded) {
                rt_invariant_stage_.store(
                  static_cast<std::uint8_t>(RtInvariantStage::kReplaceableBoundary),
                  std::memory_order_release);
                update_succeeded = beginRtStop(StopReason::kInternalInvariant);
              }
            }
          }
        }
      }
    }
  } else if (state == SessionState::kStopping) {
    update_succeeded = advanceRtStop(period_ns);
  } else if (state == SessionState::kHolding || state == SessionState::kNone) {
    rt_desired_.velocities.fill(0.0);
  }

  if (!update_succeeded || !writeRtDesired()) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

Identifier RollingTrajectoryController::generateIdentifier() noexcept
{
  static std::atomic<std::uint64_t> counter{0U};
  const auto system_ticks = std::chrono::system_clock::now().time_since_epoch().count();
  const auto steady_ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  std::uint64_t state =
    static_cast<std::uint64_t>(system_ticks) ^
    (static_cast<std::uint64_t>(steady_ticks) << 1U) ^
    counter.fetch_add(1U, std::memory_order_relaxed);
  Identifier identifier{};
  for (std::size_t word_index = 0U; word_index < 2U; ++word_index) {
    const std::uint64_t word = splitMix64(state);
    for (std::size_t byte = 0U; byte < sizeof(word); ++byte) {
      identifier[word_index * sizeof(word) + byte] =
        static_cast<std::uint8_t>(word >> (byte * 8U));
    }
  }
  identifier[6] = static_cast<std::uint8_t>((identifier[6] & 0x0fU) | 0x40U);
  identifier[8] = static_cast<std::uint8_t>((identifier[8] & 0x3fU) | 0x80U);
  return identifier;
}

std::uint64_t RollingTrajectoryController::steadyNowNs() noexcept
{
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return nanoseconds < 0 ? 0U : static_cast<std::uint64_t>(nanoseconds);
}

std::uint64_t RollingTrajectoryController::encodeDouble(double value) noexcept
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double RollingTrajectoryController::decodeDouble(std::uint64_t bits) noexcept
{
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool RollingTrajectoryController::isZeroIdentifier(const Identifier & identifier) noexcept
{
  return std::all_of(
    identifier.begin(), identifier.end(), [](std::uint8_t value) {return value == 0U;});
}

bool RollingTrajectoryController::sameOpenRequest(
  const OpenService::Request & lhs, const OpenService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.expected_controller_boot_id.uuid == rhs.expected_controller_boot_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.axis_set_hash == rhs.axis_set_hash;
}

bool RollingTrajectoryController::sameCloseRequest(
  const CloseService::Request & lhs, const CloseService::Request & rhs) noexcept
{
  return lhs.protocol_major == rhs.protocol_major && lhs.protocol_minor == rhs.protocol_minor &&
         lhs.request_id.uuid == rhs.request_id.uuid &&
         lhs.controller_boot_id.uuid == rhs.controller_boot_id.uuid &&
         lhs.session_id.uuid == rhs.session_id.uuid &&
         lhs.client_instance_id.uuid == rhs.client_instance_id.uuid &&
         lhs.operation == rhs.operation;
}

bool RollingTrajectoryController::elapsedExceeds(
  std::uint64_t now_ns, std::uint64_t start_ns, std::uint64_t limit_ns) noexcept
{
  return now_ns < start_ns || now_ns - start_ns > limit_ns;
}

bool RollingTrajectoryController::validPeriod(std::int64_t period_ns) const noexcept
{
  return period_ns > 0 &&
         static_cast<std::uint64_t>(period_ns) <=
         runtime_configuration_.maximum_controller_period_ns;
}

bool RollingTrajectoryController::addTime(
  std::uint64_t value_ns, std::uint64_t increment_ns, std::uint64_t & sum_ns) noexcept
{
  if (increment_ns > std::numeric_limits<std::uint64_t>::max() - value_ns) {
    return false;
  }
  sum_ns = value_ns + increment_ns;
  return true;
}

bool RollingTrajectoryController::synchronizeBufferStateFromRt() noexcept
{
  const SessionState rt_state = static_cast<SessionState>(
    rt_session_state_.load(std::memory_order_acquire));
  SessionState buffer_state = buffer_.sessionState();

  if (
    rt_state == SessionState::kRunning &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning))
  {
    const std::uint64_t active_generation =
      active_generation_.load(std::memory_order_acquire);
    if (!buffer_.acknowledgeActiveGeneration(active_generation)) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (
    (rt_state == SessionState::kStopping || rt_state == SessionState::kHolding) &&
    (buffer_state == SessionState::kPriming || buffer_state == SessionState::kRunning))
  {
    if (!buffer_.requestStop()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kHolding && buffer_state == SessionState::kStopping) {
    if (!buffer_.markHolding()) {
      return false;
    }
    buffer_state = buffer_.sessionState();
  }
  if (rt_state == SessionState::kTerminated) {
    if (buffer_state != SessionState::kTerminated) {
      buffer_.terminateSession();
    }
    return true;
  }

  if (rt_state == SessionState::kRunning) {
    return buffer_state == SessionState::kRunning;
  }
  if (rt_state == SessionState::kStopping) {
    return buffer_state == SessionState::kStopping;
  }
  if (rt_state == SessionState::kHolding) {
    return buffer_state == SessionState::kHolding;
  }
  return true;
}

bool RollingTrajectoryController::readRtStateView(RtStateView & view) const noexcept
{
  constexpr std::size_t kMaximumReadAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumReadAttempts; ++attempt) {
    const std::uint64_t version_before =
      rt_state_version_.load(std::memory_order_acquire);
    if ((version_before & 1U) != 0U) {
      continue;
    }

    RtStateView candidate;
    candidate.session_epoch =
      rt_state_session_epoch_.load(std::memory_order_acquire);
    candidate.session_state = static_cast<SessionState>(
      rt_session_state_.load(std::memory_order_acquire));
    candidate.stop_reason = static_cast<StopReason>(
      stop_reason_.load(std::memory_order_acquire));
    candidate.active_generation =
      active_generation_.load(std::memory_order_acquire);
    candidate.execution_time_ns =
      execution_time_ns_.load(std::memory_order_acquire);
    candidate.replaceable_from_ns =
      replaceable_from_ns_.load(std::memory_order_acquire);
    candidate.buffered_until_ns =
      buffered_until_ns_.load(std::memory_order_acquire);
    candidate.prime_start_time_ns =
      prime_start_time_ns_.load(std::memory_order_acquire);
    candidate.accepted_arrival_time_ns =
      rt_accepted_arrival_time_ns_.load(std::memory_order_acquire);
    candidate.timeout_count = timeout_count_.load(std::memory_order_acquire);
    candidate.invariant_failure_count =
      invariant_failure_count_.load(std::memory_order_acquire);
    candidate.has_accepted_update =
      has_accepted_update_.load(std::memory_order_acquire);
    candidate.desired.time_ns = candidate.execution_time_ns;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      candidate.desired.positions[axis] = decodeDouble(
        desired_position_bits_[axis].load(std::memory_order_acquire));
      candidate.desired.velocities[axis] = decodeDouble(
        desired_velocity_bits_[axis].load(std::memory_order_acquire));
    }

    const std::uint64_t version_after =
      rt_state_version_.load(std::memory_order_acquire);
    if (
      version_before == version_after && (version_after & 1U) == 0U &&
      candidate.session_state <= SessionState::kTerminated &&
      candidate.stop_reason <= StopReason::kControllerRestart &&
      desiredIsValid(candidate.desired))
    {
      view = candidate;
      return true;
    }
  }
  return false;
}

bool RollingTrajectoryController::buildAdmissionContext(
  AdmissionContext & context) const noexcept
{
  if (
    buffer_.sessionState() == SessionState::kPriming &&
    static_cast<SessionState>(rt_session_state_.load(std::memory_order_acquire)) ==
    SessionState::kPriming &&
    active_generation_.load(std::memory_order_acquire) == 0U)
  {
    context.execution_time_ns = 0U;
    context.replaceable_from_ns = 0U;
    context.minimum_horizon_ns =
      runtime_configuration_.scheduling_guard.one_cycle_detection_ns +
      runtime_configuration_.scheduling_guard.stop_time_growth_ns +
      runtime_configuration_.scheduling_guard.non_rt_to_rt_visibility_ns +
      runtime_configuration_.scheduling_guard.period_quantization_ns;
    return true;
  }

  RtStateView view;
  if (
    !readRtStateView(view) ||
    view.session_epoch != session_epoch_.load(std::memory_order_acquire))
  {
    return false;
  }

  std::uint64_t stop_duration_ns = 0U;
  if (!calculateStopDurationNs(view.desired, stop_duration_ns)) {
    return false;
  }
  std::uint64_t minimum_horizon_ns = stop_duration_ns;
  if (
    !addTime(
      minimum_horizon_ns,
      runtime_configuration_.scheduling_guard.one_cycle_detection_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns,
      runtime_configuration_.scheduling_guard.stop_time_growth_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns,
      runtime_configuration_.scheduling_guard.non_rt_to_rt_visibility_ns,
      minimum_horizon_ns) ||
    !addTime(
      minimum_horizon_ns,
      runtime_configuration_.scheduling_guard.period_quantization_ns,
      minimum_horizon_ns))
  {
    return false;
  }
  context.execution_time_ns = view.execution_time_ns;
  context.replaceable_from_ns = view.replaceable_from_ns;
  context.minimum_horizon_ns = minimum_horizon_ns;
  return true;
}

bool RollingTrajectoryController::buildPublicState(StateMessage & state)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  RtStateView view;
  bool coherent = false;
  constexpr std::size_t kMaximumStateBuildAttempts = 8U;
  for (std::size_t attempt = 0U; attempt < kMaximumStateBuildAttempts; ++attempt) {
    if (!synchronizeBufferStateFromRt() || !readRtStateView(view)) {
      continue;
    }
    const std::uint64_t session_epoch = session_epoch_.load(std::memory_order_acquire);
    if (view.session_epoch != session_epoch) {
      continue;
    }

    const SessionState buffer_state = buffer_.sessionState();
    const bool state_aligned =
      (view.session_state == SessionState::kRunning &&
      buffer_state == SessionState::kRunning) ||
      (view.session_state == SessionState::kStopping &&
      buffer_state == SessionState::kStopping) ||
      (view.session_state == SessionState::kHolding &&
      buffer_state == SessionState::kHolding) ||
      (view.session_state == SessionState::kTerminated &&
      buffer_state == SessionState::kTerminated) ||
      (view.session_state == SessionState::kPriming &&
      buffer_state == SessionState::kPriming) ||
      (view.session_state == SessionState::kNone &&
      buffer_state == SessionState::kNone);
    if (!state_aligned) {
      continue;
    }
    if (
      view.session_state == SessionState::kRunning &&
      buffer_.hasPending() &&
      buffer_.pendingImage().generation <= view.active_generation)
    {
      continue;
    }
    coherent = true;
    break;
  }
  if (!coherent) {
    return false;
  }

  const std::uint64_t now_ns = steadyNowNs();
  const SessionState buffer_state = buffer_.sessionState();
  const bool has_session =
    buffer_state != SessionState::kNone && buffer_state != SessionState::kTerminated;
  const bool pending_valid = buffer_.hasPending();
  const TrajectoryImage & validation_head =
    pending_valid ? buffer_.pendingImage() : buffer_.image();

  state = StateMessage{};
  state.published_at = get_node()->now();
  state.protocol_major = kProtocolMajor;
  state.protocol_minor = kProtocolMinor;
  state.controller_boot_id.uuid = controller_boot_id_;
  if (has_session) {
    state.session_id.uuid = buffer_.identity().session_id;
    state.client_instance_id.uuid = buffer_.identity().client_instance_id;
  }
  state.control_mode.value = active_.load(std::memory_order_acquire) ?
    ControlModeMessage::ROLLING_READY : ControlModeMessage::DISABLED;
  state.session_state.value = static_cast<std::uint8_t>(view.session_state);
  state.has_session = has_session;
  state.has_accepted_update = view.has_accepted_update;
  state.pending_generation_valid = pending_valid;
  state.test_only_limits = envelope_.source == LimitsSource::kTestOnly;
  state.limits_source.value = static_cast<std::uint8_t>(envelope_.source);
  state.axis_set_hash = kAxisSetHash;
  state.limits_version = envelope_.limits_version;
  state.active_generation = view.active_generation;
  state.validation_base_generation = buffer_.validationBaseGeneration();
  state.pending_generation = pending_valid ? validation_head.generation : 0U;
  state.last_seen_sequence = buffer_.lastSeenSequence();
  state.last_accepted_sequence = buffer_.lastAcceptedSequence();
  state.last_rejected_sequence = last_rejected_sequence_;
  state.execution_time_ns = view.execution_time_ns;
  state.replaceable_from_ns = view.replaceable_from_ns;
  state.buffered_until_ns = view.buffered_until_ns;
  state.available_horizon_ns = view.buffered_until_ns >= view.execution_time_ns ?
    view.buffered_until_ns - view.execution_time_ns : 0U;
  state.prime_age_ns =
    view.session_state == SessionState::kPriming &&
    now_ns >= view.prime_start_time_ns ?
    now_ns - view.prime_start_time_ns : 0U;
  state.accepted_update_age_ns =
    view.has_accepted_update && now_ns >= view.accepted_arrival_time_ns ?
    now_ns - view.accepted_arrival_time_ns : 0U;
  state.buffer_point_count = static_cast<std::uint16_t>(validation_head.point_count);
  state.buffer_capacity = static_cast<std::uint16_t>(buffer_.capacity());
  state.desired_positions = view.desired.positions;
  state.desired_velocities = view.desired.velocities;
  state.last_reject.value = static_cast<std::uint8_t>(last_reject_code_);
  state.stop_reason.value = static_cast<std::uint8_t>(view.stop_reason);
  if (last_mode_result_sequence_ != 0U) {
    state.last_mode_request_id = last_mode_result_.request_id;
    state.last_mode_result = last_mode_result_.result;
    state.source_controller_deactivated =
      last_mode_result_.source_controller_deactivated;
    state.target_controller_activated = last_mode_result_.target_controller_activated;
    state.restart_required = last_mode_result_.restart_required;
  } else {
    state.last_mode_result.value = ServiceResultMessage::NONE;
    state.source_controller_deactivated =
      view.stop_reason == StopReason::kControllerDeactivated;
  }
  state.accepted_count = accepted_count_;
  state.rejected_count = rejected_count_;
  state.superseded_pending_count = superseded_pending_count_;
  state.timeout_count = view.timeout_count;
  state.invariant_failure_count = view.invariant_failure_count;
  state.command_deadline_miss_count = 0U;
  state.state_sequence = ++state_sequence_;
  return true;
}

bool RollingTrajectoryController::publishPublicState()
{
  std::lock_guard<std::mutex> publish_lock(state_publish_mutex_);
  if (!state_publisher_ || !state_publisher_->is_activated()) {
    return false;
  }
  StateMessage state;
  if (!buildPublicState(state)) {
    return false;
  }
  state_publisher_->publish(state);
  return true;
}

bool RollingTrajectoryController::resetRtEpoch(std::uint64_t epoch) noexcept
{
  const std::uint64_t publication_floor =
    session_publication_floor_.load(std::memory_order_acquire);
  if (publication_floor == 0U) {
    return false;
  }

  JointPoint hold;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    hold.positions[axis] = decodeDouble(
      desired_position_bits_[axis].load(std::memory_order_acquire));
    hold.velocities[axis] = 0.0;
  }
  if (!desiredIsValid(hold)) {
    return false;
  }

  StopTrajectory stop_trajectory;
  if (!stop_trajectory.configure(envelope_, true)) {
    return false;
  }

  rt_active_trajectory_ = TrajectoryImage{};
  rt_sampling_cursor_.reset();
  rt_identity_ = SessionIdentity{};
  rt_desired_ = hold;
  rt_stop_trajectory_ = stop_trajectory;
  rt_publication_floor_ = publication_floor;
  rt_consumed_publication_sequence_ = publication_floor - 1U;
  rt_last_accepted_arrival_ns_ = 0U;
  rt_accepted_arrival_time_ns_.store(0U, std::memory_order_release);
  rt_stop_elapsed_ns_ = 0U;
  rt_invariant_stage_.store(
    static_cast<std::uint8_t>(RtInvariantStage::kNone), std::memory_order_release);
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  rt_observed_session_epoch_ = epoch;
  return true;
}

RollingTrajectoryController::SnapshotConsumeResult
RollingTrajectoryController::consumeLatestRtTrajectory(bool priming) noexcept
{
  SnapshotLease lease;
  if (!snapshot_exchange_.acquire(lease)) {
    return SnapshotConsumeResult::kNoChange;
  }
  const RollingSnapshot & snapshot = *lease;
  if (
    snapshot.publication_sequence < rt_publication_floor_ ||
    snapshot.publication_sequence <= rt_consumed_publication_sequence_)
  {
    return SnapshotConsumeResult::kNoChange;
  }

  const TrajectoryImage & trajectory = snapshot.trajectory;
  const std::uint64_t active_generation =
    active_generation_.load(std::memory_order_relaxed);
  if (
    trajectory.point_count == 0U || trajectory.point_count > kTransportMaxPoints ||
    trajectory.generation == 0U || trajectory.generation <= active_generation ||
    !trajectoryImageStructureIsValid(trajectory))
  {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  const std::uint64_t execution_ns = execution_time_ns_.load(std::memory_order_relaxed);
  if (priming) {
    if (execution_ns != 0U || trajectory.points[0].time_ns != 0U) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPrimingTime),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  } else {
    if (
      trajectory.points[0].time_ns > execution_ns ||
      trajectory.points[trajectory.point_count - 1U].time_ns < execution_ns)
    {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
    const std::uint64_t replaceable_from_ns =
      replaceable_from_ns_.load(std::memory_order_acquire);
    const bool safe_zero_time_handoff =
      execution_ns == 0U && trajectory.earliest_changed_ns == 0U;
    if (
      snapshot.identity.controller_boot_id != rt_identity_.controller_boot_id ||
      snapshot.identity.session_id != rt_identity_.session_id ||
      snapshot.identity.client_instance_id != rt_identity_.client_instance_id)
    {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotIdentity),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
    if (trajectory.earliest_changed_ns < replaceable_from_ns && !safe_zero_time_handoff) {
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotLateBoundary),
        std::memory_order_release);
      return SnapshotConsumeResult::kInvalid;
    }
  }

  JointPoint candidate;
  MonotonicTrajectoryCursor candidate_cursor;
  std::uint64_t replacement_boundary_ns = 0U;
  if (
    !sampleTrajectoryImageMonotonic(
      trajectory, execution_ns, candidate_cursor, candidate) ||
    !desiredIsValid(candidate) ||
    !addTime(
      execution_ns, runtime_configuration_.replace_lead_ns,
      replacement_boundary_ns))
  {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotSample),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }

  if (!copyTrajectoryImageEffective(trajectory, rt_active_trajectory_)) {
    rt_invariant_stage_.store(
      static_cast<std::uint8_t>(RtInvariantStage::kSnapshotShapeOrGeneration),
      std::memory_order_release);
    return SnapshotConsumeResult::kInvalid;
  }
  rt_sampling_cursor_ = candidate_cursor;
  rt_identity_ = snapshot.identity;
  rt_desired_ = candidate;
  rt_consumed_publication_sequence_ = snapshot.publication_sequence;
  rt_last_accepted_arrival_ns_ = snapshot.arrival_time_ns;
  rt_accepted_arrival_time_ns_.store(snapshot.arrival_time_ns, std::memory_order_release);
  active_generation_.store(trajectory.generation, std::memory_order_release);
  buffered_until_ns_.store(
    trajectory.points[trajectory.point_count - 1U].time_ns,
    std::memory_order_release);
  replaceable_from_ns_.store(replacement_boundary_ns, std::memory_order_release);
  has_accepted_update_.store(true, std::memory_order_release);
  if (priming) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kRunning),
      std::memory_order_release);
  }
  return SnapshotConsumeResult::kAccepted;
}

bool RollingTrajectoryController::beginRtStop(StopReason reason) noexcept
{
  const SessionState state = static_cast<SessionState>(
    rt_session_state_.load(std::memory_order_relaxed));
  if (state == SessionState::kStopping || state == SessionState::kHolding) {
    return true;
  }
  if (state != SessionState::kPriming && state != SessionState::kRunning) {
    return false;
  }

  std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
  const bool reason_latched = stop_reason_.compare_exchange_strong(
    expected_reason, static_cast<std::uint8_t>(reason),
    std::memory_order_acq_rel, std::memory_order_acquire);
  if (reason_latched) {
    if (reason == StopReason::kPrimeTimeout || reason == StopReason::kUpdateTimeout) {
      timeout_count_.fetch_add(1U, std::memory_order_relaxed);
    } else if (reason == StopReason::kInternalInvariant) {
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
  if (!rt_stop_trajectory_.begin(rt_desired_)) {
    return false;
  }
  rt_stop_elapsed_ns_ = 0U;
  stop_requested_.store(false, std::memory_order_release);

  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    JointPoint terminal;
    if (!rt_stop_trajectory_.sample(0U, terminal) || !desiredIsValid(terminal)) {
      return false;
    }
    rt_desired_ = terminal;
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding),
      std::memory_order_release);
  } else {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kStopping),
      std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::advanceRtStop(std::int64_t period_ns) noexcept
{
  const StopReason reason = static_cast<StopReason>(
    stop_reason_.load(std::memory_order_acquire));
  std::uint64_t increment_ns = runtime_configuration_.nominal_controller_period_ns;
  if (reason != StopReason::kClockAnomaly && validPeriod(period_ns)) {
    increment_ns = static_cast<std::uint64_t>(period_ns);
  }

  std::uint64_t next_elapsed_ns = 0U;
  std::uint64_t next_execution_ns = 0U;
  if (
    !addTime(rt_stop_elapsed_ns_, increment_ns, next_elapsed_ns) ||
    !addTime(
      execution_time_ns_.load(std::memory_order_relaxed), increment_ns,
      next_execution_ns))
  {
    return false;
  }

  JointPoint candidate;
  if (
    !rt_stop_trajectory_.sample(next_elapsed_ns, candidate) ||
    !desiredIsValid(candidate))
  {
    return false;
  }
  rt_desired_ = candidate;
  rt_stop_elapsed_ns_ = next_elapsed_ns;
  execution_time_ns_.store(next_execution_ns, std::memory_order_release);
  if (rt_stop_trajectory_.state() == StopTrajectoryState::kHolding) {
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kHolding),
      std::memory_order_release);
  }
  return true;
}

bool RollingTrajectoryController::updateRtReplaceableBoundary() noexcept
{
  std::uint64_t boundary_ns = 0U;
  if (!addTime(
      execution_time_ns_.load(std::memory_order_relaxed),
      runtime_configuration_.replace_lead_ns, boundary_ns))
  {
    return false;
  }
  replaceable_from_ns_.store(boundary_ns, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::writeRtDesired() noexcept
{
  if (!desiredIsValid(rt_desired_)) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    command_interfaces_[axis].set_value(rt_desired_.positions[axis]);
    desired_position_bits_[axis].store(
      encodeDouble(rt_desired_.positions[axis]), std::memory_order_release);
    desired_velocity_bits_[axis].store(
      encodeDouble(rt_desired_.velocities[axis]), std::memory_order_release);
  }
  rt_state_session_epoch_.store(rt_observed_session_epoch_, std::memory_order_release);
  rt_accepted_arrival_time_ns_.store(
    rt_last_accepted_arrival_ns_, std::memory_order_release);
  return true;
}

bool RollingTrajectoryController::desiredIsValid(const JointPoint & desired) const noexcept
{
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double position = desired.positions[axis];
    const double velocity = desired.velocities[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (
      !std::isfinite(position) || !std::isfinite(velocity) ||
      position < safe_lower || position > safe_upper ||
      velocity > limits.velocity_positive || velocity < -limits.velocity_negative)
    {
      return false;
    }
  }
  return true;
}

bool RollingTrajectoryController::calculateStopDurationNs(
  const JointPoint & desired, std::uint64_t & duration_ns) const noexcept
{
  if (!desiredIsValid(desired)) {
    return false;
  }
  long double duration_seconds = 0.0L;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double velocity = desired.velocities[axis];
    const double limit = velocity >= 0.0 ?
      envelope_.axes[axis].stop_acceleration_positive :
      envelope_.axes[axis].stop_acceleration_negative;
    const long double axis_duration =
      std::abs(static_cast<long double>(velocity)) /
      static_cast<long double>(limit);
    duration_seconds = std::max(duration_seconds, axis_duration);
  }
  const long double nanoseconds = std::ceil(duration_seconds * 1.0e9L);
  if (
    !std::isfinite(nanoseconds) || nanoseconds < 0.0L ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::uint64_t>::max()))
  {
    return false;
  }
  duration_ns = static_cast<std::uint64_t>(nanoseconds);
  return true;
}

void RollingTrajectoryController::handleOpen(
  const std::shared_ptr<OpenService::Request> request,
  std::shared_ptr<OpenService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  response->protocol_major = kProtocolMajor;
  response->protocol_minor = kProtocolMinor;
  response->controller_boot_id.uuid = controller_boot_id_;

  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setOpenError(*response, ServiceResultMessage::WRONG_PROTOCOL);
    return;
  }
  if (isZeroIdentifier(request->request_id.uuid)) {
    setOpenError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (request->expected_controller_boot_id.uuid != controller_boot_id_) {
    setOpenError(*response, ServiceResultMessage::WRONG_BOOT);
    return;
  }
  if (isZeroIdentifier(request->client_instance_id.uuid)) {
    setOpenError(*response, ServiceResultMessage::WRONG_CLIENT);
    return;
  }
  if (request->axis_set_hash != kAxisSetHash) {
    setOpenError(*response, ServiceResultMessage::AXIS_SET_MISMATCH);
    return;
  }
  if (
    open_cache_.valid &&
    open_cache_.request.request_id.uuid == request->request_id.uuid &&
    open_cache_.request.client_instance_id.uuid == request->client_instance_id.uuid)
  {
    if (!sameOpenRequest(open_cache_.request, *request)) {
      setOpenError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = open_cache_.response;
    last_service_error_ = response->result.value;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setOpenError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setOpenError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (buffer_.sessionState() != SessionState::kNone) {
    setOpenError(*response, ServiceResultMessage::SESSION_BUSY);
    return;
  }

  const double feedback_age_ms =
    decodeDouble(feedback_age_bits_.load(std::memory_order_acquire));
  if (
    !std::isfinite(feedback_age_ms) || feedback_age_ms < 0.0 ||
    feedback_age_ms > runtime_configuration_.open_feedback_age_limit_ms)
  {
    setOpenError(*response, ServiceResultMessage::FEEDBACK_STALE);
    return;
  }

  std::array<double, kAxisCount> hold_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double hold =
      decodeDouble(desired_position_bits_[axis].load(std::memory_order_acquire));
    const double actual =
      decodeDouble(observed_position_bits_[axis].load(std::memory_order_acquire));
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (!std::isfinite(hold) || hold < safe_lower || hold > safe_upper) {
      setOpenError(*response, ServiceResultMessage::UNSAFE_HOLD);
      return;
    }
    if (
      !std::isfinite(actual) ||
      std::abs(hold - actual) > runtime_configuration_.takeover_tolerances[axis])
    {
      setOpenError(*response, ServiceResultMessage::TAKEOVER_MISMATCH);
      return;
    }
    hold_positions[axis] = hold;
  }

  SessionIdentity identity;
  identity.controller_boot_id = controller_boot_id_;
  identity.session_id = generateIdentifier();
  identity.client_instance_id = request->client_instance_id.uuid;
  JointPoint prime_anchor;
  prime_anchor.positions = hold_positions;
  prime_anchor.velocities.fill(0.0);
  const std::uint64_t current_epoch = session_epoch_.load(std::memory_order_acquire);
  const std::uint64_t latest_publication =
    snapshot_exchange_.latestPublicationSequence();
  if (
    current_epoch == std::numeric_limits<std::uint64_t>::max() ||
    latest_publication == std::numeric_limits<std::uint64_t>::max())
  {
    setOpenError(*response, ServiceResultMessage::RESTART_REQUIRED);
    return;
  }
  if (!buffer_.beginSession(identity, prime_anchor)) {
    setOpenError(*response, ServiceResultMessage::LIMITS_UNAVAILABLE);
    return;
  }

  response->accepted = true;
  response->result.value = ServiceResultMessage::NONE;
  populateServiceError(response->error, ServiceResultMessage::NONE);
  response->client_instance_id = request->client_instance_id;
  response->session_id.uuid = identity.session_id;
  response->session_state.value = SessionStateMessage::PRIMING;
  response->axis_set_hash = kAxisSetHash;
  response->limits_version = envelope_.limits_version;
  response->limits_source.value = static_cast<std::uint8_t>(envelope_.source);
  response->capability_bits = kCapabilityBits;
  response->transport_max_points = static_cast<std::uint16_t>(kTransportMaxPoints);
  response->buffer_capacity = static_cast<std::uint16_t>(buffer_.capacity());
  response->max_horizon_ns = runtime_configuration_.max_horizon_ns;
  response->required_initial_horizon_ns =
    runtime_configuration_.required_initial_horizon_ns;
  response->replace_lead_ns = runtime_configuration_.replace_lead_ns;
  response->update_timeout_ns = runtime_configuration_.update_timeout_ns;
  response->nominal_controller_period_ns =
    runtime_configuration_.nominal_controller_period_ns;
  response->initial_replaceable_from_ns = 0U;
  response->test_only_limits = envelope_.source == LimitsSource::kTestOnly;
  response->hold_positions = hold_positions;
  response->hold_velocities.fill(0.0);
  open_cache_.valid = true;
  open_cache_.request = *request;
  open_cache_.response = *response;
  clearActiveCloseCache();
  execution_time_ns_.store(0U, std::memory_order_release);
  active_generation_.store(0U, std::memory_order_release);
  buffered_until_ns_.store(0U, std::memory_order_release);
  replaceable_from_ns_.store(0U, std::memory_order_release);
  has_accepted_update_.store(false, std::memory_order_release);
  prime_start_time_ns_.store(steadyNowNs(), std::memory_order_release);
  session_publication_floor_.store(latest_publication + 1U, std::memory_order_release);
  stop_reason_.store(
    static_cast<std::uint8_t>(StopReason::kNone), std::memory_order_release);
  stop_requested_.store(false, std::memory_order_release);
  session_epoch_.store(current_epoch + 1U, std::memory_order_release);
  rt_session_state_.store(
    static_cast<std::uint8_t>(SessionState::kPriming),
    std::memory_order_release);
  last_service_error_ = ServiceResultMessage::NONE;
}

void RollingTrajectoryController::handleClose(
  const std::shared_ptr<CloseService::Request> request,
  std::shared_ptr<CloseService::Response> response)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  response->request_id = request->request_id;
  if (request->protocol_major != kProtocolMajor || request->protocol_minor != kProtocolMinor) {
    setCloseError(*response, ServiceResultMessage::WRONG_PROTOCOL);
    return;
  }
  if (
    isZeroIdentifier(request->request_id.uuid) ||
    (request->operation != CloseService::Request::REQUEST_STOP &&
    request->operation != CloseService::Request::FINALIZE))
  {
    setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (request->controller_boot_id.uuid != controller_boot_id_) {
    setCloseError(*response, ServiceResultMessage::WRONG_BOOT);
    return;
  }
  if (
    finalize_cache_.valid &&
    finalize_cache_.request.request_id.uuid == request->request_id.uuid)
  {
    if (!sameCloseRequest(finalize_cache_.request, *request)) {
      setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = finalize_cache_.response;
    last_service_error_ = response->result.value;
    return;
  }
  if (!active_.load(std::memory_order_acquire)) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }
  if (!synchronizeBufferStateFromRt()) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }

  const SessionState state = buffer_.sessionState();
  if (state == SessionState::kNone || state == SessionState::kTerminated) {
    setCloseError(*response, ServiceResultMessage::WRONG_SESSION);
    return;
  }
  const SessionIdentity & identity = buffer_.identity();
  if (request->session_id.uuid != identity.session_id) {
    setCloseError(*response, ServiceResultMessage::WRONG_SESSION);
    return;
  }
  if (request->client_instance_id.uuid != identity.client_instance_id) {
    setCloseError(*response, ServiceResultMessage::WRONG_CLIENT);
    return;
  }

  if (CloseCacheEntry * cached = findActiveCloseCache(request->request_id.uuid)) {
    if (!sameCloseRequest(cached->request, *request)) {
      setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
      return;
    }
    *response = cached->response;
    last_service_error_ = response->result.value;
    return;
  }

  if (request->operation == CloseService::Request::FINALIZE) {
    if (state != SessionState::kHolding) {
      setCloseError(*response, ServiceResultMessage::WRONG_MODE);
      return;
    }
    if (!buffer_.finishSession()) {
      setCloseError(*response, ServiceResultMessage::NOT_READY);
      return;
    }
    response->accepted = true;
    response->completed = true;
    response->result.value = ServiceResultMessage::NONE;
    populateServiceError(response->error, ServiceResultMessage::NONE);
    response->session_state.value = SessionStateMessage::NONE;
    response->stop_reason.value = StopReasonMessage::GRACEFUL_CLOSE;
    finalize_cache_.valid = true;
    finalize_cache_.request = *request;
    finalize_cache_.response = *response;
    clearActiveCloseCache();
    stop_requested_.store(false, std::memory_order_release);
    rt_session_state_.store(
      static_cast<std::uint8_t>(SessionState::kNone),
      std::memory_order_release);
    stop_reason_.store(
      static_cast<std::uint8_t>(StopReason::kNone),
      std::memory_order_release);
    has_accepted_update_.store(false, std::memory_order_release);
    session_epoch_.fetch_add(1U, std::memory_order_acq_rel);
    last_service_error_ = ServiceResultMessage::NONE;
    return;
  }

  CloseCacheEntry * cache = allocateActiveCloseCache();
  if (cache == nullptr) {
    setCloseError(*response, ServiceResultMessage::WRONG_REQUEST);
    return;
  }
  if (state == SessionState::kPriming || state == SessionState::kRunning) {
    if (!buffer_.requestStop()) {
      setCloseError(*response, ServiceResultMessage::NOT_READY);
      return;
    }
    std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
    (void)stop_reason_.compare_exchange_strong(
      expected_reason, static_cast<std::uint8_t>(StopReason::kGracefulClose),
      std::memory_order_acq_rel, std::memory_order_acquire);
    stop_requested_.store(true, std::memory_order_release);
  } else if (state != SessionState::kStopping && state != SessionState::kHolding) {
    setCloseError(*response, ServiceResultMessage::NOT_READY);
    return;
  }

  response->accepted = true;
  response->completed = state == SessionState::kHolding;
  response->result.value = ServiceResultMessage::NONE;
  populateServiceError(response->error, ServiceResultMessage::NONE);
  response->session_state.value = state == SessionState::kHolding ?
    SessionStateMessage::HOLDING : SessionStateMessage::STOPPING;
  response->stop_reason.value = StopReasonMessage::GRACEFUL_CLOSE;
  cache->valid = true;
  cache->request = *request;
  cache->response = *response;
  last_service_error_ = ServiceResultMessage::NONE;
}

void RollingTrajectoryController::handleUpdate(const BatchMessage::SharedPtr message)
{
  if (!message) {
    return;
  }
  BlockingFlagGuard initial_handoff(initial_handoff_gate_);
  std::array<PointView, kTransportMaxPoints> point_views{};
  const std::size_t point_count = message->points.size();
  if (point_count <= point_views.size()) {
    for (std::size_t index = 0U; index < point_count; ++index) {
      const auto & point = message->points[index];
      point_views[index] = PointView{
        point.time_from_session_start_ns,
        point.positions.data(), point.positions.size(),
        point.velocities.data(), point.velocities.size()};
    }
  }

  std::lock_guard<std::mutex> lock(callback_mutex_);
  RejectCode result = RejectCode::kNone;
  PreparedSubmission submission;
  AdmissionContext admission_context;
  bool supersedes_pending = false;
  if (!synchronizeBufferStateFromRt() || !buildAdmissionContext(admission_context)) {
    result = RejectCode::kSessionNotAccepting;
  } else if (
    message->protocol_major != kProtocolMajor || message->protocol_minor != kProtocolMinor)
  {
    result = RejectCode::kWrongProtocol;
  } else if (message->controller_boot_id.uuid != controller_boot_id_) {
    result = RejectCode::kWrongBoot;
  } else {
    BatchView batch;
    batch.protocol_major = message->protocol_major;
    batch.protocol_minor = message->protocol_minor;
    batch.controller_boot_id = message->controller_boot_id.uuid;
    batch.session_id = message->session_id.uuid;
    batch.client_instance_id = message->client_instance_id.uuid;
    batch.sequence = message->sequence;
    batch.replace_from_ns = message->replace_from_ns;
    batch.points = point_views.data();
    batch.point_count = point_count;
    supersedes_pending = buffer_.hasPending();
    result = buffer_.prepare(batch, admission_context, submission);
    if (result == RejectCode::kNone) {
      AdmissionContext commit_context;
      if (
        !synchronizeBufferStateFromRt() ||
        !buildAdmissionContext(commit_context))
      {
        result = RejectCode::kSessionNotAccepting;
      } else {
        result = buffer_.commit(submission, commit_context);
      }
    }
  }

  if (result == RejectCode::kNone) {
    if (!snapshot_exchange_.publish(
        buffer_.identity(), buffer_.pendingImage(), steadyNowNs()))
    {
      buffer_.terminateSession();
      std::uint8_t expected_reason = static_cast<std::uint8_t>(StopReason::kNone);
      (void)stop_reason_.compare_exchange_strong(
        expected_reason, static_cast<std::uint8_t>(StopReason::kInternalInvariant),
        std::memory_order_acq_rel, std::memory_order_acquire);
      rt_invariant_stage_.store(
        static_cast<std::uint8_t>(RtInvariantStage::kSnapshotPublication),
        std::memory_order_release);
      stop_requested_.store(true, std::memory_order_release);
      invariant_failure_count_.fetch_add(1U, std::memory_order_relaxed);
      result = RejectCode::kSessionNotAccepting;
    } else {
      ++accepted_count_;
      if (supersedes_pending) {
        ++superseded_pending_count_;
      }
    }
  }
  last_reject_code_ = result;
  if (result != RejectCode::kNone) {
    last_rejected_sequence_ = message->sequence;
    ++rejected_count_;
  }
}

void RollingTrajectoryController::handleModeResult(
  const ModeResultMessage::SharedPtr message)
{
  if (
    message == nullptr ||
    message->protocol_major != ModeResultMessage::PROTOCOL_MAJOR ||
    message->protocol_minor != ModeResultMessage::PROTOCOL_MINOR ||
    message->sequence == 0U || isZeroIdentifier(message->request_id.uuid) ||
    message->result.value > ServiceResultMessage::NOT_READY ||
    message->mode.value > ControlModeMessage::RESTART_REQUIRED)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (message->sequence <= last_mode_result_sequence_) {
    return;
  }
  last_mode_result_ = *message;
  last_mode_result_sequence_ = message->sequence;
}

void RollingTrajectoryController::clearActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    entry = CloseCacheEntry{};
  }
}

RollingTrajectoryController::CloseCacheEntry *
RollingTrajectoryController::findActiveCloseCache(const Identifier & request_id) noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (entry.valid && entry.request.request_id.uuid == request_id) {
      return &entry;
    }
  }
  return nullptr;
}

RollingTrajectoryController::CloseCacheEntry *
RollingTrajectoryController::allocateActiveCloseCache() noexcept
{
  for (CloseCacheEntry & entry : close_cache_) {
    if (!entry.valid) {
      return &entry;
    }
  }
  return nullptr;
}

void RollingTrajectoryController::setOpenError(
  OpenService::Response & response, std::uint8_t error_code)
{
  response.accepted = false;
  response.result.value = error_code;
  populateServiceError(response.error, error_code);
  last_service_error_ = error_code;
}

void RollingTrajectoryController::setCloseError(
  CloseService::Response & response, std::uint8_t error_code)
{
  response.accepted = false;
  response.completed = false;
  response.result.value = error_code;
  populateServiceError(response.error, error_code);
  response.session_state.value = static_cast<std::uint8_t>(buffer_.sessionState());
  response.stop_reason.value = buffer_.sessionState() == SessionState::kStopping ||
    buffer_.sessionState() == SessionState::kHolding ?
    StopReasonMessage::GRACEFUL_CLOSE : StopReasonMessage::NONE;
  last_service_error_ = error_code;
}

}  // namespace rolling_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
  rolling_trajectory_controller::RollingTrajectoryController,
  controller_interface::ControllerInterface)
