#include "swerve_driver/swerve_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "rt_control_semantic_components/cia402_axis.hpp"

namespace swerve_driver
{
namespace
{
using CallbackReturn = controller_interface::CallbackReturn;
bool positive(double value) {return std::isfinite(value) && value > 0.0;}
bool enabled(double value)
{
  if (!std::isfinite(value) || value < 0.0 || value > 65535.0 || std::trunc(value) != value) {
    return false;
  }
  return rt_control_semantic_components::Cia402Axis::decode_state(static_cast<uint16_t>(value)) ==
         rt_control_semantic_components::Cia402State::kOperationEnabled;
}
bool fresh(int64_t received, int64_t now, double limit)
{
  return received > 0 && now >= received && static_cast<double>(now - received) * 1e-9 <= limit;
}
const char * status_name(ControlStatus status)
{
  switch (status) {
    case ControlStatus::inactive: return "inactive";
    case ControlStatus::running: return "running";
    case ControlStatus::alignment_gated: return "alignment_gated";
    case ControlStatus::command_timeout: return "command_timeout";
    case ControlStatus::feedback_fault: return "feedback_fault";
    case ControlStatus::invalid_command: return "invalid_command";
    case ControlStatus::steering_limit: return "steering_limit";
    case ControlStatus::steering_hold_fault:
      return "steering_hold_fault";
  }
  return "unknown";
}
}  // namespace

int64_t SwerveController::steady_now() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

CallbackReturn SwerveController::on_init()
{
  ChassisRuntime::declare_parameters(get_node());
  last_steering_commands_.fill(std::numeric_limits<double>::quiet_NaN());
  auto_declare<bool>("calibration_verified", false);
  auto_declare<std::vector<double>>("steering_to_yaw_sign", {1.0, 1.0, 1.0, 1.0});
  auto_declare<double>("max_steering_acceleration", std::numeric_limits<double>::infinity());
  auto_declare<double>("fault_stop_speed", 0.01);
  auto_declare<double>("fault_stop_hold", 0.05);
  auto_declare<double>("fault_stop_timeout", 2.0);
  // External encoder monitor window (user 2026-10-18): sustained inconsistency only.
  auto_declare<double>("encoder_fault_window", 0.05);
  // Hard caps for operator-tuned steering motion (user 2026-10-18: speed at most 45 deg/s).
  auto_declare<double>("max_steering_slew_limit", 0.7853981633974483);
  auto_declare<double>("max_steering_acceleration_limit", 1.5707963267948966);
  // S-curve steering (user 2026-10-18): jerk limit; infinity keeps the trapezoidal profile.
  auto_declare<double>("max_steering_jerk", std::numeric_limits<double>::infinity());
  // Chassis twist S-curve (user 2026-10-22), wheel-rim units; infinity disables it.
  auto_declare<double>("max_body_acceleration", std::numeric_limits<double>::infinity());
  auto_declare<double>("max_body_jerk", std::numeric_limits<double>::infinity());
  auto_declare<double>("max_steering_jerk_limit", 17.453292519943297);
  for (const auto * name : {"steering_joints", "drive_joints", "steering_encoders"}) {
    auto_declare<std::vector<std::string>>(name, {});
  }
  for (const auto * name : {"module_x", "module_y", "wheel_radius", "steering_min", "steering_max",
      "steering_limit_margin", "steering_limit_tolerance", "pose_covariance", "twist_covariance"})
  {
    auto_declare<std::vector<double>>(name, {});
  }
  for (const auto * name : {"max_linear_speed", "max_angular_speed", "max_wheel_speed",
      "max_wheel_acceleration", "velocity_deadband", "max_encoder_difference", "max_update_period",
      "slip_residual_threshold",
      "alignment_threshold", "flip_hysteresis", "max_steering_slew", "feedback_timeout",
      "translation_heading_epsilon", "steering_angle_deadband",
      "imu_timeout", "max_imu_yaw_step", "quaternion_norm_tolerance",
      "imu_fallback_covariance_scale", "missing_module_covariance_scale",
      "slip_covariance_scale"})
  {
    auto_declare<double>(name, 0.0);
  }
  auto_declare<bool>("imu_enabled", false);
  // Startup mode is read at configure; runtime switches use the public stopped-only service.
  // align_first gates all four wheels together; steer_while_driving retains cosine scaling.
  auto_declare<std::string>("drive_mode", "steer_while_driving");
  auto_declare<double>("align_first_threshold", 0.08726646259971647);  // 5 deg
  auto_declare<double>("align_first_release", 0.05235987755982988);    // 3 deg
  auto_declare<std::string>("imu_frame_id", "");
  // Registered after every declaration: declaring a parameter also runs this check.
  steering_parameter_callback_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "drive_mode") {
          std::lock_guard<std::mutex> guard(lifecycle_mutex_);
          if (core_ || active_.load()) {
            result.successful = false;
            result.reason = "drive_mode is a startup default; use /control/set_chassis_align_first";
          } else if (
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            (parameter.as_string() != "align_first" &&
             parameter.as_string() != "steer_while_driving")) {
            result.successful = false;
            result.reason = "drive_mode must be align_first or steer_while_driving";
          }
          continue;
        }
        if (
          name == "alignment_threshold" || name == "align_first_threshold" ||
          name == "align_first_release") {
          const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                                 ? parameter.as_double()
                                 : std::numeric_limits<double>::quiet_NaN();
          if (!std::isfinite(value) || value <= 0.0 || value > M_PI / 2.0) {
            result.successful = false;
            result.reason = name + " must be in (0, pi/2] rad";
          }
          continue;
        }
        const char * limit_name = name == "max_steering_slew" ? "max_steering_slew_limit"
                                  : name == "max_steering_acceleration"
                                    ? "max_steering_acceleration_limit"
                                  : name == "max_steering_jerk" ? "max_steering_jerk_limit"
                                                                : nullptr;
        if (!limit_name) {
          continue;
        }
        const double limit = get_node()->get_parameter(limit_name).as_double();
        const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                               ? parameter.as_double()
                               : std::numeric_limits<double>::quiet_NaN();
        if (!std::isfinite(value) || value <= 0.0 || value > limit) {
          result.successful = false;
          result.reason = name + " must be in (0, " + std::to_string(limit) + "] rad";
        }
      }
      return result;
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (active_.load()) {return CallbackReturn::ERROR;}
  try {
    if (!get_node()->get_parameter("calibration_verified").as_bool()) {
      throw std::invalid_argument("Swerve calibration must be explicitly verified");
    }
    const std::set<std::string> removed{
      "kp", "kd", "ks", "kv", "ka", "kff_omega", "max_ff_speed_radps", "rezero_tolerance_rad"};
    for (const auto & name : get_node()->list_parameters({}, 100).names) {
      const auto dot = name.find_last_of('.');
      const auto leaf = dot == std::string::npos ? name : name.substr(dot + 1);
      if (removed.count(leaf)) {throw std::invalid_argument("Removed motor parameter: " + name);}
    }
    auto numbers = [this](const std::string & name, size_t size) {
        const auto values = get_node()->get_parameter(name).as_double_array();
        if (values.size() != size || !std::all_of(
            values.begin(), values.end(),
            [](double value) {return std::isfinite(value);}))
        {
          throw std::invalid_argument(name + " must contain finite values for every module");
        }
        return values;
      };
    auto number = [this](const char * name) {return get_node()->get_parameter(name).as_double();};
    const auto yaw_signs = numbers("steering_to_yaw_sign", 4);
    for (size_t i = 0; i < 4; ++i) {
      if (yaw_signs[i] != -1.0 && yaw_signs[i] != 1.0) {
        throw std::invalid_argument("steering_to_yaw_sign must be exactly +/-1");
      }
      steering_to_yaw_sign_[i] = yaw_signs[i];
    }
    const auto x = numbers("module_x", 4);
    const auto y = numbers("module_y", 4);
    const auto radius = numbers("wheel_radius", 4);
    const auto low = numbers("steering_min", 4);
    const auto high = numbers("steering_max", 4);
    const auto margin = numbers("steering_limit_margin", 4);
    const auto tolerance = numbers("steering_limit_tolerance", 4);
    for (size_t i = 0; i < 4; ++i) {
      config_.locations[i] = {x[i], y[i]};
      config_.wheel_radius[i] = radius[i];
      config_.steering_min[i] = low[i];
      config_.steering_max[i] = high[i];
      config_.steering_limit_margin[i] = margin[i];
      config_.steering_limit_tolerance[i] = tolerance[i];
    }
    config_.max_linear_speed = number("max_linear_speed");
    config_.max_angular_speed = number("max_angular_speed");
    config_.max_wheel_speed = number("max_wheel_speed");
    config_.max_wheel_acceleration = number("max_wheel_acceleration");
    config_.velocity_deadband = number("velocity_deadband");
    config_.max_encoder_difference = number("max_encoder_difference");
    config_.encoder_fault_window = number("encoder_fault_window");
    config_.max_body_acceleration = number("max_body_acceleration");
    config_.max_body_jerk = number("max_body_jerk");
    config_.max_update_period = number("max_update_period");
    config_.slip_residual_threshold = number("slip_residual_threshold");
    const auto drive_mode = get_node()->get_parameter("drive_mode").as_string();
    if (drive_mode == "align_first") {
      const double enter = number("align_first_threshold");
      const double release = number("align_first_release");
      if (!(release > 0.0 && release <= enter && enter <= M_PI / 2.0)) {
        throw std::invalid_argument(
          "align_first needs 0 < align_first_release <= align_first_threshold <= pi/2");
      }
      align_first_ = true;
      config_.setpoint.alignment_threshold_rad = enter;
      config_.setpoint.alignment_release_rad = release;
    } else if (drive_mode == "steer_while_driving") {
      align_first_ = false;
      config_.setpoint.alignment_threshold_rad = number("alignment_threshold");
      config_.setpoint.alignment_release_rad = std::numeric_limits<double>::quiet_NaN();
    } else {
      throw std::invalid_argument("drive_mode must be align_first or steer_while_driving");
    }
    RCLCPP_INFO(
      get_node()->get_logger(), "drive_mode %s: gate %.1f deg, release %.1f deg",
      drive_mode.c_str(), config_.setpoint.alignment_threshold_rad * 180.0 / M_PI,
      (align_first_ ? config_.setpoint.alignment_release_rad
                    : config_.setpoint.alignment_threshold_rad) *
        180.0 / M_PI);
    config_.setpoint.flip_hysteresis_rad = number("flip_hysteresis");
    config_.setpoint.maximum_steering_slew_radps = number("max_steering_slew");
    config_.setpoint.maximum_steering_acceleration_radps2 = number("max_steering_acceleration");
    config_.setpoint.maximum_steering_jerk_radps3 = number("max_steering_jerk");
    config_.setpoint.translation_heading_epsilon_rad = number("translation_heading_epsilon");
    config_.setpoint.steering_angle_deadband_rad = number("steering_angle_deadband");
    feedback_timeout_ = number("feedback_timeout");
    fault_stop_speed_ = number("fault_stop_speed");
    fault_stop_hold_ = number("fault_stop_hold");
    fault_stop_timeout_ = number("fault_stop_timeout");
    if (
      !positive(fault_stop_speed_) || !std::isfinite(fault_stop_hold_) || fault_stop_hold_ < 0.0 ||
      !positive(fault_stop_timeout_)) {
      throw std::invalid_argument("fault stop speed/hold/timeout must be configured");
    }
    if (!positive(feedback_timeout_)) {
      throw std::invalid_argument("feedback_timeout must be positive");
    }
    auto candidate = std::make_unique<ControlCore>(config_);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "body velocity shaping: %s, acceleration %.3f m/s^2, jerk %.3f m/s^3",
      std::isfinite(config_.max_body_acceleration) ? "enabled" : "disabled",
      config_.max_body_acceleration, config_.max_body_jerk);

    const auto steer = get_node()->get_parameter("steering_joints").as_string_array();
    const auto drive = get_node()->get_parameter("drive_joints").as_string_array();
    const auto encoders = get_node()->get_parameter("steering_encoders").as_string_array();
    std::set<std::string> used;
    for (const auto * list : {&steer, &drive, &encoders}) {
      if (list->size() != 4) {
        throw std::invalid_argument("Exactly four ordered module names required");
      }
      for (const auto & name : *list) {
        if (name.empty() || name.find("TBD") != std::string::npos ||
          name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")
          !=
          std::string::npos ||
          !used.insert(name).second)
        {
          throw std::invalid_argument("Swerve joint/sensor names must be explicit and unique");
        }
      }
    }
    command_names_.clear();
    state_names_.clear();
    for (size_t i = 0; i < 4; ++i) {
      command_names_.push_back(steer[i] + "/position");
      command_names_.push_back(drive[i] + "/velocity");
      for (const auto * field : {"position", "status_word", "mode_of_operation_display"}) {
        state_names_.push_back(steer[i] + "/" + field);
      }
      for (const auto * field :
        {"position", "velocity", "status_word", "mode_of_operation_display"})
      {
        state_names_.push_back(drive[i] + "/" + field);
      }
      state_names_.push_back(encoders[i] + "/position");
      state_names_.push_back(encoders[i] + "/feedback_age_ms");
    }
    state_names_.push_back("ethercat_domain/process_data_age_ms");
    OdometryParameters covariance;
    const auto pose = numbers("pose_covariance", 6);
    const auto twist = numbers("twist_covariance", 6);
    std::copy(pose.begin(), pose.end(), covariance.pose_covariance_diagonal.begin());
    std::copy(twist.begin(), twist.end(), covariance.twist_covariance_diagonal.begin());
    covariance.imu_fallback_covariance_scale = number("imu_fallback_covariance_scale");
    covariance.missing_module_covariance_scale = number("missing_module_covariance_scale");
    covariance.slip_covariance_scale = number("slip_covariance_scale");
    for (size_t i = 0; i < covariances_.size(); ++i) {
      covariances_[i] = make_odometry_covariances(
        covariance, (i & 1U) != 0, (i & 2U) != 0 ? 0U : 4U, (i & 4U) != 0);
      for (const auto * matrix : {&covariances_[i].pose, &covariances_[i].twist}) {
        if (!std::all_of(
            matrix->begin(), matrix->end(), [](double value) {
              return std::isfinite(value);
            }))
        {
          throw std::invalid_argument("Swerve covariance overflow");
        }
      }
    }
    imu_enabled_ = get_node()->get_parameter("imu_enabled").as_bool();
    imu_frame_ = get_node()->get_parameter("imu_frame_id").as_string();
    imu_timeout_ = number("imu_timeout");
    max_imu_yaw_step_ = number("max_imu_yaw_step");
    quaternion_tolerance_ = number("quaternion_norm_tolerance");
    if (imu_enabled_ && (imu_frame_.empty() || imu_frame_.find("TBD") != std::string::npos ||
      !positive(imu_timeout_) || !positive(max_imu_yaw_step_) || !positive(quaternion_tolerance_)))
    {
      throw std::invalid_argument("IMU frame and validation limits must be configured");
    }
    runtime_.reset();
    if (get_node()->get_parameter("relative.enabled").as_bool()) {
      runtime_ = std::make_shared<ChassisRuntime>(get_node(), config_);
      for (size_t i = 0; i < 4; ++i) {
        for (const auto * field :
          {"position", "mode_of_operation", "write_sequence", "write_mask"})
        {
          command_names_.push_back(drive[i] + "/" + field);
        }
        for (const auto * field : {"velocity", "feedback_age_ms"}) {
          state_names_.push_back(steer[i] + "/" + field);
        }
        for (const auto * field : {"feedback_age_ms", "feedback_sequence", "sent_sequence",
            "feedback_sequence_at_send", "sent_velocity", "mode_request_error", "mode_request_ack"})
        {
          state_names_.push_back(drive[i] + "/" + field);
        }
      }
    }
    commands_.assign(command_names_.size(), nullptr);
    states_.assign(state_names_.size(), nullptr);
    core_ = std::move(candidate);
    mode_pending_.store(0);
    mode_stationary_since_ns_ = 0;
    published_mode_.reset();
    mode_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "/rt_control/chassis/align_first", robot_interfaces_qos::latched());
    mode_service_ = get_node()->create_service<ModeService>(
      "/control/set_chassis_align_first",
      [this](
        ModeServer::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
        ModeService::Request::SharedPtr request) { request_drive_mode(service, header, request); });
    mode_timer_ =
      get_node()->create_wall_timer(std::chrono::milliseconds(20), [this]() { mode_timer(); });
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "~/odom", robot_interfaces_qos::fast_state());
    realtime_odometry_ =
      std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);
    realtime_odometry_->msg_.header.frame_id = "odom";
    realtime_odometry_->msg_.child_frame_id = "base_footprint";
    chassis_feedback_publisher_=get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "/rt_control/chassis/joint_states",rclcpp::SensorDataQoS());
    realtime_feedback_=std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      chassis_feedback_publisher_);
    auto & state=realtime_feedback_->msg_;
    state.header.frame_id="base_footprint";
    state.name={"module_1_steering","module_2_steering","module_3_steering","module_4_steering",
      "module_1_wheel","module_2_wheel","module_3_wheel","module_4_wheel"};
    state.position.resize(8);state.velocity.resize(8);feedback_published_ns_=0;

    command_subscription_.reset();
    imu_subscription_.reset();
    diagnostic_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", robot_interfaces_qos::diagnostic());
    // Latched, reliable: enable_manager subscribes and performs the ordered disable.
    fault_stop_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/fault_stop", rclcpp::QoS(1).reliable().transient_local());
    fault_stop_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(20), [this]() {
      std_msgs::msg::Bool message;
      message.data = active_.load() && fault_stop_ready_.load();
      fault_stop_publisher_->publish(message);
    });
    diagnostic_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {publish_diagnostics();});
    slipping_mask_.store(0U);
  } catch (const std::exception & error) {
    mode_timer_.reset();
    mode_service_.reset();
    mode_publisher_.reset();
    core_.reset();
    RCLCPP_ERROR(get_node()->get_logger(), "Swerve configuration rejected: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_names_};
}
controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

bool SwerveController::apply_steering_motion_parameters()
{
  const double slew = get_node()->get_parameter("max_steering_slew").as_double();
  const double acceleration = get_node()->get_parameter("max_steering_acceleration").as_double();
  const double jerk = get_node()->get_parameter("max_steering_jerk").as_double();
  // Each runtime mode has its own gate. Reload operator tuning only at activation.
  const double alignment =
    get_node()
      ->get_parameter(align_first_ ? "align_first_threshold" : "alignment_threshold")
      .as_double();
  const double release = align_first_ ? get_node()->get_parameter("align_first_release").as_double()
                                      : std::numeric_limits<double>::quiet_NaN();
  const bool same_release =
    release == config_.setpoint.alignment_release_rad ||
    (std::isnan(release) && std::isnan(config_.setpoint.alignment_release_rad));
  if (
    slew == config_.setpoint.maximum_steering_slew_radps &&
    acceleration == config_.setpoint.maximum_steering_acceleration_radps2 &&
    jerk == config_.setpoint.maximum_steering_jerk_radps3 &&
    alignment == config_.setpoint.alignment_threshold_rad && same_release) {
    return true;
  }
  if (!(alignment > 0.0 && alignment <= M_PI / 2.0)) {
    return false;
  }
  if (align_first_ && !(release > 0.0 && release <= alignment)) {
    RCLCPP_ERROR(get_node()->get_logger(), "align_first_release must be in (0, threshold]");
    return false;
  }
  // A mode-only reload must preserve already-configured unlimited acceleration/jerk.
  // New tuning values still obey the operator caps and the ControlCore constructor checks.
  if (
    jerk != config_.setpoint.maximum_steering_jerk_radps3 &&
    !(jerk > 0.0 && jerk <= get_node()->get_parameter("max_steering_jerk_limit").as_double())) {
    return false;
  }
  if (
    (slew != config_.setpoint.maximum_steering_slew_radps &&
     !(slew > 0.0 && slew <= get_node()->get_parameter("max_steering_slew_limit").as_double())) ||
    (acceleration != config_.setpoint.maximum_steering_acceleration_radps2 &&
     !(acceleration > 0.0 &&
       acceleration <= get_node()->get_parameter("max_steering_acceleration_limit").as_double()))) {
    return false;
  }
  auto config = config_;
  config.setpoint.maximum_steering_slew_radps = slew;
  config.setpoint.maximum_steering_acceleration_radps2 = acceleration;
  config.setpoint.maximum_steering_jerk_radps3 = jerk;
  config.setpoint.alignment_threshold_rad = alignment;
  config.setpoint.alignment_release_rad = release;
  try {
    auto candidate = std::make_unique<ControlCore>(config);
    core_ = std::move(candidate);
    config_ = config;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "steering motion parameters rejected: %s", error.what());
    return false;
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "steering motion: slew %.1f deg/s, acceleration %.1f deg/s^2, jerk %.1f deg/s^3, "
    "alignment gate %.1f deg, release %.1f deg",
    slew * 180.0 / M_PI, acceleration * 180.0 / M_PI, jerk * 180.0 / M_PI, alignment * 180.0 / M_PI,
    (align_first_ ? release : alignment) * 180.0 / M_PI);
  return true;
}

CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (!core_ || active_.load()) {return CallbackReturn::ERROR;}
  auto bind = [](const auto & names, auto & handles, auto & result) {
      if (names.size() != result.size() || handles.size() != result.size()) {return false;}
      std::fill(result.begin(), result.end(), nullptr);
      for (size_t i = 0; i < names.size(); ++i) {
        for (auto & handle : handles) {
          if (handle.get_name() == names[i]) {
            if (result[i]) {return false;}
            result[i] = &handle;
          }
        }
        if (!result[i]) {return false;}
      }
      return true;
    };
  if (!bind(
      command_names_, command_interfaces_,
      commands_) || !bind(state_names_, state_interfaces_, states_))
  {
    std::fill(commands_.begin(), commands_.end(), nullptr);
    std::fill(states_.begin(), states_.end(), nullptr);
    return CallbackReturn::ERROR;
  }
  last_steering_commands_.fill(std::numeric_limits<double>::quiet_NaN());
  if (!apply_steering_motion_parameters()) {
    std::fill(commands_.begin(), commands_.end(), nullptr);
    std::fill(states_.begin(), states_.end(), nullptr);
    return CallbackReturn::ERROR;
  }
  const auto feedback = read_feedback();
  if (!core_->activate(feedback)) {
    stop_outputs();
    std::fill(commands_.begin(), commands_.end(), nullptr);
    std::fill(states_.begin(), states_.end(), nullptr);
    return CallbackReturn::ERROR;
  }
  try {
    ++generation_;
    if (runtime_) {
      const auto now = steady_now();
      if (!runtime_->activate(
          static_cast<double>(now) * 1e-9,
          read_runtime_feedback(static_cast<double>(now) * 1e-9), static_cast<uint64_t>(now)))
      {
        throw std::runtime_error(
                "Cyclic handoff requires healthy stationary CSV and actually sent zero velocity");
      }
      for (size_t i = 0; i < 4; ++i) {
        commands_[8 + 4 * i]->set_value(states_[9 * i + 3]->get_value());
        commands_[8 + 4 * i + 1]->set_value(9.0);
        commands_[8 + 4 * i + 2]->set_value(0.0);
        commands_[8 + 4 * i + 3]->set_value(0.0);
      }
      was_navigation_ = false;
    }
    // Fresh subscriptions plus generation tags exclude queued callbacks from old activations.
    command_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", robot_interfaces_qos::control(),
      [this, generation = generation_](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        accept_command(*message, generation);
      });
    if (imu_enabled_) {
      imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        "~/imu", robot_interfaces_qos::fast_state(),
        [this, generation = generation_](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          accept_imu(*message, generation);
        });
    }
    block_before_ns_ = steady_now();
    command_buffer_.writeFromNonRT(Command{});
    imu_buffer_.writeFromNonRT(ImuSample{});
    publish_elapsed_ = 0.0;
    last_output_ = core_->update(feedback, {}, false, 0.004);
    if (!write_output(last_output_)) {
      throw std::runtime_error{"Swerve activation produced an unsafe output"};
    }
    status_.store(last_output_.status);
    slipping_mask_.store(0U);
    fault_latched_.store(false);
    fault_stop_ready_.store(false);
    ready_.store(true);
    active_.store(true);
  } catch (const std::exception & error) {
    stop_outputs();
    if (runtime_) {runtime_->deactivate();}
    core_->deactivate();
    std::fill(commands_.begin(), commands_.end(), nullptr);
    std::fill(states_.begin(), states_.end(), nullptr);
    command_subscription_.reset();
    imu_subscription_.reset();
    RCLCPP_ERROR(get_node()->get_logger(), "Swerve activation failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

ModuleFeedbackArray SwerveController::read_feedback() const
{
  ModuleFeedbackArray result{};
  auto recent = [this](double age) {
      return std::isfinite(age) && age >= 0.0 && age * 0.001 <= feedback_timeout_;
    };
  const bool bus_fresh = recent(states_[36]->get_value());
  for (size_t i = 0; i < 4; ++i) {
    const size_t offset = 9 * i;
    result[i] = {
      states_[offset]->get_value() * steering_to_yaw_sign_[i],
      states_[offset + 7]->get_value(),
      states_[offset + 3]->get_value(),
      states_[offset + 4]->get_value(),
      bus_fresh,
      enabled(states_[offset + 1]->get_value()) && states_[offset + 2]->get_value() == 8.0 &&
        enabled(states_[offset + 5]->get_value()) &&
        (states_[offset + 6]->get_value() == 9.0 ||
         (runtime_ && states_[offset + 6]->get_value() == 8.0)),
      recent(states_[offset + 8]->get_value())};
  }
  return result;
}

bool SwerveController::write_output(const ControlOutput & output)
{
  for (size_t i = 0; i < 4; ++i) {
    const SteeringAngleLimits limits{
      config_.steering_min[i], config_.steering_max[i],
      config_.steering_limit_margin[i], config_.steering_limit_tolerance[i]};
    if (!steering_angle_within_limits(output.steering_position[i], limits) ||
      !std::isfinite(output.drive_velocity[i]))
    {
      stop_outputs();
      return false;
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    const double position = output.steering_position[i] * steering_to_yaw_sign_[i];
    commands_[2 * i]->set_value(position);
    last_steering_commands_[i] = position;
    commands_[2 * i + 1]->set_value(output.drive_velocity[i]);
  }
  return true;
}

void SwerveController::stop_outputs()
{
  for (size_t i = 0; i < 4; ++i) {
    if (commands_.size() < 8 || states_.size() < 37) {return;}
    if (commands_[2 * i + 1]) {commands_[2 * i + 1]->set_value(0.0);}
    if (runtime_) {
      // Revoke sequence admission and retain the last CSP reference. Lifecycle owner
      // handles drive disable; this controller never fabricates a controlword.
      if (commands_[8 + 4 * i + 2]) {commands_[8 + 4 * i + 2]->set_value(0);}
      if (commands_[8 + 4 * i + 3]) {commands_[8 + 4 * i + 3]->set_value(0);}
      continue;
    }
    if (commands_[2 * i]) {
      // Keep the last written target; measurement jitter must not become a new target.
      const SteeringAngleLimits limits{
        config_.steering_min[i], config_.steering_max[i], config_.steering_limit_margin[i],
        config_.steering_limit_tolerance[i]};
      double position = last_steering_commands_[i];
      if (
        !std::isfinite(position) ||
        !steering_angle_within_limits(position * steering_to_yaw_sign_[i], limits)) {
        position =
          states_[9 * i] ? states_[9 * i]->get_value() : std::numeric_limits<double>::quiet_NaN();
      }
      if (
        std::isfinite(position) &&
        steering_angle_within_limits(position * steering_to_yaw_sign_[i], limits)) {
        commands_[2 * i]->set_value(position);
        last_steering_commands_[i] = position;
      }
    }
  }
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  active_.store(false);
  ready_.store(false);
  mode_stationary_since_ns_ = 0;
  command_subscription_.reset();
  imu_subscription_.reset();
  stop_outputs();
  if (runtime_) {runtime_->deactivate();}
  if (core_) {core_->deactivate();}
  std::fill(commands_.begin(), commands_.end(), nullptr);
  std::fill(states_.begin(), states_.end(), nullptr);
  release_interfaces();
  status_.store(ControlStatus::inactive);
  slipping_mask_.store(0U);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  on_deactivate(state);
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  command_subscription_.reset();
  imu_subscription_.reset();
  mode_timer_.reset();
  mode_pending_.store(0);
  if (mode_response_) {
    ModeService::Response response;
    response.success = false;
    response.message = "Controller cleanup; mode result not confirmed";
    try {
      mode_response_->service->send_response(mode_response_->header, response);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_node()->get_logger(), "mode cleanup response failed: %s", error.what());
    }
    mode_response_.reset();
  }
  mode_service_.reset();
  mode_publisher_.reset();
  diagnostic_timer_.reset();
  diagnostic_publisher_.reset();
  fault_stop_timer_.reset();
  fault_stop_publisher_.reset();
  realtime_feedback_.reset();chassis_feedback_publisher_.reset();
  realtime_odometry_.reset();
  odometry_publisher_.reset();
  runtime_.reset();
  core_.reset();
  return CallbackReturn::SUCCESS;
}
CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

void SwerveController::accept_command(
  const geometry_msgs::msg::Twist & message,
  uint64_t generation)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (!active_.load() || !ready_.load() || generation != generation_) {return;}
  Command command{{message.linear.x, message.linear.y, message.angular.z}, steady_now(),
    generation_,
    std::isfinite(message.linear.x) && std::isfinite(message.linear.y) && std::isfinite(
      message.angular.z) &&
    message.linear.z == 0.0 && message.angular.x == 0.0 && message.angular.y == 0.0};
  command.navigation_generation = runtime_ ? runtime_->navigation_generation() : 0;
  if (runtime_ && command.navigation_generation == 0) {return;}
  command_buffer_.writeFromNonRT(command);
}

void SwerveController::accept_imu(const sensor_msgs::msg::Imu & message, uint64_t generation)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (!active_.load() || generation != generation_) {return;}
  ImuSample sample{{}, steady_now(), generation_, false};
  const auto & q = message.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (message.header.frame_id == imu_frame_ && std::isfinite(norm) && norm > 0.0 &&
    std::abs(norm - 1.0) <= quaternion_tolerance_ && std::isfinite(
      message.orientation_covariance[0]) &&
    message.orientation_covariance[0] >= 0.0 && std::isfinite(message.angular_velocity.z) &&
    std::isfinite(message.angular_velocity_covariance[0]) &&
    message.angular_velocity_covariance[0] >= 0.0)
  {
    if (runtime_) {
      const auto stamp = rclcpp::Time(
        message.header.stamp,
        get_node()->get_clock()->get_clock_type());
      const double age = (get_node()->now() - stamp).seconds();
      if (stamp.nanoseconds() <= 0 || !std::isfinite(age) || age < 0 || age > imu_timeout_) {
        imu_buffer_.writeFromNonRT(sample); return;
      }
      // Preserve source age as well as local receipt; delayed IMU samples do not become fresh.
      sample.received_ns -= static_cast<int64_t>(age * 1e9);
    }
    sample.yaw = {std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        norm * norm - 2.0 * (q.y * q.y + q.z * q.z)), message.angular_velocity.z};
    const auto previous = *imu_buffer_.readFromNonRT();
    sample.valid = !previous.valid ||
      !fresh(previous.received_ns, sample.received_ns, imu_timeout_) ||
      std::abs(wrap_pi(sample.yaw.yaw_rad - previous.yaw.yaw_rad)) <= max_imu_yaw_step_;
  }
  imu_buffer_.writeFromNonRT(sample);
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!active_.load()) {return controller_interface::return_type::OK;}
  const auto now = steady_now();
  const auto feedback = read_feedback();
  const double dt = period.seconds();
  const int64_t feedback_ns=time.nanoseconds();
  const bool feedback_fresh=core_->feedback_ready(feedback);
  if(!feedback_fresh || feedback_ns<feedback_published_ns_) {feedback_published_ns_=0;}
  if(feedback_fresh && realtime_feedback_ &&
    (feedback_published_ns_==0 || feedback_ns-feedback_published_ns_>=10000000) && realtime_feedback_->trylock()) {
    auto & message=realtime_feedback_->msg_;message.header.stamp=time;
    const double feedback_dt=static_cast<double>(feedback_ns-feedback_published_ns_)*1.e-9;
    for(size_t i=0;i<4;++i) {
      message.position[i]=feedback[i].steering_position;
      message.position[i+4]=feedback[i].wheel_position;
      message.velocity[i]=feedback_published_ns_>0 && feedback_dt>0 && feedback_dt<.3 ?
        (feedback[i].steering_position-published_steering_[i])/feedback_dt : 0.;
      message.velocity[i+4]=feedback[i].wheel_velocity;
      published_steering_[i]=feedback[i].steering_position;
    }
    feedback_published_ns_=feedback_ns;realtime_feedback_->unlockAndPublish();
  }

  const bool ready = !core_->hold_fault_latched() && core_->feedback_ready(feedback) &&
                     positive(dt) && dt <= config_.max_update_period;
  ready_.store(ready);
  if (!ready) {block_before_ns_ = now;}
  const auto command = *command_buffer_.readFromRT();
  // N-04 uses the local receive interval, never a fabricated Twist timestamp.
  const bool command_fresh = command.valid && command.generation == generation_ &&
    command.received_ns > block_before_ns_ && fresh(command.received_ns, now, 0.5);
  const auto imu = *imu_buffer_.readFromRT();
  const auto yaw = imu_enabled_ && imu.valid && imu.generation == generation_ &&
    fresh(imu.received_ns, now, imu_timeout_) ? std::optional<YawSample>(imu.yaw) : std::nullopt;
  const bool navigation = !runtime_ || runtime_->navigation_open();
  if (runtime_ && navigation && !was_navigation_) {block_before_ns_ = now;}
  try {
    // Odometry observes every mode; only the selected command owner can write targets.
    last_output_ = core_->update(
      feedback, command.speeds,
      command_fresh && navigation && command.received_ns > block_before_ns_, dt, yaw);
  } catch (const std::exception &) {
    ready_.store(false); block_before_ns_ = now; stop_outputs();
    status_.store(ControlStatus::feedback_fault); slipping_mask_.store(0U);
    track_fault(ControlStatus::feedback_fault, feedback, now);
    return controller_interface::return_type::ERROR;
  }
  if (runtime_) {
    const auto out = runtime_->update(
      static_cast<double>(now) * 1e-9, dt, time.nanoseconds(),
      read_runtime_feedback(static_cast<double>(now) * 1e-9), last_output_,
      static_cast<double>(command.received_ns) * 1e-9, command.navigation_generation,
      command_fresh && command.received_ns > block_before_ns_);
    was_navigation_ = navigation;
    if (!write_runtime_output(out)) {
      stop_outputs(); ready_.store(false); return controller_interface::return_type::ERROR;
    }
  }
  if (last_output_.status == ControlStatus::steering_hold_fault) {
    ready_.store(false);
    block_before_ns_ = now;
  }
  if (last_output_.status == ControlStatus::steering_limit ||
    last_output_.status == ControlStatus::invalid_command)
  {
    block_before_ns_ = now;
  }
  if (!runtime_ && !write_output(last_output_)) {
    ready_.store(false);
    block_before_ns_ = now;
    status_.store(ControlStatus::steering_limit);
    slipping_mask_.store(0U);
    track_fault(ControlStatus::steering_limit, feedback, now);
    return controller_interface::return_type::ERROR;
  }
  status_.store(last_output_.status);
  track_fault(last_output_.status, feedback, now);
  if (fault_latched_.load()) {
    // A latched fault never resumes motion, even if feedback recovers.
    ready_.store(false);
    block_before_ns_ = now;
  }
  process_drive_mode(feedback, command, command_fresh, navigation, now);
  unsigned int slipping_mask{0U};
  for (size_t i = 0; i < last_output_.slipping_modules.size(); ++i) {
    if (last_output_.slipping_modules[i]) {slipping_mask |= 1U << i;}
  }
  slipping_mask_.store(slipping_mask);
  publish_elapsed_ += positive(dt) ? dt : 0.0;
  if (publish_elapsed_ >= 0.02 && realtime_odometry_->trylock()) {
    auto & message = realtime_odometry_->msg_;
    message.header.stamp = time;
    message.pose.pose.position.x = last_output_.pose.x_m;
    message.pose.pose.position.y = last_output_.pose.y_m;
    message.pose.pose.orientation.z = std::sin(last_output_.pose.heading_rad / 2.0);
    message.pose.pose.orientation.w = std::cos(last_output_.pose.heading_rad / 2.0);
    message.twist.twist.linear.x = last_output_.measured_twist.vx_mps;
    message.twist.twist.linear.y = last_output_.measured_twist.vy_mps;
    message.twist.twist.angular.z = last_output_.measured_twist.omega_radps;
    const size_t covariance = (last_output_.imu_fallback ? 1U : 0U) +
      (last_output_.valid_modules < 4 ? 2U : 0U) + (last_output_.slip_detected ? 4U : 0U);
    message.pose.covariance = covariances_[covariance].pose;
    message.twist.covariance = covariances_[covariance].twist;
    realtime_odometry_->unlockAndPublish();
    publish_elapsed_ = std::fmod(publish_elapsed_, 0.02);
  }
  return controller_interface::return_type::OK;
}

ChassisRuntimeFeedback SwerveController::read_runtime_feedback(double now)
{
  ChassisRuntimeFeedback out;
  const auto modules = read_feedback();
  const auto imu = *imu_buffer_.readFromRT();
  out.move.imu_age = now - static_cast<double>(imu.received_ns) * 1e-9;
  out.move.imu_yaw = imu.yaw.yaw_rad;
  out.move.imu_valid = imu.valid && imu.generation == generation_;
  const double bus_age = states_[36]->get_value() * 0.001;
  out.move.age = bus_age;
  out.move.bus_ok = std::isfinite(bus_age) && bus_age >= 0 && bus_age <= feedback_timeout_;
  out.move.drives_ok = true;
  auto sequence = [](double value) -> uint64_t {
      return std::isfinite(value) && value >= 0 && value <= 9007199254740991.0 &&
             std::trunc(value) == value ?
             static_cast<uint64_t>(value) : 0;
    };
  for (size_t i = 0; i < 4; ++i) {
    const size_t j = 9 * i, x = 37 + 9 * i;
    out.move.positions[i] =
    {modules[i].steering_position, modules[i].steering_angle, modules[i].wheel_position};
    out.move.wheel_velocity[i] = modules[i].wheel_velocity;
    out.move.steering_velocity[i] = states_[x]->get_value() * steering_to_yaw_sign_[i];
    out.move.steering_mode[i] = states_[j + 2]->get_value() == 8 ? 8 : 0;
    out.move.drive_mode[i] = states_[j + 6]->get_value() == 8 ? 8 : states_[j + 6]->get_value() ==
      9 ? 9 : 0;
    double age = bus_age;
    bool valid = modules[i].valid && modules[i].enabled;
    for (const auto index : {j + 8, x + 1, x + 2}) {
      const double seconds = states_[index]->get_value() * 0.001;
      valid = valid && std::isfinite(seconds) && seconds >= 0 && seconds <= feedback_timeout_;
      age = std::max(age, seconds);
    }
    valid = valid && states_[x + 7]->get_value() == 0;
    out.move.age = std::max(out.move.age, age);
    out.move.drives_ok = out.move.drives_ok && valid;
    const auto status = sequence(states_[j + 5]->get_value());
    out.drives[i] = {modules[i].wheel_position, modules[i].wheel_velocity,
      out.move.steering_velocity[i], now - age, sequence(states_[x + 3]->get_value()),
      static_cast<uint16_t>(status <= 65535 ? status : 0),
      static_cast<int8_t>(out.move.drive_mode[i]), valid, states_[x + 8]->get_value() == 1};
    const auto sent = sequence(states_[x + 4]->get_value());
    out.ack[i] = {sent > runtime_->wire_base() ? sent - runtime_->wire_base() : 0,
      sent > runtime_->wire_base() ? sequence(states_[x + 5]->get_value()) : 0};
    out.sent_velocity[i] = states_[x + 6]->get_value();
  }
  return out;
}
bool SwerveController::write_runtime_output(const ChassisRuntimeOutput & output)
{
  if (output.drive.inhibited) {return false;}
  const auto & d = output.drive;
  if (d.write_sequence > 9007199254740991ULL - runtime_->wire_base()) {return false;}
  for (size_t i = 0; i < 4; ++i) {
    const SteeringAngleLimits limits{config_.steering_min[i], config_.steering_max[i],
      config_.steering_limit_margin[i], config_.steering_limit_tolerance[i]};
    if (!steering_angle_within_limits(output.steering[i], limits) ||
      (d.write_position && !std::isfinite(d.wheel_position[i])) ||
      (d.write_velocity && !std::isfinite(d.wheel_velocity[i]))) {return false;}
  }
  for (size_t i = 0; i < 4; ++i) {
    commands_[2 * i]->set_value(output.steering[i] * steering_to_yaw_sign_[i]);
    last_steering_commands_[i] = output.steering[i] * steering_to_yaw_sign_[i];
    if (d.write_velocity) {commands_[2 * i + 1]->set_value(d.wheel_velocity[i]);}
    const size_t x = 8 + 4 * i;
    if (d.write_position) {commands_[x]->set_value(d.wheel_position[i]);}
    if (d.write_mode) {commands_[x + 1]->set_value(d.requested_mode);}
    const unsigned int mask = (d.write_velocity ? 1U : 0U) | (d.write_position ? 2U : 0U) |
      (d.write_mode ? 4U : 0U);
    commands_[x + 3]->set_value(mask);
    commands_[x + 2]->set_value(
      d.write_sequence ==
      0 ? 0 : static_cast<double>(runtime_->wire_base() + d.write_sequence));
  }
  return true;
}

void SwerveController::track_fault(
  ControlStatus status, const ModuleFeedbackArray & feedback, int64_t now)
{
  const bool fault = status == ControlStatus::feedback_fault ||
                     status == ControlStatus::steering_hold_fault ||
                     status == ControlStatus::steering_limit;
  if (fault && !fault_latched_.load()) {
    fault_latched_.store(true);
    fault_since_ns_ = now;
    wheels_stopped_since_ns_ = 0;
    uint32_t bits = 0U;
    double max_difference = 0.0;
    for (size_t i = 0; i < 4; ++i) {
      const auto & f = feedback[i];
      const double difference = std::abs(f.steering_position - f.steering_angle);
      const bool finite = std::isfinite(f.steering_position) && std::isfinite(f.steering_angle) &&
                          std::isfinite(f.wheel_position) && std::isfinite(f.wheel_velocity);
      uint32_t module = 0U;
      if (!f.valid) {
        module |= 1U;
      }
      if (!f.enabled) {
        module |= 2U;
      }
      if (finite && difference > config_.max_encoder_difference) {
        module |= 4U;
      }
      if (!finite) {
        module |= 8U;
      }
      if (finite) {
        max_difference = std::max(max_difference, difference);
      }
      bits |= module << (4U * i);
    }
    double worst = 0.0;
    int worst_module = 0;
    for (size_t i = 0; i < 4; ++i) {
      const double target = last_output_.steering_position[i];
      for (const double measured : {feedback[i].steering_position, feedback[i].steering_angle}) {
        const double deviation = std::abs(measured - target);
        if (std::isfinite(deviation) && deviation > worst) {
          worst = deviation;
          worst_module = static_cast<int>(i) + 1;
        }
      }
    }
    fault_target_deviation_mdeg_.store(static_cast<int32_t>(worst * 180000.0 / M_PI));
    fault_target_module_.store(worst_module);
    fault_status_code_.store(static_cast<int>(status));
    fault_module_bits_.store(bits);
    fault_max_difference_mdeg_.store(static_cast<int32_t>(max_difference * 180000.0 / M_PI));
  }
  if (!fault_latched_.load() || fault_stop_ready_.load()) {
    return;
  }
  bool stopped = true, measurable = true;
  for (size_t i = 0; i < 4; ++i) {
    const double speed = feedback[i].wheel_velocity * config_.wheel_radius[i];
    measurable = measurable && std::isfinite(speed);
    stopped = stopped && std::isfinite(speed) && std::abs(speed) <= fault_stop_speed_;
  }
  if (!stopped) {
    wheels_stopped_since_ns_ = 0;
  }
  if (stopped && wheels_stopped_since_ns_ == 0) {
    wheels_stopped_since_ns_ = now;
  }
  const double held = stopped ? static_cast<double>(now - wheels_stopped_since_ns_) * 1e-9 : 0.0;
  const double elapsed = static_cast<double>(now - fault_since_ns_) * 1e-9;
  // Disable only after the wheels are stopped; without wheel feedback, or after the timeout,
  // waiting longer cannot make the stop safer.
  if ((stopped && held >= fault_stop_hold_) || !measurable || elapsed >= fault_stop_timeout_) {
    fault_stop_ready_.store(true);
  }
}

void SwerveController::publish_diagnostics()
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  if (!diagnostic_publisher_) {return;}
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = get_node()->now();
  diagnostic_msgs::msg::DiagnosticStatus item;
  const auto status = status_.load();
  const auto slipping_mask = slipping_mask_.load();
  const bool slip_detected{slipping_mask != 0U};
  item.name = get_node()->get_name();
  item.hardware_id = "swerve_chassis";
  const bool error =
    status == ControlStatus::feedback_fault || status == ControlStatus::steering_hold_fault ||
    status == ControlStatus::steering_limit || status == ControlStatus::invalid_command;
  item.message = slip_detected && !error ? "wheel_slip" : status_name(status);
  item.level = error ? item.ERROR :
    (slip_detected || status != ControlStatus::running ? item.WARN : item.OK);
  if (fault_latched_.load()) {
    const int code = fault_status_code_.load();
    const uint32_t bits = fault_module_bits_.load();
    std::string modules;
    for (size_t i = 0; i < 4; ++i) {
      const uint32_t m = (bits >> (4U * i)) & 0xFU;
      if (m == 0U) {
        continue;
      }
      if (!modules.empty()) {
        modules += ' ';
      }
      modules += "module_" + std::to_string(i + 1) + ':';
      if (m & 1U) {
        modules += "stale,";
      }
      if (m & 2U) {
        modules += "not_enabled_or_mode,";
      }
      if (m & 4U) {
        modules += "encoder_difference,";
      }
      if (m & 8U) {
        modules += "non_finite,";
      }
    }
    diagnostic_msgs::msg::KeyValue latched;
    latched.key = "latched_fault";
    latched.value = code < 0 ? "unknown" : status_name(static_cast<ControlStatus>(code));
    item.values.push_back(latched);
    diagnostic_msgs::msg::KeyValue detail;
    detail.key = "latched_fault_modules";
    detail.value = modules.empty() ? "none" : modules;
    item.values.push_back(detail);
    diagnostic_msgs::msg::KeyValue difference;
    difference.key = "latched_fault_max_encoder_difference_mdeg";
    difference.value = std::to_string(fault_max_difference_mdeg_.load());
    item.values.push_back(difference);
    diagnostic_msgs::msg::KeyValue deviation;
    deviation.key = "latched_fault_target_deviation_mdeg";
    deviation.value = "module_" + std::to_string(fault_target_module_.load()) + ":" +
                      std::to_string(fault_target_deviation_mdeg_.load());
    item.values.push_back(deviation);
  }
  diagnostic_msgs::msg::KeyValue slip_value;
  slip_value.key = "slip_detected";
  slip_value.value = slip_detected ? "true" : "false";
  item.values.push_back(std::move(slip_value));
  diagnostic_msgs::msg::KeyValue modules_value;
  modules_value.key = "slipping_modules";
  constexpr std::array<const char *, 4> module_names{"FL", "FR", "RL", "RR"};
  for (size_t i = 0; i < module_names.size(); ++i) {
    if ((slipping_mask & (1U << i)) != 0U) {
      if (!modules_value.value.empty()) {modules_value.value += ',';}
      modules_value.value += module_names[i];
    }
  }
  modules_value.value = modules_value.value.empty() ? "none" : modules_value.value;
  item.values.push_back(std::move(modules_value));
  message.status.push_back(std::move(item));
  diagnostic_publisher_->publish(message);
}
}  // namespace swerve_driver

PLUGINLIB_EXPORT_CLASS(swerve_driver::SwerveController, controller_interface::ControllerInterface)
