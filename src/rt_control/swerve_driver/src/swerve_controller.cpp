#include "swerve_driver/swerve_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
  auto_declare<bool>("calibration_verified", false);
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
  auto_declare<std::string>("imu_frame_id", "");
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
    config_.max_update_period = number("max_update_period");
    config_.slip_residual_threshold = number("slip_residual_threshold");
    config_.setpoint.alignment_threshold_rad = number("alignment_threshold");
    config_.setpoint.flip_hysteresis_rad = number("flip_hysteresis");
    config_.setpoint.maximum_steering_slew_radps = number("max_steering_slew");
    config_.setpoint.translation_heading_epsilon_rad = number("translation_heading_epsilon");
    config_.setpoint.steering_angle_deadband_rad = number("steering_angle_deadband");
    feedback_timeout_ = number("feedback_timeout");
    if (!positive(feedback_timeout_)) {
      throw std::invalid_argument("feedback_timeout must be positive");
    }
    auto candidate = std::make_unique<ControlCore>(config_);

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
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "~/odom", robot_interfaces_qos::fast_state());
    realtime_odometry_ =
      std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);
    realtime_odometry_->msg_.header.frame_id = "odom";
    realtime_odometry_->msg_.child_frame_id = "base_footprint";
    command_subscription_.reset();
    imu_subscription_.reset();
    diagnostic_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", robot_interfaces_qos::diagnostic());
    diagnostic_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {publish_diagnostics();});
    slipping_mask_.store(0U);
  } catch (const std::exception & error) {
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
    result[i] = {states_[offset]->get_value(), states_[offset + 7]->get_value(),
      states_[offset + 3]->get_value(), states_[offset + 4]->get_value(),
      bus_fresh && recent(states_[offset + 8]->get_value()),
      enabled(states_[offset + 1]->get_value()) && states_[offset + 2]->get_value() == 8.0 &&
      enabled(states_[offset + 5]->get_value()) && (states_[offset + 6]->get_value() == 9.0 ||
      (runtime_ && states_[offset + 6]->get_value() == 8.0))};
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
    commands_[2 * i]->set_value(output.steering_position[i]);
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
    if (commands_[2 * i] && states_[9 * i]) {
      double position = states_[9 * i]->get_value();
      if (!std::isfinite(position) || position < config_.steering_min[i] ||
        position > config_.steering_max[i])
      {
        position = core_->hold_positions()[i];
      }
      if (std::isfinite(position) && position >= config_.steering_min[i] &&
        position <= config_.steering_max[i])
      {
        commands_[2 * i]->set_value(position);
      }
    }
  }
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> guard(lifecycle_mutex_);
  active_.store(false);
  ready_.store(false);
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
  diagnostic_timer_.reset();
  diagnostic_publisher_.reset();
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
  const bool ready = core_->feedback_ready(feedback) && positive(dt) &&
    dt <= config_.max_update_period;
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
    return controller_interface::return_type::ERROR;
  }
  status_.store(last_output_.status);
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
    out.move.steering_velocity[i] = states_[x]->get_value();
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
    commands_[2 * i]->set_value(output.steering[i]);
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
  const bool error = status == ControlStatus::feedback_fault ||
    status == ControlStatus::steering_limit || status == ControlStatus::invalid_command;
  item.message = slip_detected && !error ? "wheel_slip" : status_name(status);
  item.level = error ? item.ERROR :
    (slip_detected || status != ControlStatus::running ? item.WARN : item.OK);
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
