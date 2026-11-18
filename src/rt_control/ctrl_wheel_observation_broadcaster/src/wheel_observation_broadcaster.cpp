#include "ctrl_wheel_observation_broadcaster/wheel_observation_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <stdexcept>

#include "ctrl_swerve/swerve_kinematics.hpp"
#include "hw_semantic_components/cia402_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/observation_meta.hpp"

namespace ctrl_wheel_observation_broadcaster
{
namespace
{
using CallbackReturn = controller_interface::CallbackReturn;
using Meta = robot_rt_control_interfaces::msg::ObservationMeta;

// State interfaces per module, in this order.
enum Offset : std::size_t {
  kSteerPosition,
  kSteerStatus,
  kSteerAge,
  kSteerSequence,
  kDrivePosition,
  kDriveVelocity,
  kDriveStatus,
  kDriveAge,
  kDriveSequence,
  kEncoderPosition,
  kEncoderAge,
  kPerModule
};
// No wheel turns this far within one 1 ms sample (about 100 m at 0.1 m radius): a larger
// step is a counter wrap or reset, never a displacement.
constexpr double kMaxDrivePositionStep = 1000.0;
// Session suffix width: the session string keeps its length, so updating it does not allocate.
constexpr int kSessionDigits = 6;

bool faulted(double status_word)
{
  if (!std::isfinite(status_word) || status_word < 0.0 || status_word > 65535.0) {
    return true;
  }
  const auto state =
    hw_semantic_components::decode_cia402_state(static_cast<std::uint16_t>(status_word));
  return state == hw_semantic_components::Cia402State::kFault ||
         state == hw_semantic_components::Cia402State::kFaultReactionActive;
}

// A drive feedback_sequence counts its complete PDO cycles.
ChannelInput counted_sample(
  double sequence, double previous, double age_ms, bool finite, bool fault)
{
  ChannelInput input{age_ms, finite, fault, false, false};
  input.new_sample = std::isfinite(sequence) && sequence > 0.0 && sequence != previous;
  input.discontinuity = input.new_sample && sequence < previous;
  return input;
}

void fill(
  robot_rt_control_interfaces::msg::WheelFeedbackChannel & channel, const ChannelState & state,
  std::uint8_t time_source)
{
  channel.sample_stamp.sec = static_cast<std::int32_t>(state.sample_ns / 1000000000);
  channel.sample_stamp.nanosec = static_cast<std::uint32_t>(state.sample_ns % 1000000000);
  channel.time_source = time_source;
  channel.sample_sequence = state.sample_sequence;
  channel.epoch = state.epoch;
  channel.valid = state.valid;
  channel.reason = static_cast<std::uint8_t>(state.reason);
}
}  // namespace

CallbackReturn WheelObservationBroadcaster::on_init()
{
  for (const auto * name : {"steering_joints", "drive_joints", "steering_encoders"}) {
    auto_declare<std::vector<std::string>>(name, {});
  }
  for (const auto * name : {"module_x", "module_y", "wheel_radius", "steering_to_yaw_sign"}) {
    auto_declare<std::vector<double>>(name, {});
  }
  auto_declare<double>("feedback_timeout", 0.0);
  auto_declare<double>("publish_rate", 0.0);
  for (const auto * name : {"chassis_calibration_id", "encoder_calibration_id", "startup_id"}) {
    auto_declare<std::string>(name, "");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelObservationBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    auto numbers = [this](const char * name) {
      const auto values = get_node()->get_parameter(name).as_double_array();
      if (values.size() != 4U || !std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
          })) {
        throw std::invalid_argument(std::string(name) + " needs four finite values");
      }
      return values;
    };
    const auto x = numbers("module_x");
    const auto y = numbers("module_y");
    const auto radius = numbers("wheel_radius");
    const auto sign = numbers("steering_to_yaw_sign");
    std::set<std::size_t> slots;
    for (std::size_t i = 0; i < 4U; ++i) {
      const auto quadrant = ctrl_swerve::module_quadrant({x[i], y[i]});
      if (
        quadrant == ctrl_swerve::ModuleQuadrant::unknown ||
        !slots.insert(static_cast<std::size_t>(quadrant)).second) {
        throw std::invalid_argument("module_x/module_y must place one module in each quadrant");
      }
      if (!(radius[i] > 0.0) || (sign[i] != 1.0 && sign[i] != -1.0)) {
        throw std::invalid_argument("wheel_radius must be positive and steering_to_yaw_sign +/-1");
      }
      modules_[i].slot = static_cast<std::size_t>(quadrant);
      modules_[i].yaw_sign = sign[i];
      x_[i] = x[i];
      y_[i] = y[i];
      radius_[i] = radius[i];
    }
    const double timeout = get_node()->get_parameter("feedback_timeout").as_double();
    const double rate = get_node()->get_parameter("publish_rate").as_double();
    if (!(std::isfinite(timeout) && timeout > 0.0) || !(std::isfinite(rate) && rate > 0.0)) {
      throw std::invalid_argument("feedback_timeout and publish_rate must be positive");
    }
    timeout_ms_ = timeout * 1000.0;
    publish_period_ = 1.0 / rate;
    const auto chassis = get_node()->get_parameter("chassis_calibration_id").as_string();
    const auto encoders = get_node()->get_parameter("encoder_calibration_id").as_string();
    if (chassis.empty() || encoders.empty()) {
      throw std::invalid_argument("chassis and encoder calibration ids are required");
    }
    calibration_id_ = chassis + ";" + encoders;
    const auto startup = get_node()->get_parameter("startup_id").as_string();
    session_prefix_ = (startup.empty() ? std::string(get_node()->get_name()) : startup) + ":";

    const auto steer = get_node()->get_parameter("steering_joints").as_string_array();
    const auto drive = get_node()->get_parameter("drive_joints").as_string_array();
    const auto encoder = get_node()->get_parameter("steering_encoders").as_string_array();
    if (steer.size() != 4U || drive.size() != 4U || encoder.size() != 4U) {
      throw std::invalid_argument("Exactly four steering, drive and encoder names required");
    }
    state_names_.clear();
    for (std::size_t i = 0; i < 4U; ++i) {
      for (const auto * field :
           {"position", "status_word", "feedback_age_ms", "feedback_sequence"}) {
        state_names_.push_back(steer[i] + "/" + field);
      }
      for (const auto * field :
           {"position", "velocity", "status_word", "feedback_age_ms", "feedback_sequence"}) {
        state_names_.push_back(drive[i] + "/" + field);
      }
      state_names_.push_back(encoder[i] + "/position");
      state_names_.push_back(encoder[i] + "/feedback_age_ms");
    }
    if (
      std::set<std::string>(state_names_.begin(), state_names_.end()).size() !=
      state_names_.size()) {
      throw std::invalid_argument("Steering, drive and encoder names must be unique");
    }

    publisher_ = get_node()->create_publisher<Message>(
      "/rt/base/wheel_observations", robot_interfaces_qos::fast_state());
    realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<Message>>(publisher_);
    auto & message = realtime_publisher_->msg_;
    message.header.frame_id = "base_footprint";
    message.source_id = get_node()->get_node_base_interface()->get_fully_qualified_name();
    message.calibration_id = calibration_id_;
    message.session_id.reserve(session_prefix_.size() + kSessionDigits + 1U);
    for (std::size_t i = 0; i < 4U; ++i) {
      auto & wheel = message.wheels[modules_[i].slot];
      wheel.wheel_id = static_cast<std::uint8_t>(modules_[i].slot);
      wheel.steering_axis_x = x_[i];
      wheel.steering_axis_y = y_[i];
      wheel.wheel_radius = radius_[i];
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Wheel observation configuration rejected: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WheelObservationBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
WheelObservationBroadcaster::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

CallbackReturn WheelObservationBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  if (!realtime_publisher_ || state_interfaces_.size() != state_names_.size()) {
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < state_names_.size(); ++i) {
    if (state_interfaces_[i].get_name() != state_names_[i]) {
      return CallbackReturn::ERROR;
    }
  }
  start_session(0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelObservationBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelObservationBroadcaster::on_cleanup(const rclcpp_lifecycle::State &)
{
  realtime_publisher_.reset();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

double WheelObservationBroadcaster::state(std::size_t module, std::size_t offset) const
{
  return state_interfaces_[kPerModule * module + offset].get_value();
}

// A new session: restarted counters and continuity, so no increment spans two sessions.
void WheelObservationBroadcaster::start_session(std::int64_t time_ns)
{
  ++session_count_;
  sequence_ = 0U;
  publish_elapsed_ = publish_period_;
  last_time_ns_ = time_ns;
  for (auto & module : modules_) {
    module.drive.reset();
    module.steering_motor.reset();
    module.steering_encoder.reset();
    module.drive_sequence = module.steering_sequence = 0.0;
    module.encoder_age_ms = std::numeric_limits<double>::infinity();
    module.drive_position = std::numeric_limits<double>::quiet_NaN();
  }
}

controller_interface::return_type WheelObservationBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const std::int64_t now_ns = time.nanoseconds();
  if (now_ns < last_time_ns_) {
    start_session(now_ns);
  }
  last_time_ns_ = now_ns;
  for (std::size_t i = 0; i < modules_.size(); ++i) {
    auto & module = modules_[i];
    const double steer_sequence = state(i, kSteerSequence);
    module.steering_motor.update(
      counted_sample(
        steer_sequence, module.steering_sequence, state(i, kSteerAge),
        std::isfinite(state(i, kSteerPosition)), faulted(state(i, kSteerStatus))),
      now_ns, timeout_ms_);
    module.steering_sequence = steer_sequence;

    const double drive_sequence = state(i, kDriveSequence);
    const double position = state(i, kDrivePosition);
    auto drive = counted_sample(
      drive_sequence, module.drive_sequence, state(i, kDriveAge),
      std::isfinite(position) && std::isfinite(state(i, kDriveVelocity)),
      faulted(state(i, kDriveStatus)));
    drive.discontinuity =
      drive.discontinuity ||
      (drive.new_sample && std::abs(position - module.drive_position) > kMaxDrivePositionStep);
    module.drive.update(drive, now_ns, timeout_ms_);
    module.drive_sequence = drive_sequence;
    if (drive.new_sample && std::isfinite(position)) {
      module.drive_position = position;
    }

    // The encoder exposes no counter: its receive age falls only when a newer frame arrived
    // (one frame per 4 ms SYNC, read every 1 ms cycle).
    const double age = state(i, kEncoderAge);
    const bool fresh_age = std::isfinite(age) && age >= 0.0;
    ChannelInput encoder{age, std::isfinite(state(i, kEncoderPosition)), false, false, false};
    encoder.new_sample =
      fresh_age && (!std::isfinite(module.encoder_age_ms) || age < module.encoder_age_ms);
    module.steering_encoder.update(encoder, now_ns, timeout_ms_);
    module.encoder_age_ms = fresh_age ? age : std::numeric_limits<double>::infinity();
  }

  publish_elapsed_ += std::max(0.0, period.seconds());
  if (publish_elapsed_ < publish_period_ || !realtime_publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }
  publish_elapsed_ = std::fmod(publish_elapsed_, publish_period_);
  auto & message = realtime_publisher_->msg_;
  message.header.stamp = time;
  char suffix[kSessionDigits + 1];
  std::snprintf(
    suffix, sizeof(suffix), "%0*llu", kSessionDigits,
    static_cast<unsigned long long>(session_count_ % 1000000ULL));  // NOLINT(runtime/int)
  message.session_id.assign(session_prefix_).append(suffix);
  message.sequence = ++sequence_;
  for (std::size_t i = 0; i < modules_.size(); ++i) {
    const auto & module = modules_[i];
    auto & wheel = message.wheels[module.slot];
    wheel.drive_position = state(i, kDrivePosition);
    wheel.drive_velocity = state(i, kDriveVelocity);
    wheel.steering_motor_angle = state(i, kSteerPosition) * module.yaw_sign;
    wheel.steering_encoder_angle = state(i, kEncoderPosition);
    fill(wheel.drive, module.drive.state(), Meta::TIME_SOURCE_READ_COMPLETE);
    fill(wheel.steering_motor, module.steering_motor.state(), Meta::TIME_SOURCE_READ_COMPLETE);
    fill(wheel.steering_encoder, module.steering_encoder.state(), Meta::TIME_SOURCE_HOST_RECEIVE);
  }
  realtime_publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

}  // namespace ctrl_wheel_observation_broadcaster

PLUGINLIB_EXPORT_CLASS(
  ctrl_wheel_observation_broadcaster::WheelObservationBroadcaster,
  controller_interface::ControllerInterface)
