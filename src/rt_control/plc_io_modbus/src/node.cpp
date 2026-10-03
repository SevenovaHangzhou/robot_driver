// Hardware-only Modbus bridge. Public cross-domain interfaces are provided by
// control_api_adapter; this node owns the serialised socket transactions.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "modbus_client.hpp"
#include "plc_io_modbus/msg/vacuum_sensor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_qos/profiles.hpp"
#include "robot_rt_control_interfaces/msg/digital_input_state.hpp"
#include "robot_rt_control_interfaces/msg/digital_input_state_array.hpp"
#include "robot_rt_control_interfaces/msg/observation_meta.hpp"
#include "robot_system_interfaces/msg/error_info.hpp"
#include "rt_control_interfaces/msg/plc_io_state.hpp"
#include "rt_control_interfaces/srv/set_digital_output.hpp"
#include "std_msgs/msg/bool.hpp"

namespace plc_io_modbus
{
constexpr uint8_t kReadCoils = 0x01;
constexpr uint8_t kReadDiscreteInputs = 0x02;
constexpr uint8_t kReadInputRegisters = 0x04;
constexpr uint8_t kWriteSingleCoil = 0x05;
constexpr uint16_t kCoilOn = 0xFF00;
constexpr uint16_t kCoilOff = 0x0000;

struct ModuleConfig
{
  std::string name;
  std::string host;
  int port;
  int unit_id;
};

struct PressureConfig
{
  std::string channel;
  std::string sensor_id;
  int address;
  double raw_at_zero_kpa;
  double raw_at_full_scale;
  double full_scale_kpa;
  int valid_raw_min;
  int valid_raw_max;
};

struct PressureSample
{
  int32_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  uint16_t raw{0};
  float pressure_kpa{std::numeric_limits<float>::quiet_NaN()};
  bool valid{false};
  std::string error;
};

class IoModuleNode final : public rclcpp::Node
{
public:
  IoModuleNode() : Node("plc_io_modbus")
  {
    declare_parameter("digital.module.host", "192.168.1.12");
    declare_parameter("digital.module.port", 502);
    declare_parameter("digital.module.unit_id", 1);
    declare_parameter("analog.module.host", "192.168.1.13");
    declare_parameter("analog.module.port", 502);
    declare_parameter("analog.module.unit_id", 1);
    const double poll_period_seconds = declare_parameter("poll_period", 0.2);

    infrared_configured_ = declare_parameter("digital.inputs.infrared_laser.configured", false);
    infrared_active_high_ = declare_parameter("digital.inputs.infrared_laser.active_high", true);
    declare_parameter("digital.inputs.infrared_laser.di_address", 0);

    vacuum_configured_ = declare_parameter("vacuum_system.configured", false);
    declare_parameter("digital.outputs.vacuum_pump_relay.do_address", -1);
    declare_parameter("digital.outputs.left_vacuum_valve.do_address", -1);
    declare_parameter("digital.outputs.right_vacuum_valve.do_address", -1);
    declare_pressure_parameters("left", "left_vacuum_sensor");
    declare_pressure_parameters("right", "right_vacuum_sensor");

    if (!std::isfinite(poll_period_seconds) || poll_period_seconds < 1e-3) {
      throw std::invalid_argument("poll_period must be finite and at least 0.001 s");
    }
    digital_config_ = read_module_config("digital.module", "DI/DO");
    analog_config_ = read_module_config("analog.module", "AI/AO");
    validate_parameters();
    initialise_instance_id();

    for (size_t index = 0; index < pressure_publishers_.size(); ++index) {
      const auto channel = index == 0U ? "left" : "right";
      pressure_publishers_[index] = create_publisher<plc_io_modbus::msg::VacuumSensorState>(
        std::string("/plc_io/vacuum/") + channel, 10);
    }
    laser_publisher_ = create_publisher<std_msgs::msg::Bool>("/plc_io/infrared_laser", 10);
    infrared_state_publisher_ =
      create_publisher<robot_rt_control_interfaces::msg::DigitalInputStateArray>(
      "/infrared/state", robot_interfaces_qos::state());
    plc_state_publisher_ = create_publisher<rt_control_interfaces::msg::PlcIoState>(
      "/plc/io_state", 10);

    pump_service_ = create_output_service(
      "/plc/vacuum_pump", "digital.outputs.vacuum_pump_relay.do_address", &pump_on_);
    left_valve_service_ = create_output_service(
      "/plc/vacuum_valve/left", "digital.outputs.left_vacuum_valve.do_address",
      &left_valve_on_);
    right_valve_service_ = create_output_service(
      "/plc/vacuum_valve/right", "digital.outputs.right_vacuum_valve.do_address",
      &right_valve_on_);

    poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(poll_period_seconds), [this]() {poll_all_inputs();});
  }

private:
  using SetDigitalOutput = rt_control_interfaces::srv::SetDigitalOutput;

  void validate_parameters() const
  {
    if (vacuum_configured_) {
      validate_vacuum_configuration();
    }
    if (infrared_configured_) {
      (void)read_integer_parameter("digital.inputs.infrared_laser.di_address", 0, 65535);
    }
  }

  void initialise_instance_id()
  {
    std::random_device random;
    for (auto & byte : producer_instance_id_.uuid) {
      byte = static_cast<uint8_t>(random());
    }
  }

  void declare_pressure_parameters(const std::string & side, const std::string & sensor_id)
  {
    const auto prefix = "analog.inputs." + side + "_vacuum_sensor";
    declare_parameter(prefix + ".sensor_id", sensor_id);
    declare_parameter(prefix + ".register", -1);
    declare_parameter(prefix + ".raw_at_zero_kpa", 1000.0);
    declare_parameter(prefix + ".raw_at_full_scale", 5000.0);
    declare_parameter(prefix + ".full_scale_kpa", -101.0);
    declare_parameter(prefix + ".valid_raw_min", 800);
    declare_parameter(prefix + ".valid_raw_max", 5200);
  }

  int read_integer_parameter(const std::string & name, int minimum, int maximum) const
  {
    const auto value = get_parameter(name).as_int();
    if (value < minimum || value > maximum) {
      throw std::invalid_argument(name + " out of range");
    }
    return static_cast<int>(value);
  }

  ModuleConfig read_module_config(const std::string & prefix, const std::string & name) const
  {
    const auto host = get_parameter(prefix + ".host").as_string();
    if (host.empty()) {
      throw std::invalid_argument(prefix + ".host must not be empty");
    }
    return ModuleConfig{
      name, host,
      read_integer_parameter(prefix + ".port", 1, 65535),
      read_integer_parameter(prefix + ".unit_id", 0, 255)};
  }

  PressureConfig read_pressure_config(const std::string & side) const
  {
    const auto prefix = "analog.inputs." + side + "_vacuum_sensor";
    PressureConfig config{
      side,
      get_parameter(prefix + ".sensor_id").as_string(),
      read_integer_parameter(prefix + ".register", 0, 65535),
      get_parameter(prefix + ".raw_at_zero_kpa").as_double(),
      get_parameter(prefix + ".raw_at_full_scale").as_double(),
      get_parameter(prefix + ".full_scale_kpa").as_double(),
      read_integer_parameter(prefix + ".valid_raw_min", 0, 65535),
      read_integer_parameter(prefix + ".valid_raw_max", 0, 65535)};
    if (config.sensor_id.empty() || !std::isfinite(config.raw_at_zero_kpa) ||
      !std::isfinite(config.raw_at_full_scale) || !std::isfinite(config.full_scale_kpa) ||
      config.raw_at_full_scale <= config.raw_at_zero_kpa ||
      config.valid_raw_max < config.valid_raw_min)
    {
      throw std::invalid_argument(prefix + " conversion parameters are invalid");
    }
    return config;
  }

  void validate_vacuum_configuration() const
  {
    const std::array<int, 3> outputs{
      read_integer_parameter("digital.outputs.vacuum_pump_relay.do_address", 0, 65535),
      read_integer_parameter("digital.outputs.left_vacuum_valve.do_address", 0, 65535),
      read_integer_parameter("digital.outputs.right_vacuum_valve.do_address", 0, 65535)};
    if (outputs[0] == outputs[1] || outputs[0] == outputs[2] || outputs[1] == outputs[2]) {
      throw std::invalid_argument("vacuum pump and valve output addresses must be distinct");
    }
    const auto left = read_pressure_config("left");
    const auto right = read_pressure_config("right");
    if (left.address == right.address) {
      throw std::invalid_argument("left and right pressure registers must be distinct");
    }
  }

  void ensure_connected(ModbusClient & client, const ModuleConfig & config)
  {
    client.connect(config.host, config.port, config.unit_id);
  }

  rclcpp::Service<SetDigitalOutput>::SharedPtr create_output_service(
    const std::string & service_name, const std::string & address_parameter, bool * state)
  {
    return create_service<SetDigitalOutput>(service_name,
      [this, service_name, address_parameter, state](
        SetDigitalOutput::Request::SharedPtr request,
        SetDigitalOutput::Response::SharedPtr response)
      {
        on_output_command(service_name, address_parameter, *state, request->enabled, *response);
      });
  }

  void on_output_command(
    const std::string & service_name, const std::string & address_parameter,
    bool & state, bool enabled, SetDigitalOutput::Response & response)
  {
    if (!vacuum_configured_) {
      response.accepted = false;
      response.outcome = SetDigitalOutput::Response::OUTCOME_NOT_EXECUTED;
      response.observation_valid = false;
      response.message = "vacuum hardware is not configured";
      return;
    }

    response.accepted = true;
    bool write_may_have_occurred = false;
    try {
      const int address = read_integer_parameter(address_parameter, 0, 65535);
      ensure_connected(digital_client_, digital_config_);
      if (read_single_bit(kReadCoils, static_cast<uint16_t>(address)) != enabled) {
        const uint16_t output_value = enabled ? kCoilOn : kCoilOff;
        write_may_have_occurred = true;
        const auto reply = digital_client_.request(
          kWriteSingleCoil, static_cast<uint16_t>(address), output_value);
        if (reply.size() != 5 ||
          ModbusClient::read_uint16_be(reply.data() + 1) != address ||
          ModbusClient::read_uint16_be(reply.data() + 3) != output_value)
        {
          throw std::runtime_error("coil write echo mismatch");
        }
      }
      state = read_single_bit(kReadCoils, static_cast<uint16_t>(address));
      response.observed_value = state;
      response.observation_valid = true;
      if (state != enabled) {
        throw std::runtime_error("coil state readback mismatch");
      }
      response.outcome = SetDigitalOutput::Response::OUTCOME_CONFIRMED;
      response.message = service_name + (enabled ? "=ON verified" : "=OFF verified");
      digital_error_.clear();
    } catch (const std::exception & error) {
      digital_client_.close();
      response.outcome = write_may_have_occurred ?
        SetDigitalOutput::Response::OUTCOME_UNKNOWN :
        SetDigitalOutput::Response::OUTCOME_NOT_EXECUTED;
      response.observation_valid = false;
      response.message = error.what();
      digital_error_ = error.what();
    }
    publish_plc_state();
  }

  bool read_single_bit(uint8_t function, uint16_t address)
  {
    const auto reply = digital_client_.request(function, address, 1);
    if (reply.size() != 3 || reply[1] != 1) {
      throw std::runtime_error("invalid single-bit response");
    }
    return (reply[2] & 0x01) != 0;
  }

  void poll_all_inputs()
  {
    digital_fresh_ = false;
    analog_connected_ = false;
    poll_digital_module();
    poll_pressure_channels();
    publish_plc_state();
  }

  void poll_digital_module()
  {
    if (!infrared_configured_ && !vacuum_configured_) {
      publish_infrared_state(false, false, false, "infrared input is not configured");
      digital_error_ = "no digital IO capability is configured";
      return;
    }
    try {
      ensure_connected(digital_client_, digital_config_);
      if (infrared_configured_) {
        read_and_publish_laser();
      } else {
        publish_infrared_state(false, false, false, "infrared input is not configured");
      }
      if (vacuum_configured_) {
        pump_on_ = read_output("digital.outputs.vacuum_pump_relay.do_address");
        left_valve_on_ = read_output("digital.outputs.left_vacuum_valve.do_address");
        right_valve_on_ = read_output("digital.outputs.right_vacuum_valve.do_address");
        pump_output_valid_ = true;
        left_valve_output_valid_ = true;
        right_valve_output_valid_ = true;
      }
      digital_fresh_ = true;
      digital_error_.clear();
    } catch (const std::exception & error) {
      digital_client_.close();
      pump_output_valid_ = false;
      left_valve_output_valid_ = false;
      right_valve_output_valid_ = false;
      digital_error_ = error.what();
      if (infrared_configured_) {
        publish_infrared_state(false, false, false, error.what());
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "DI/DO module poll failed: %s", error.what());
    }
  }

  bool read_output(const std::string & parameter)
  {
    return read_single_bit(
      kReadCoils, static_cast<uint16_t>(read_integer_parameter(parameter, 0, 65535)));
  }

  void poll_pressure_channels()
  {
    left_pressure_ = PressureSample{};
    right_pressure_ = PressureSample{};
    if (!vacuum_configured_) {
      analog_error_ = "vacuum hardware is not configured";
      publish_pressure("left", left_pressure_, 0U);
      publish_pressure("right", right_pressure_, 1U);
      return;
    }

    try {
      ensure_connected(analog_client_, analog_config_);
      left_pressure_ = read_pressure(read_pressure_config("left"));
      right_pressure_ = read_pressure(read_pressure_config("right"));
      analog_connected_ = true;
      analog_error_.clear();
    } catch (const std::exception & error) {
      analog_client_.close();
      analog_error_ = error.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "AI/AO module poll failed: %s", error.what());
    }
    publish_pressure("left", left_pressure_, 0U);
    publish_pressure("right", right_pressure_, 1U);
  }

  PressureSample read_pressure(const PressureConfig & config)
  {
    PressureSample sample;
    const auto reply = analog_client_.request(
      kReadInputRegisters, static_cast<uint16_t>(config.address), 1);
    if (reply.size() != 4 || reply[1] != 2) {
      throw std::runtime_error("invalid analog input response for " + config.channel);
    }
    sample.raw = ModbusClient::read_uint16_be(reply.data() + 2);
    sample.pressure_kpa = static_cast<float>(
      (sample.raw - config.raw_at_zero_kpa) * config.full_scale_kpa /
      (config.raw_at_full_scale - config.raw_at_zero_kpa));
    const auto stamp_ns = now().nanoseconds();
    sample.stamp_sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    sample.stamp_nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    if (sample.raw < config.valid_raw_min || sample.raw > config.valid_raw_max) {
      sample.error = "analog input outside configured valid range";
    } else if (!std::isfinite(sample.pressure_kpa)) {
      sample.error = "vacuum pressure conversion overflow";
    } else {
      sample.valid = true;
    }
    return sample;
  }

  void publish_pressure(const std::string & channel, const PressureSample & sample, size_t index)
  {
    plc_io_modbus::msg::VacuumSensorState message;
    message.header.stamp.sec = sample.stamp_sec;
    message.header.stamp.nanosec = sample.stamp_nanosec;
    message.header.frame_id = channel + "_vacuum_sensor";
    message.channel = channel;
    message.sensor_id = vacuum_configured_ ?
      get_parameter("analog.inputs." + channel + "_vacuum_sensor.sensor_id").as_string() : "";
    message.raw_millivolts = sample.raw;
    message.pressure_kpa = sample.pressure_kpa;
    message.unit = "kPa";
    message.valid = sample.valid;
    message.error = sample.error.empty() && !vacuum_configured_ ?
      "vacuum hardware is not configured" : sample.error;
    pressure_publishers_[index]->publish(message);
  }

  static void fill_error(
    robot_system_interfaces::msg::ErrorInfo & error, uint32_t code,
    const std::string & message, bool retryable)
  {
    error.code = code;
    error.message = message;
    error.retryable = retryable;
    error.severity = code == 0U ?
      robot_system_interfaces::msg::ErrorInfo::OK :
      robot_system_interfaces::msg::ErrorInfo::FAULT;
    error.source = "rt_control";
    error.detail = "";
  }

  void publish_infrared_state(
    bool raw_level, bool active, bool valid, const std::string & error_text)
  {
    using ObservationMeta = robot_rt_control_interfaces::msg::ObservationMeta;
    robot_rt_control_interfaces::msg::DigitalInputStateArray array;
    array.header.stamp = now();
    robot_rt_control_interfaces::msg::DigitalInputState input;
    input.input_id = "infrared_laser";
    input.side = "";
    input.raw_level = raw_level;
    input.active = active;
    if (valid) {
      input.observation.stamp = array.header.stamp;
      input.observation.time_source = ObservationMeta::TIME_SOURCE_READ_COMPLETE;
      input.observation.sample_sequence = ++infrared_sample_sequence_;
    } else {
      input.observation.time_source = ObservationMeta::TIME_SOURCE_UNKNOWN;
      input.observation.sample_sequence = infrared_sample_sequence_;
    }
    input.observation.valid = valid;
    input.observation.source_instance_id = producer_instance_id_;
    fill_error(
      input.observation.error,
      valid ? 0U : (infrared_configured_ ? 1100U : 10U),
      error_text,
      infrared_configured_ && !valid);
    array.inputs.push_back(std::move(input));
    infrared_state_publisher_->publish(array);
  }

  void read_and_publish_laser()
  {
    const int address = read_integer_parameter(
      "digital.inputs.infrared_laser.di_address", 0, 65535);
    const bool raw_level = read_single_bit(kReadDiscreteInputs, static_cast<uint16_t>(address));
    std_msgs::msg::Bool legacy;
    legacy.data = raw_level;
    laser_publisher_->publish(legacy);
    publish_infrared_state(
      raw_level, infrared_active_high_ ? raw_level : !raw_level, true, "");
  }

  void publish_plc_state()
  {
    rt_control_interfaces::msg::PlcIoState message;
    message.header.stamp = now();
    message.header.frame_id = "plc";
    message.hardware_configured = vacuum_configured_;
    message.connected = vacuum_configured_ && digital_fresh_ && analog_connected_;
    message.data_fresh = message.connected && left_pressure_.valid && right_pressure_.valid &&
      pump_output_valid_ && left_valve_output_valid_ && right_valve_output_valid_;
    message.pump_output_valid = pump_output_valid_;
    message.vacuum_pump_on = pump_on_;
    message.left_valve_output_valid = left_valve_output_valid_;
    message.left_solenoid_on = left_valve_on_;
    message.right_valve_output_valid = right_valve_output_valid_;
    message.right_solenoid_on = right_valve_on_;
    message.left_pressure_stamp.sec = left_pressure_.stamp_sec;
    message.left_pressure_stamp.nanosec = left_pressure_.stamp_nanosec;
    message.left_pressure_sensor_id = vacuum_configured_ ?
      get_parameter("analog.inputs.left_vacuum_sensor.sensor_id").as_string() : "";
    message.left_pressure_raw = left_pressure_.raw;
    message.left_pressure_valid = left_pressure_.valid;
    message.left_pressure_kpa = left_pressure_.pressure_kpa;
    message.right_pressure_stamp.sec = right_pressure_.stamp_sec;
    message.right_pressure_stamp.nanosec = right_pressure_.stamp_nanosec;
    message.right_pressure_sensor_id = vacuum_configured_ ?
      get_parameter("analog.inputs.right_vacuum_sensor.sensor_id").as_string() : "";
    message.right_pressure_raw = right_pressure_.raw;
    message.right_pressure_valid = right_pressure_.valid;
    message.right_pressure_kpa = right_pressure_.pressure_kpa;
    message.io_alarm = 0;
    message.error = digital_error_;
    if (!analog_error_.empty()) {
      if (!message.error.empty()) {
        message.error += "; ";
      }
      message.error += analog_error_;
    }
    plc_state_publisher_->publish(message);
  }

  ModuleConfig digital_config_;
  ModuleConfig analog_config_;
  ModbusClient digital_client_;
  ModbusClient analog_client_;
  bool vacuum_configured_{false};
  bool infrared_configured_{false};
  bool infrared_active_high_{true};
  bool digital_fresh_{false};
  bool analog_connected_{false};
  bool pump_output_valid_{false};
  bool left_valve_output_valid_{false};
  bool right_valve_output_valid_{false};
  bool pump_on_{false};
  bool left_valve_on_{false};
  bool right_valve_on_{false};
  PressureSample left_pressure_;
  PressureSample right_pressure_;
  std::string digital_error_;
  std::string analog_error_;
  unique_identifier_msgs::msg::UUID producer_instance_id_;
  uint64_t infrared_sample_sequence_{0};
  std::array<rclcpp::Publisher<plc_io_modbus::msg::VacuumSensorState>::SharedPtr, 2>
    pressure_publishers_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr laser_publisher_;
  rclcpp::Publisher<robot_rt_control_interfaces::msg::DigitalInputStateArray>::SharedPtr
    infrared_state_publisher_;
  rclcpp::Publisher<rt_control_interfaces::msg::PlcIoState>::SharedPtr plc_state_publisher_;
  rclcpp::Service<SetDigitalOutput>::SharedPtr pump_service_;
  rclcpp::Service<SetDigitalOutput>::SharedPtr left_valve_service_;
  rclcpp::Service<SetDigitalOutput>::SharedPtr right_valve_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};
}  // namespace plc_io_modbus

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<plc_io_modbus::IoModuleNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("plc_io_modbus"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
