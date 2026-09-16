// 阅读顺序：构造函数创建接口 -> poll_all_inputs() 轮询 -> on_output_command() 写输出。
// 公共 /vacuum/* 接口仍由 control_api_adapter 提供，本节点只负责 Modbus 硬件适配。
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "modbus_client.hpp"
#include "plc_io_modbus/msg/vacuum_sensor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rt_control_interfaces/msg/plc_io_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

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

class IoModuleNode : public rclcpp::Node
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

    declare_parameter("digital.inputs.infrared_laser.di_address", 0);
    declare_parameter("digital.outputs.vacuum_pump_relay.do_address", 0);

    declare_parameter("analog.inputs.vacuum_sensor.register", 0);
    declare_parameter("analog.inputs.vacuum_sensor.raw_at_zero_kpa", 1000.0);
    declare_parameter("analog.inputs.vacuum_sensor.raw_at_full_scale", 5000.0);
    declare_parameter("analog.inputs.vacuum_sensor.full_scale_kpa", -101.0);
    declare_parameter("analog.inputs.vacuum_sensor.valid_raw_min", 800);
    declare_parameter("analog.inputs.vacuum_sensor.valid_raw_max", 5200);
    declare_parameter("analog.inputs.vacuum_sensor.attached_threshold_kpa",
      std::numeric_limits<double>::quiet_NaN());
    declare_parameter("analog.inputs.vacuum_sensor.released_threshold_kpa", 0.0);

    if (!std::isfinite(poll_period_seconds) || poll_period_seconds < 1e-3) {
      throw std::invalid_argument("poll_period must be finite and at least 0.001 s");
    }
    digital_config_ = read_module_config("digital.module", "DI/DO");
    analog_config_ = read_module_config("analog.module", "AI/AO");

    vacuum_publisher_ = create_publisher<plc_io_modbus::msg::VacuumSensorState>(
      "/plc_io/vacuum_sensor", 10);
    laser_publisher_ = create_publisher<std_msgs::msg::Bool>("/plc_io/infrared_laser", 10);
    plc_state_publisher_ = create_publisher<rt_control_interfaces::msg::PlcIoState>(
      "/plc/io_state", 10);

    pump_service_ = create_output_service(
      "/plc/vacuum_pump", "digital.outputs.vacuum_pump_relay.do_address", &pump_on_);

    poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(poll_period_seconds), [this]() {poll_all_inputs();});
  }

private:
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

  void ensure_connected(ModbusClient & client, const ModuleConfig & config)
  {
    client.connect(config.host, config.port, config.unit_id);
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr create_output_service(
    const std::string & service_name, const std::string & address_parameter, bool * state)
  {
    return create_service<std_srvs::srv::SetBool>(service_name,
      [this, service_name, address_parameter, state](
        std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        on_output_command(service_name, address_parameter, *state, request->data, *response);
      });
  }

  void on_output_command(
    const std::string & service_name, const std::string & address_parameter,
    bool & state, bool enabled, std_srvs::srv::SetBool::Response & response)
  {
    try {
      const int address = read_integer_parameter(address_parameter, 0, 65535);
      ensure_connected(digital_client_, digital_config_);

      // 先读输出状态，使重复的 ON/ON 或 OFF/OFF 调用保持幂等，不会反转线圈。
      if (read_single_bit(kReadCoils, static_cast<uint16_t>(address)) != enabled) {
        const uint16_t output_value = enabled ? kCoilOn : kCoilOff;
        const auto reply = digital_client_.request(
          kWriteSingleCoil, static_cast<uint16_t>(address), output_value);
        if (reply.size() != 5 ||
          ModbusClient::read_uint16_be(reply.data() + 1) != address ||
          ModbusClient::read_uint16_be(reply.data() + 3) != output_value)
        {
          throw std::runtime_error("coil write echo mismatch");
        }
      }

      // 输出读回只证明 IO 模块的线圈状态，绝不作为“工件已吸牢”的依据。
      state = read_single_bit(kReadCoils, static_cast<uint16_t>(address));
      if (state != enabled) {
        throw std::runtime_error("coil state readback mismatch");
      }
      response.success = true;
      response.message = service_name + (enabled ? "=ON verified" : "=OFF verified");
      digital_error_.clear();
    } catch (const ModbusException & error) {
      response.success = false;
      response.message = error.what();
      digital_error_ = error.what();
    } catch (const std::exception & error) {
      digital_client_.close();
      response.success = false;
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
    poll_and_publish_vacuum();
    publish_plc_state();
  }

  void poll_digital_module()
  {
    try {
      ensure_connected(digital_client_, digital_config_);
      read_and_publish_laser();
      pump_on_ = read_single_bit(
        kReadCoils,
        static_cast<uint16_t>(read_integer_parameter(
          "digital.outputs.vacuum_pump_relay.do_address", 0, 65535)));
      digital_fresh_ = true;
      digital_error_.clear();
    } catch (const ModbusException & error) {
      digital_error_ = error.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "DI/DO module request rejected: %s", error.what());
    } catch (const std::exception & error) {
      digital_client_.close();
      digital_error_ = error.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "DI/DO module poll failed: %s", error.what());
    }
  }

  void poll_and_publish_vacuum()
  {
    plc_io_modbus::msg::VacuumSensorState message;
    message.header.frame_id = "vacuum_sensor";
    message.pressure_kpa = std::numeric_limits<float>::quiet_NaN();
    message.unit = "kPa";
    message.valid = false;
    vacuum_valid_ = false;
    left_attached_ = false;
    right_attached_ = false;
    vacuum_released_ = false;
    vacuum_pressure_kpa_ = std::numeric_limits<float>::quiet_NaN();

    try {
      ensure_connected(analog_client_, analog_config_);
      const int address = read_integer_parameter(
        "analog.inputs.vacuum_sensor.register", 0, 65535);
      const double raw_at_zero =
        get_parameter("analog.inputs.vacuum_sensor.raw_at_zero_kpa").as_double();
      const double raw_at_full =
        get_parameter("analog.inputs.vacuum_sensor.raw_at_full_scale").as_double();
      const double full_scale_kpa =
        get_parameter("analog.inputs.vacuum_sensor.full_scale_kpa").as_double();
      const int valid_min = read_integer_parameter(
        "analog.inputs.vacuum_sensor.valid_raw_min", 0, 65535);
      const int valid_max = read_integer_parameter(
        "analog.inputs.vacuum_sensor.valid_raw_max", 0, 65535);
      if (!std::isfinite(raw_at_zero) || !std::isfinite(raw_at_full) ||
        !std::isfinite(full_scale_kpa) || raw_at_full <= raw_at_zero || valid_max < valid_min)
      {
        throw std::invalid_argument("invalid vacuum conversion parameters");
      }

      // 已确认的 AE0830 CH0：FC04，单个 16 位无符号寄存器，原始单位为 mV。
      const auto reply = analog_client_.request(
        kReadInputRegisters, static_cast<uint16_t>(address), 1);
      if (reply.size() != 4 || reply[1] != 2) {
        throw std::runtime_error("invalid analog input response");
      }
      message.raw_millivolts = ModbusClient::read_uint16_be(reply.data() + 2);
      message.pressure_kpa = static_cast<float>(
        (message.raw_millivolts - raw_at_zero) * full_scale_kpa /
        (raw_at_full - raw_at_zero));
      analog_connected_ = true;

      if (message.raw_millivolts < valid_min || message.raw_millivolts > valid_max) {
        message.error = "analog input outside configured valid range";
      } else if (!std::isfinite(message.pressure_kpa)) {
        message.error = "vacuum pressure conversion overflow";
      } else {
        message.valid = true;
        vacuum_valid_ = true;
        vacuum_pressure_kpa_ = message.pressure_kpa;
        update_vacuum_state_from_pressure(message.pressure_kpa);
      }
      analog_error_ = message.error;
    } catch (const ModbusException & error) {
      analog_error_ = error.what();
      message.error = error.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "AI/AO module request rejected: %s", error.what());
    } catch (const std::exception & error) {
      analog_client_.close();
      analog_error_ = error.what();
      message.error = error.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "AI/AO module poll failed: %s", error.what());
    }

    // 模块没有采样时钟：时间戳表示本次响应处理完成时间，valid 表示本帧是否可用。
    message.header.stamp = now();
    vacuum_publisher_->publish(message);
  }

  void update_vacuum_state_from_pressure(float pressure_kpa)
  {
    const double threshold =
      get_parameter("analog.inputs.vacuum_sensor.attached_threshold_kpa").as_double();
    // 单泵单传感器没有左右物理通道；两个公共通道名看到的是同一条真空回路。
    const bool established = std::isfinite(threshold) && pressure_kpa <= threshold;
    left_attached_ = established;
    right_attached_ = established;
    const double released_threshold =
      get_parameter("analog.inputs.vacuum_sensor.released_threshold_kpa").as_double();
    vacuum_released_ = std::isfinite(released_threshold) && pressure_kpa >= released_threshold;
  }

  void read_and_publish_laser()
  {
    const int address = read_integer_parameter(
      "digital.inputs.infrared_laser.di_address", 0, 65535);
    std_msgs::msg::Bool message;
    message.data = read_single_bit(kReadDiscreteInputs, static_cast<uint16_t>(address));
    laser_publisher_->publish(message);
  }

  void publish_plc_state()
  {
    rt_control_interfaces::msg::PlcIoState message;
    message.header.stamp = now();
    message.header.frame_id = "plc";
    message.connected = digital_fresh_ && analog_connected_;
    // Action 只有在输出状态和模拟量吸附反馈都新鲜时才允许判定成功。
    message.data_fresh = digital_fresh_ && vacuum_valid_;
    message.left_vacuum_established = left_attached_;
    message.right_vacuum_established = right_attached_;
    // 该硬件没有独立左右阀；两个兼容字段都反映唯一的泵继电器读回状态。
    message.left_solenoid_on = pump_on_;
    message.right_solenoid_on = pump_on_;
    message.vacuum_pump_on = pump_on_;
    message.vacuum_pressure_valid = vacuum_valid_;
    message.vacuum_pressure_kpa = vacuum_pressure_kpa_;
    message.vacuum_released = vacuum_released_;
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
  bool digital_fresh_{false};
  bool analog_connected_{false};
  bool vacuum_valid_{false};
  bool left_attached_{false};
  bool right_attached_{false};
  bool vacuum_released_{false};
  bool pump_on_{false};
  float vacuum_pressure_kpa_{std::numeric_limits<float>::quiet_NaN()};
  std::string digital_error_;
  std::string analog_error_;
  rclcpp::Publisher<plc_io_modbus::msg::VacuumSensorState>::SharedPtr vacuum_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr laser_publisher_;
  rclcpp::Publisher<rt_control_interfaces::msg::PlcIoState>::SharedPtr plc_state_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pump_service_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};
}  // namespace plc_io_modbus

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<plc_io_modbus::IoModuleNode>();
    // 单线程串行化定时器和服务，防止同一 Modbus TCP socket 上的请求交错。
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("plc_io_modbus"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
