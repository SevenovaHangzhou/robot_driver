// 阅读顺序：构造函数创建接口 -> poll_all_inputs() 定时读取 -> on_pump_command() 控制泵。
// TCP 连接、报文组装和分包接收放在 modbus_client.hpp 中。
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "modbus_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace plc_io_modbus
{
// Modbus 功能码：读线圈、读数字输入、读模拟输入、写单个线圈。
constexpr uint8_t kReadCoils = 0x01;
constexpr uint8_t kReadDiscreteInputs = 0x02;
constexpr uint8_t kReadInputRegisters = 0x04;
constexpr uint8_t kWriteSingleCoil = 0x05;
constexpr uint16_t kCoilOn = 0xFF00;
constexpr uint16_t kCoilOff = 0x0000;

class IoModuleNode : public rclcpp::Node
{
public:
  IoModuleNode() : Node("plc_io_modbus")
  {
    // 1. 声明参数。启动时，YAML 中同名参数会覆盖这些默认值。
    declare_parameter("host", "192.168.1.12");
    declare_parameter("port", 502);
    declare_parameter("unit_id", 1);
    const double poll_period_seconds = declare_parameter("poll_period", 0.2);
    declare_parameter("relay_address", 0);
    declare_parameter("vacuum_register", 0);
    declare_parameter("vacuum_scale", 1.0);
    declare_parameter("vacuum_offset", 0.0);
    declare_parameter("laser_address", 0);
    declare_parameter("laser_bit", 0);
    if (!std::isfinite(poll_period_seconds) || poll_period_seconds < 1e-6) {
      throw std::invalid_argument("poll_period must be finite and at least 0.000001 s");
    }

    // 2. 创建两个话题和一个服务。队列长度为 10。
    vacuum_publisher_ = create_publisher<std_msgs::msg::Float32>("/plc_io/vacuum_sensor", 10);
    laser_publisher_ = create_publisher<std_msgs::msg::Bool>("/plc_io/infrared_laser", 10);
    pump_service_ = create_service<std_srvs::srv::SetBool>("/plc_io/vacuum_pump",
      [this](std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
        on_pump_command(request, response);
      });

    // 3. 周期读取。只有服务回调可以写继电器；定时器不会发送 ON/OFF。
    poll_timer_ = create_wall_timer(std::chrono::duration<double>(poll_period_seconds),
      [this]() {poll_all_inputs();});
  }

private:
  // 读取整数配置并检查范围，避免把错误地址发送给模块。
  int read_integer_parameter(const char * name, int minimum, int maximum)
  {
    const auto value = get_parameter(name).as_int();
    if (value < minimum || value > maximum) {
      throw std::invalid_argument(std::string(name) + " out of range");
    }
    return static_cast<int>(value);
  }

  void ensure_connected()
  {
    const auto host = get_parameter("host").as_string();
    if (host.empty()) {
      throw std::invalid_argument("host must not be empty");
    }
    const int port = read_integer_parameter("port", 1, 65535);
    const int unit_id = read_integer_parameter("unit_id", 0, 255);
    // 已连接时直接返回；此前断开时重新连接。
    modbus_.connect(host, port, unit_id);
  }

  // 服务：接收 true/false -> 写线圈 -> 校验回显 -> 返回 success/message。
  void on_pump_command(
    const std_srvs::srv::SetBool::Request::SharedPtr & request,
    const std_srvs::srv::SetBool::Response::SharedPtr & response)
  {
    try {
      const auto address = read_integer_parameter("relay_address", 0, 65535);
      ensure_connected();
      const uint16_t output_value = request->data ? kCoilOn : kCoilOff;
      const auto reply = modbus_.request(kWriteSingleCoil, address, output_value);

      // FC05 正常响应：[功能码][地址高][地址低][写入值高][写入值低]。
      // 先检查长度，再取数据，避免越界访问。
      if (reply.size() != 5) {
        throw std::runtime_error("coil write echo mismatch");
      }
      const auto echoed_address = ModbusClient::read_uint16_be(reply.data() + 1);
      const auto echoed_value = ModbusClient::read_uint16_be(reply.data() + 3);
      if (echoed_address != address || echoed_value != output_value) {
        throw std::runtime_error("coil write echo mismatch");
      }
      // 仅证明模块回复与命令一致，不证明继电器实际吸合或泵正在运转。
      response->success = true;
      response->message = "ok";
    } catch (const ModbusException & error) {
      // 模块明确拒绝了请求，但连接仍可继续使用。
      response->success = false;
      response->message = error.what();
    } catch (const std::exception & error) {
      modbus_.close();
      response->success = false;
      response->message = error.what();
    }
  }

  // 一个周期按固定顺序读取三类数据。每次调用都有自己的异常处理，
  // 所以前一路报错后，仍会尝试后一路。三类操作共用一条 TCP 连接。
  void poll_all_inputs()
  {
    poll_with_error_handling("relay", [this]() {read_relay_status();});
    poll_with_error_handling("vacuum", [this]() {read_and_publish_vacuum();});
    poll_with_error_handling("laser", [this]() {read_and_publish_laser();});
  }

  // 传入读取函数，统一处理连接和日志，不在业务函数里重复写 try/catch。
  void poll_with_error_handling(
    const char * device_name, const std::function<void()> & read_function)
  {
    try {
      ensure_connected();
      read_function();
    } catch (const ModbusException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s request rejected: %s", device_name, error.what());
    } catch (const std::exception & error) {
      modbus_.close();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s poll failed: %s", device_name, error.what());
    }
  }

  // 继电器：只读自身线圈，使连接保持有正常请求；不反复写入开启命令。
  void read_relay_status()
  {
    const auto address = read_integer_parameter("relay_address", 0, 65535);
    const auto reply = modbus_.request(kReadCoils, address, 1);
    // 一个线圈只需一个数据字节，正文共 3 字节：[01][01][线圈数据]。
    if (reply.size() != 3 || reply[1] != 1) {
      throw std::runtime_error("invalid relay coil readback response");
    }
  }

  // 真空：读取一个 16 位无符号原始值，按 YAML 比例换算后发布。
  void read_and_publish_vacuum()
  {
    const auto address = read_integer_parameter("vacuum_register", 0, 65535);
    const double scale = get_parameter("vacuum_scale").as_double();
    const double offset = get_parameter("vacuum_offset").as_double();
    if (!std::isfinite(scale) || !std::isfinite(offset)) {
      throw std::invalid_argument("vacuum scaling must be finite");
    }

    // 若模块返回异常码 3，request() 会抛异常，直接到外层日志处理，
    // 不会继续做长度检查、换算或发布数据。
    const auto reply = modbus_.request(kReadInputRegisters, address, 1);
    // 正常响应：[04][02][原始值高字节][原始值低字节]。
    if (reply.size() != 4 || reply[1] != 2) {
      throw std::runtime_error("invalid analog input response");
    }
    const uint16_t raw_value = ModbusClient::read_uint16_be(reply.data() + 2);
    const double engineering_value = raw_value * scale + offset;
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(engineering_value);
    if (!std::isfinite(message.data)) {
      throw std::runtime_error("analog value overflow");
    }
    vacuum_publisher_->publish(message);
  }

  // 激光：读取数字输入，发布高/低电平，不是距离数值。
  void read_and_publish_laser()
  {
    const auto address = read_integer_parameter("laser_address", 0, 65535);
    const int bit_offset = read_integer_parameter("laser_bit", 0, 15);
    if (address + bit_offset > 65535) {
      throw std::invalid_argument("DI range overflow");
    }
    const int input_count = bit_offset + 1;
    const auto reply = modbus_.request(kReadDiscreteInputs, address, input_count);
    const size_t data_byte_count = (input_count + 7) / 8;
    if (reply.size() != data_byte_count + 2 || reply[1] != data_byte_count) {
      throw std::runtime_error("invalid digital input response");
    }

    // 每字节存放 8 个输入，低位对应较小地址；前两个字节是功能码和字节数。
    const size_t byte_index = 2 + bit_offset / 8;
    const int bit_in_byte = bit_offset % 8;
    const bool input_is_high = (reply[byte_index] & (1 << bit_in_byte)) != 0;
    std_msgs::msg::Bool message;
    message.data = input_is_high;
    laser_publisher_->publish(message);
  }

  ModbusClient modbus_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vacuum_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr laser_publisher_;
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
    // 单线程依次执行定时器与服务，避免两个请求交错使用同一个 socket。
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("plc_io_modbus"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
