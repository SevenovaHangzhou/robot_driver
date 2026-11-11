// Python binding of the public sender. Protocol state remains in robot_interfaces.
#include <chrono>
#include <memory>
#include <thread>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <robot_interfaces_rolling_client/rolling_sender.hpp>

namespace py = pybind11;
namespace rolling = robot_interfaces_rolling_client;

class Sender
{
public:
  explicit Sender(int ack_timeout_ms)
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    node_ = std::make_shared<rclcpp::Node>("operator_jog_rolling_sender", options);
    sender_ =
      std::make_unique<rolling::RollingSender>(*node_, std::chrono::milliseconds(ack_timeout_ms));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    thread_ = std::thread([this]() { executor_->spin(); });
  }

  ~Sender() { close(); }

  void close()
  {
    if (executor_) {
      executor_->cancel();
      if (thread_.joinable()) {
        thread_.join();
      }
      executor_.reset();
      sender_.reset();
      node_.reset();
      context_->shutdown("jog sender closed");
    }
  }

  int send(const py::dict & input)
  {
    rolling::Window window;
    const auto start = input["replace_from"].cast<py::dict>();
    window.replace_from.sec = start["sec"].cast<int32_t>();
    window.replace_from.nanosec = start["nanosec"].cast<uint32_t>();
    window.final = input["final"].cast<bool>();
    for (const auto & value : input["points"].cast<py::list>()) {
      const auto item = value.cast<py::dict>();
      const auto time = item["time_from_start"].cast<py::dict>();
      rolling::Point point;
      point.time_from_start.sec = time["sec"].cast<int32_t>();
      point.time_from_start.nanosec = time["nanosec"].cast<uint32_t>();
      point.positions = item["positions"].cast<decltype(point.positions)>();
      point.velocities = item["velocities"].cast<decltype(point.velocities)>();
      window.points.push_back(std::move(point));
    }
    return static_cast<int>(sender_->send(window));
  }

  py::dict status() const
  {
    const auto state = sender_->status();
    py::dict result;
    result["phase"] = static_cast<int>(state.phase);
    result["controller_state"] = state.controller_state;
    result["epoch"] = state.epoch;
    result["first_sequence"] = state.first_sequence;
    result["last_sent_sequence"] = state.last_sent_sequence;
    result["rejections"] = state.rejections;
    result["last_rejection"] = state.last_rejection;
    result["last_rejected_sequence"] = state.last_rejected_sequence;
    result["stop_reason"] = state.stop_reason;
    result["ros_time_ns"] = node_->now().nanoseconds();
    return result;
  }

  bool reset() { return sender_->reset(); }

private:
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rolling::RollingSender> sender_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};

PYBIND11_MODULE(_rolling_sender, module)
{
  py::class_<Sender>(module, "Sender")
    .def(py::init<int>())
    .def("send", &Sender::send)
    .def("status", &Sender::status)
    .def("reset", &Sender::reset)
    .def("close", &Sender::close, py::call_guard<py::gil_scoped_release>());
}
