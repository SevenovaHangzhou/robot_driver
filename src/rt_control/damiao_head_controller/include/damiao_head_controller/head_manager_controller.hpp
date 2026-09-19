#ifndef DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CONTROLLER_HPP_
#define DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "damiao_head_controller/head_manager_core.hpp"
#include "rt_control_interfaces/srv/rt_enable.hpp"

namespace damiao_head_controller
{

class HeadManagerController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using EnableService = rt_control_interfaces::srv::RtEnable;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  enum class Operation : std::uint8_t {none, enable, disable, reset};

  [[nodiscard]] HeadSnapshot snapshot() const noexcept;
  [[nodiscard]] bool bind_interfaces();
  [[nodiscard]] bool switch_position_controller(bool activate);
  [[nodiscard]] bool wait_for_result(
    Operation operation, std::uint64_t reset_generation = 0U) const;
  void handle_enable(
    const std::shared_ptr<EnableService::Request> request,
    std::shared_ptr<EnableService::Response> response);
  void handle_disable(
    const std::shared_ptr<EnableService::Request> request,
    std::shared_ptr<EnableService::Response> response);
  void handle_reset(
    const std::shared_ptr<EnableService::Request> request,
    std::shared_ptr<EnableService::Response> response);
  void fill_response(
    EnableService::Response & response, bool ok, const std::string & stage) const;
  void handle_fault_stop();
  void publish_diagnostics();
  void release_handles() noexcept;
  static std::uint64_t encode_double(double value) noexcept;
  static double decode_double(std::uint64_t bits) noexcept;

  std::vector<std::string> joint_names_;
  std::string control_name_;
  std::string position_controller_name_;
  std::string service_prefix_{"/rt/head"};
  std::vector<std::string> command_names_;
  std::vector<std::string> state_names_;
  std::array<hardware_interface::LoanedCommandInterface *, 2U> commands_{};
  std::array<hardware_interface::LoanedStateInterface *, 20U> states_{};

  std::atomic_bool active_{false};
  std::atomic_bool requested_enable_{false};
  std::atomic<std::uint64_t> requested_reset_generation_{0U};
  std::atomic<std::uint8_t> phase_{static_cast<std::uint8_t>(HeadPhase::disabled)};
  std::atomic_bool fault_latched_{false};
  std::atomic_bool motion_allowed_{false};
  std::atomic<std::uint64_t> handled_reset_generation_{0U};
  std::atomic<Operation> operation_{Operation::none};
  std::atomic_bool fault_stop_requested_{false};
  std::atomic_bool position_controller_active_{false};
  std::array<std::atomic<std::uint64_t>, 8U> diagnostic_value_bits_{};

  std::chrono::milliseconds service_timeout_{5000};
  std::chrono::milliseconds controller_switch_timeout_{1000};
  double diagnostic_feedback_timeout_ms_{100.0};

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::Service<EnableService>::SharedPtr enable_service_;
  rclcpp::Service<EnableService>::SharedPtr disable_service_;
  rclcpp::Service<EnableService>::SharedPtr reset_service_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace damiao_head_controller

#endif  // DAMIAO_HEAD_CONTROLLER__HEAD_MANAGER_CONTROLLER_HPP_
