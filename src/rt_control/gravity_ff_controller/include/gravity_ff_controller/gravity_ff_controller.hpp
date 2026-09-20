#ifndef GRAVITY_FF_CONTROLLER__GRAVITY_FF_CONTROLLER_HPP_
#define GRAVITY_FF_CONTROLLER__GRAVITY_FF_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gravity_ff_controller/control_core.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rt_arm_dynamics/arm_dynamics.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace gravity_ff_controller
{
class GravityFeedforwardController final
  : public controller_interface::ControllerInterface
{
public:
  static_assert(
    std::atomic<double>::is_always_lock_free,
    "realtime scale requires lock-free double atomics");
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  static constexpr std::size_t kMaximumAxes = 7U;
  bool configure_model();
  bool configure_policy();
  bool bind_interfaces();
  void clear_resources();
  void publish_diagnostics(const rclcpp::Time &, StepStatus);
  rcl_interfaces::msg::SetParametersResult
  validate_parameters(const std::vector<rclcpp::Parameter> & parameters);

  rt_arm_dynamics::ArmDynamics dynamics_;
  ControlCore core_;
  std::vector<std::string> joints_, state_names_, command_names_;
  std::vector<hardware_interface::LoanedStateInterface *> position_, status_,
    torque_raw_;
  std::vector<hardware_interface::LoanedCommandInterface *> effort_;
  Eigen::VectorXd q_, gravity_nm_;
  std::vector<double> gravity_values_, status_values_, scale_values_,
    torque_raw_values_;
  std::array<std::atomic<double>, kMaximumAxes> scale_{};
  bool active_mode_{false}, torque_state_enabled_{true}, model_verified_{false},
  calibration_verified_{false}, shadow_error_{false};
  double publish_period_seconds_{0.1}, publish_elapsed_seconds_{0.0};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    publisher_;
  std::unique_ptr<
    realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
  realtime_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;
};
} // namespace gravity_ff_controller
#endif // GRAVITY_FF_CONTROLLER__GRAVITY_FF_CONTROLLER_HPP_
