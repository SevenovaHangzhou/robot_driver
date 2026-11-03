#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ctrl_gravity_feedforward/gravity_model.hpp"

namespace ctrl_gravity_feedforward
{

class GravityController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool feedback_ready() const noexcept;
  void zero_commands() noexcept;

  std::unique_ptr<GravityModel> model_;
  std::vector<std::string> command_names_;
  std::vector<std::string> state_names_;
  std::vector<hardware_interface::LoanedCommandInterface *> outputs_;
  std::vector<hardware_interface::LoanedStateInterface *> inputs_;
  std::vector<double> positions_;
  std::vector<double> limits_;
  std::vector<double> rates_;
  std::vector<double> last_effort_;
  std::vector<double> health_values_;
  double max_feedback_age_ms_{0.0};
  bool active_{false};
  bool faulted_{false};
};

}  // namespace ctrl_gravity_feedforward
