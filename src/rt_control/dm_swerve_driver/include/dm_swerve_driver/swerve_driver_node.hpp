#ifndef DM_SWERVE_DRIVER__SWERVE_DRIVER_NODE_HPP_
#define DM_SWERVE_DRIVER__SWERVE_DRIVER_NODE_HPP_

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "dm_swerve_driver/can_transport.hpp"
#include "dm_swerve_driver/control_loop.hpp"
#include "dm_swerve_driver/params.hpp"

namespace dm_swerve_driver {

using TransportFactory = std::function<std::unique_ptr<CanTransport>(
    const DriverParameters &)>;

class SwerveDriverNode final : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit SwerveDriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{},
    TransportFactory transport_factory = {});
  ~SwerveDriverNode() noexcept override;

  [[nodiscard]] ControlLoopStatus control_status() const;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__SWERVE_DRIVER_NODE_HPP_
