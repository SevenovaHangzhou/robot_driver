#ifndef DM_SWERVE_DRIVER__SRC__NODE_BACKEND_HPP_
#define DM_SWERVE_DRIVER__SRC__NODE_BACKEND_HPP_
#include "dm_swerve_driver/swerve_driver_node.hpp"
#include "dm_swerve_driver/kinco_params.hpp"
namespace dm_swerve_driver {
std::string configure_node_backend(rclcpp_lifecycle::LifecycleNode & node,
  DriverParameters & common, KincoParameters & parameters);
std::unique_ptr<ControlRunner> make_node_control_runner(const std::string & backend,
  const DriverParameters & common, const KincoParameters & parameters,
  const TransportFactory & factory, ControlLoopCallbacks callbacks);
}
#endif
