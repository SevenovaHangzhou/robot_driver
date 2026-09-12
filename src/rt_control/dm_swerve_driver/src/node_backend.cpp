#include "node_backend.hpp"
#include "dm_swerve_driver/ros_params.hpp"
#include "dm_swerve_driver/igh_ethercat_bus.hpp"
#include "dm_swerve_driver/kinco_control_loop.hpp"
#include "dm_swerve_driver/kinco_calibration.hpp"
#include <stdexcept>
#include <utility>

namespace dm_swerve_driver {
std::string configure_node_backend(rclcpp_lifecycle::LifecycleNode & node,
  DriverParameters & common, KincoParameters & parameters)
{
  if (!node.has_parameter("driver.backend")) {node.declare_parameter<std::string>("driver.backend", "damiao");}
  const auto backend = node.get_parameter("driver.backend").as_string();
  if (backend != "damiao" && backend != "kinco") {
    throw std::invalid_argument{"driver.backend must be damiao or kinco"};
  }
  if (backend == "kinco") {
    declare_kinco_parameters(node);
    parameters = load_kinco_parameters(node);
    load_kinco_calibration(common, parameters);
  }
  return backend;
}

std::unique_ptr<ControlRunner> make_node_control_runner(const std::string & backend,
  const DriverParameters & common, const KincoParameters & parameters,
  const TransportFactory & factory, ControlLoopCallbacks callbacks)
{
  if (backend == "kinco") {
    return std::make_unique<KincoControlLoop>(common, parameters,
      make_igh_ethercat_bus(parameters), make_encoder_transport(parameters), std::move(callbacks));
  }
  return std::make_unique<ControlLoop>(common, factory(common), std::move(callbacks));
}
}
