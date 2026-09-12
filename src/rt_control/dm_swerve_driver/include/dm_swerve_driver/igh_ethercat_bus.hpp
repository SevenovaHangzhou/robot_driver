#ifndef DM_SWERVE_DRIVER__IGH_ETHERCAT_BUS_HPP_
#define DM_SWERVE_DRIVER__IGH_ETHERCAT_BUS_HPP_

#include <memory>
#include "dm_swerve_driver/kinco_params.hpp"

namespace dm_swerve_driver {

[[nodiscard]] bool igh_backend_available() noexcept;
[[nodiscard]] std::unique_ptr<KincoEthercatBus> make_igh_ethercat_bus(
  const KincoParameters & parameters);

}  // namespace dm_swerve_driver
#endif
