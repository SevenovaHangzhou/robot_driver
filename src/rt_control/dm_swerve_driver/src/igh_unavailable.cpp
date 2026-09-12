#include "dm_swerve_driver/igh_ethercat_bus.hpp"
#include <stdexcept>

namespace dm_swerve_driver {
bool igh_backend_available() noexcept {return false;}
std::unique_ptr<KincoEthercatBus> make_igh_ethercat_bus(const KincoParameters &)
{
  throw std::runtime_error{
          "IgH backend was not built: install ecrt.h/libethercat and build with "
          "-DDM_SWERVE_ENABLE_IGH=ON"};
}
}  // namespace dm_swerve_driver
