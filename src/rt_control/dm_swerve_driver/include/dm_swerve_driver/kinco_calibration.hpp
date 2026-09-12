#ifndef DM_SWERVE_DRIVER__KINCO_CALIBRATION_HPP_
#define DM_SWERVE_DRIVER__KINCO_CALIBRATION_HPP_
#include "dm_swerve_driver/kinco_params.hpp"
namespace dm_swerve_driver {
void load_kinco_calibration(DriverParameters & common, KincoParameters & parameters);
void calibrate_kinco_steering(DriverParameters & common, KincoParameters & parameters);
}
#endif
