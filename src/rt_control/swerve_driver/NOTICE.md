# Source Provenance

The kinematics, setpoint generator, odometry, covariance routines and their
original tests were migrated from this project's dm_swerve_driver package at
commit 7bd20cf21299c6b590e226550181037981d54ede on feature/damiao steering.
The original Apache-2.0 license is retained. The namespace/include prefix was
changed to swerve_driver; protocol-specific motor/control code is not imported.

Source: https://github.com/SevenovaHangzhou/robot_driver/tree/7bd20cf21299c6b590e226550181037981d54ede/src/rt_control/dm_swerve_driver

OdometryParameters retains only protocol-neutral covariance configuration.
Production controller parameters must be provided explicitly; the math tests
use synthetic values and do not certify hardware or mechanical calibration.
