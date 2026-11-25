# Source Provenance

The kinematics, setpoint generator, odometry, covariance routines and their
original tests were migrated from this project's dm_swerve_driver package at
commit 7bd20cf21299c6b590e226550181037981d54ede on feature/damiao steering.
The original Apache-2.0 license is retained. The namespace/include prefix was
changed to swerve_driver (now ctrl_swerve); protocol-specific motor/control code is not imported.

Source: https://github.com/SevenovaHangzhou/robot_driver/tree/7bd20cf21299c6b590e226550181037981d54ede/src/rt_control/dm_swerve_driver

OdometryParameters retains only protocol-neutral covariance configuration.
Production controller parameters must be provided explicitly; the math tests
use synthetic values and do not certify hardware or mechanical calibration.

The 2026-11-25 FollowMPPI compatibility correction (#339) ports the prepare/finish
motion envelope from the supplied `driver_handoff_20261124` production snapshot.
Reference `include/swerve_driver/motion_shaping.hpp` SHA256:
`152af2cb71e803bf237d0388c5f49dc66d6c35681b23eccbe56996e2860e485a`.
The reference source lock and Apache-2.0 license remain in the external handoff;
the optional replay tests compile that immutable reference, rather than vendoring
another navigation source tree. Published Navigation baseline:
`3f09209babca43a716d842f58fe30a5e1fbe48a2`; simulation baseline:
`e48feba61073c02bec666f052e55f1fc547257cf`.
