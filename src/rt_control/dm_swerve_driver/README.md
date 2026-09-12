# dm_swerve_driver

ROS 2 Humble lifecycle driver for a four-module swerve chassis. The package name is
retained for downstream compatibility; the only motor backend is eight Kinco FD axes
over EtherCAT. Four external absolute steering encoders use CANopen over an independent
SocketCAN interface.

## Architecture

- Four steering axes use EtherCAT CSP; four drive axes use EtherCAT CSV.
- IgH exchanges all eight axes in one cyclic PDO operation.
- BRT absolute encoders are sampled by CANopen SYNC and remain the primary steering source.
- Motor encoders are aligned backups for a temporary external-encoder outage.
- Steering targets and their complete paths stay inside the configured mechanical interval.
- Measured wheel velocity is fitted with residual-based slip rejection for odometry.

The generic `CanFrame`, `CanTransport`, and SocketCAN implementation exist only for
the external CANopen encoders. No CAN-based motor codec, motor-register access, or
motor enable/clear/zero command is present.

## Build And Test

The default build uses a fail-closed EtherCAT stub so unit tests do not need IgH:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dm_swerve_driver
colcon test --packages-select dm_swerve_driver
colcon test-result --verbose
```

For a hardware build, install the IgH 1.6 headers and library that match the target
kernel master:

```bash
colcon build --packages-select dm_swerve_driver \
  --cmake-args -DDM_SWERVE_ENABLE_IGH=ON
```

## Configure And Run

`config/kinco_params.yaml` deliberately contains zero-valued hardware facts. Fill in
the measured geometry, gearing, encoder resolutions, EtherCAT slave order, PDO watchdog,
CANopen encoder identity and calibration paths before activation.

```bash
source install/setup.bash
ros2 launch dm_swerve_driver swerve_driver.launch.py \
  params_file:=/absolute/path/to/kinco_params.yaml
```

The lifecycle node configures and activates automatically. Activation fails unless all
eight EtherCAT axes, all four external encoders, persisted steering snapshots, source
agreement and mechanical-limit checks pass.

## Steering And Safety

Steering joints are bounded revolute joints, normally near -pi to +pi, not continuous
joints. Forward and reverse wheel branches are compared using actual travel inside the
safe interval. Neither the planner nor the EtherCAT output layer wraps across a mechanical
endpoint or silently clamps an unsafe target.

Any EtherCAT domain failure, offline axis, invalid steering source or serious drive fault
sets all drive targets to zero. Recoverable faults have a bounded retry count; latching
faults require `~/clear_faults` and fresh confirmation from every axis. Command timeout
and IMU fallback remain non-latching.

Services:

```bash
ros2 service call /swerve_driver/disable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/enable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/clear_faults std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/rezero_steering std_srvs/srv/Trigger {}
```

The rezero service is accepted only while disabled. It records software installation
offsets and the motor reference without writing a hardware zero to the external encoder.

## Hardware Acceptance

Software tests do not replace EtherCAT/CANopen and mechanical validation. Use:

- [Kinco commissioning](doc/kinco_bringup.md)
- [Calibration overview](doc/swerve_calibration_overview.md)
- [Field calibration manual](doc/swerve_calibration_manual.md)
- [Calibration record](doc/swerve_calibration_record_template.md)
- [Hardware validation status](doc/kinco_hardware_validation_status.md)

Do not sign off the vehicle until PDO watchdog, physical EtherCAT disconnect, axis faults,
encoder disconnect/rejoin, mechanical endpoints, +179/-179-degree paths and loaded motion
have been measured on hardware.
