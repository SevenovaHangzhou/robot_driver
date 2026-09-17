# lpms_nav3_can

ROS 2 Humble sensor adapter for the LPMS-NAV3 default 16-bit CANopen TPDO mapping.
Migrated for ELECTRI-105 into robot_driver; see [NOTICE.md](NOTICE.md) for provenance.
It is independent of controller_manager and never writes CAN frames, NMT, SDO,
bitrate, interface configuration or TF. Reconnect only reopens the receive socket.

## Integration Status

Source builds in native/CI and is an installed bringup dependency, but is not
started by production launch. The standalone launch defaults to static validation.
No physical interface, Node ID or frame is selected by default. `can0/can1` are
rejected without an override. An interface name alone is not a hardware identity
or proof that a bus is dedicated: the commissioning operator must verify it.

The historical 2026-08-28 independent CANable bench used 500 kbit/s, Node 1 and
four TPDOs at approximately 100 Hz; it does not qualify Gen3 shared CAN. The shared
bus involving ELECTRI-105/120/122 still needs arbitration IDs, bitrate, ownership,
load and fault behavior verified before any production launch is enabled.

## Receive Mapping

All IDs below are standard classic CAN data frames, offset by configured Node ID.

| Base ID | Payload |
| --- | --- |
| 0x180 | acceleration XYZ (0.001 g/LSB), gyro X (0.1 deg/s/LSB) |
| 0x280 | gyro YZ, magnetic field XY (0.01 uT/LSB) |
| 0x380 | magnetic field Z, Euler XYZ (0.01 deg/LSB) |
| 0x480 | quaternion WXYZ (0.0001/LSB) |
| 0x700 | heartbeat, including the observed zero-padded DLC=8 form |

Conversion is g to m/s², deg/s to rad/s, and uT to tesla. A sample requires all
four TPDOs; old partial assemblies are discarded on the next incoming PDO after
`assembly_timeout_ms`. No device sample counter is available, so assembly is
host-arrival based and is not proof that the four frames share a device timestamp.
ROS timestamp is publication time, not hardware or kernel CAN capture time.

## Private Topics

| Topic | Type | QoS |
| --- | --- | --- |
| ~/data | sensor_msgs/msg/Imu | robot_interfaces_qos::fast_state (reliable) |
| ~/mag | sensor_msgs/msg/MagneticField | fast_state (reliable) |
| ~/diagnostics | diagnostic_msgs/msg/DiagnosticArray | diagnostic (reliable) |

These are commissioning/private endpoints, not newly frozen cross-domain contracts.
Unlike the staging version's SensorDataQoS, publication uses the repository's
named profiles. Explicit bench remaps can reproduce `/imu/data` and `/imu/mag`;
production remapping/fusion requires a separately approved public contract.

`convert_to_ros_convention=false` keeps LPMS axes after SI conversion. The optional
legacy conversion negates acceleration and conjugates the quaternion; it does not
replace a measured mounting transform or dynamic axis-sign validation. Covariance
arrays remain zero (unknown), not calibrated. Quaternion validity, magnetic
calibration and extrinsics must be verified before navigation fusion.

## Build And Static Validation

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to lpms_nav3_can --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select lpms_nav3_can
colcon test-result --verbose
ros2 launch lpms_nav3_can lpms_nav3_can.launch.py
```

The last command starts no node. For authorized standalone commissioning, provide
a site YAML following [config/lpms_nav3_can.yaml](config/lpms_nav3_can.yaml) with
explicit `can_interface`, `node_id` (1..127), `frame_id` and validated timeouts, then
set `config_file:=<site-yaml> validation_only:=false`. Bus setup is external and
must be authorized; this package contains no systemd/udev or interface-up scripts.
On missing interface/data the node reports diagnostics, never republishes stale
or fabricated samples, and exits cleanly on SIGINT/SIGTERM.

Tests use decoded synthetic frames and a deliberately nonexistent interface.
No live or virtual CAN creation is performed automatically. `ENABLE_COVERAGE=ON`
instruments library/node targets for separate coverage analysis.
Diagnostics are verified using same-process intra-process delivery; cross-process
DDS discovery did not pass in the current local environment and remains unverified.
