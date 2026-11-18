# periph_imu_lpms

ROS 2 Humble sensor adapter for the LPMS-NAV3 default 16-bit CANopen TPDO mapping.
Migrated for ELECTRI-105 into robot_driver; see [NOTICE.md](NOTICE.md) for provenance.
It is independent of controller_manager and never writes CAN frames, NMT, SDO,
bitrate, interface configuration or TF. Reconnect only reopens the receive socket.

## Integration Status

The `lpms_nav3_decoder` and `lpms_nav3_socket_can` libraries expose headers at
`periph_imu_lpms/protocol/decoder.hpp` and `periph_imu_lpms/protocol/socket_can.hpp`.
These headers have no ROS dependencies. ROS message conversion remains in
`periph_imu_lpms/ros_conversion.hpp` and the `lpms_nav3_ros_conversion` target.

Source builds in native/CI and is an installed bringup dependency, but is not
started by production launch. The standalone launch defaults to static validation.
No physical interface, Node ID or frame is selected by default; which interface the IMU
uses is a per-host deployment fact, not a rule in this node. An interface name alone is not a hardware identity
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
| 0x280 | gyro YZ, bias-calibrated gyro XY (0.1 deg/s/LSB) |
| 0x380 | bias-calibrated gyro Z, Euler XYZ (0.01 deg/LSB) |
| 0x480 | quaternion WXYZ (0.0001/LSB) |
| 0x700 | heartbeat, including the observed zero-padded DLC=8 form |

Channel layout of the installed sensor (sensor team host-tool readback 2026-10-19, kept on
`feature/electri-127-chassis-imu`): channels 1..3 calibrated acceleration (gravity
retained, about +1 g on Z at rest), 4..6 axis-alignment-calibrated gyro, 7..9
static-bias-calibrated gyro, 10..12 Euler, 13..16 quaternion WXYZ. There is no
magnetometer channel. The Advantech raw CAN log of 2026-10-20 agrees: channels 7..9 track
4..6 (correlation 0.95/0.97/1.00 during full turns, equal integrated yaw). The NAV3-CAN
port only transmits data; mapping and filter-mode readback needs the TTL port.

Conversion is g to m/s² and deg/s to rad/s. A sample requires all four TPDOs; old
partial assemblies are discarded on the next incoming PDO after `assembly_timeout_ms`.
No device sample counter or device time is available, so assembly is host-arrival based.
The header stamp is the kernel SocketCAN receive time (`SO_TIMESTAMPNS`, CLOCK_REALTIME)
of the sample's first TPDO and `/rt/sensors/state` reports `TIME_SOURCE_HOST_RECEIVE`.
A frame without a kernel stamp falls back to publication time, reported as
`TIME_SOURCE_PUBLICATION` and counted in the `publication_stamped_samples` diagnostic.

## Topics

| Topic | Type | QoS |
| --- | --- | --- |
| `/rt/imu/data` | sensor_msgs/msg/Imu | robot_interfaces_qos::fast_state (reliable) |
| `~/gyro_bias_corrected` | sensor_msgs/msg/Imu, angular velocity only | fast_state (reliable), RT-local commissioning only |
| `~/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | diagnostic (reliable), engineering diagnostics |
| `/rt/sensors/state` | robot_rt_control_interfaces/msg/SensorStatusArray | state (reliable), cross-domain validity |

The IMU observation and structured validity status are public contract endpoints.
The bias-calibrated gyro remains an RT-local commissioning endpoint. Publication uses
repository named QoS profiles.

`convert_to_ros_convention=false` keeps LPMS axes after SI conversion. The optional
legacy conversion negates acceleration and conjugates the quaternion; it does not
replace a measured mounting transform or dynamic axis-sign validation. Covariance
arrays default to zero (unknown), not calibrated. `orientation_covariance` and
`angular_velocity_covariance` are startup-only parameters containing nine finite
numbers in row-major 3x3 order, in rad² and (rad/s)² respectively. Set them in the
YAML using manufacturer or calibration evidence, then restart the node. Invalid
lengths or non-finite values reject startup. Acceleration covariance remains zero
(unknown). No mounting rotation is applied: `frame_id` names the sensor's own axes and
the mounting extrinsic belongs to the consumer's calibration, not to this driver.
Quaternion validity must be verified before fusing orientation.

## Build And Static Validation

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to periph_imu_lpms --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select periph_imu_lpms
colcon test-result --verbose
ros2 launch periph_imu_lpms lpms_nav3_can.launch.py
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
