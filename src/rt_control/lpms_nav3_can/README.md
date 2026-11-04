# lpms_nav3_can

ROS 2 Humble sensor adapter for the specified LPMS-NAV3 six-axis 16-bit CANopen TPDO mapping.
Migrated for ELECTRI-105 into robot_driver; see [NOTICE.md](NOTICE.md) for provenance.
It is independent of controller_manager and never writes CAN frames, NMT, SDO,
bitrate, interface configuration or TF. Reconnect only reopens the receive socket.

## Integration Status

Source builds in native/CI and is an installed bringup dependency, but is not
started by production launch. The standalone node and launch default to `can0`,
Node ID 1 and `imu_link` for the connected USB-CAN adapter. `can1` remains reserved.

The historical 2026-08-28 independent CANable bench used 500 kbit/s, Node 1 and
four TPDOs at approximately 100 Hz; it does not qualify Gen3 shared CAN. The shared
bus involving ELECTRI-105/120/122 still needs arbitration IDs, bitrate, ownership,
load and fault behavior verified before any production launch is enabled.

## Receive Mapping

All IDs below are standard classic CAN data frames, offset by configured Node ID.

| Base ID | Payload |
| --- | --- |
| 0x180 | calibrated acceleration XYZ (0.001 g/LSB), axis-calibrated GyroI X (0.1 deg/s/LSB) |
| 0x280 | axis-calibrated GyroI YZ, bias-calibrated GyroI XY (0.1 deg/s/LSB) |
| 0x380 | bias-calibrated GyroI Z (0.1 deg/s/LSB), Euler XYZ (0.01 deg/LSB) |
| 0x480 | quaternion WXYZ (0.0001/LSB) |
| 0x700 | heartbeat, including the observed zero-padded DLC=8 form |

The required channel mapping indices are
`[4, 5, 6, 19, 20, 21, 13, 14, 15, 38, 39, 40, 34, 35, 36, 37]`.
This is a site-provided mapping, not a configuration readback from the device.
`sensor_msgs/msg/Imu.angular_velocity` uses the axis-calibrated GyroI values;
the bias-calibrated GyroI values are decoded but have no ROS topic. No
magnetometer is present in this mapping.

Conversion is g to m/s² and deg/s to rad/s. A sample requires all
four TPDOs; old partial assemblies are discarded on the next incoming PDO after
`assembly_timeout_ms`. No device sample counter is available, so assembly is
host-arrival based and is not proof that the four frames share a device timestamp.
ROS timestamp is publication time, not hardware or kernel CAN capture time.

## Standalone Topics

| Topic | Type | QoS |
| --- | --- | --- |
| `/imu/data` | sensor_msgs/msg/Imu | robot_interfaces_qos::fast_state (reliable) |
| `~/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | diagnostic (reliable) |
| `/rt_control/sensors/status` | robot_rt_control_interfaces/msg/SensorStatusArray | state (reliable) |

These standalone topic names are not newly frozen cross-domain contracts.
Unlike the staging version's SensorDataQoS, publication uses the repository's
named profiles. Production fusion requires a separately approved public contract.

`convert_to_ros_convention=false` keeps the LPMS convention before mount rotation. The optional
legacy conversion negates acceleration and conjugates the quaternion; it does not
replace a measured mounting transform or dynamic axis-sign validation. This machine's
sensor X points robot left, so `mount_yaw_deg=90.0` rotates sensor vectors into a
robot-aligned `imu_link`: output X = -sensor Y, output Y = sensor X, and Z is unchanged.
The driver also adjusts the quaternion and all covariance matrices for that rotation.
The `imu_link` frame now describes the aligned output axes; a later TF/URDF must not
apply this same 90-degree rotation again. Confirm signs with forward and lateral
motion before using the IMU for fusion. `mount_yaw_deg=0.0` disables this mounting
correction for a differently installed sensor. Three
row-major 3x3 covariance parameters are supported: orientation in rad², angular
velocity in (rad/s)², and linear acceleration in (m/s²)². Each must contain nine
finite values with nonnegative diagonal entries. In
[config/lpms_nav3_can.yaml](config/lpms_nav3_can.yaml), only
`angular_velocity_covariance` is active; the other two entries are commented out.
The node defaults to zero (unknown) without a configured matrix. The bundled
angular-velocity matrix is a provisional estimate from static X/Y noise and a
steady-turn Z interval, without an independent angular-rate reference; it is not
a completed dynamic calibration. The other two matrices remain unknown. Configure
only the covariances used by the fusion system. Quaternion validity and
extrinsics must be verified before navigation fusion.

## Build And Run

```bash
source /opt/ros/humble/setup.bash
source /home/sev_v3/Robot/rt-control/main/install/setup.bash
cd /home/sev_v3/Robot/rt-control/main/robot_driver
colcon build --packages-up-to lpms_nav3_can --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select lpms_nav3_can
colcon test-result --verbose
ros2 launch lpms_nav3_can lpms_nav3_can.launch.py
```

The last command starts the receive-only node on an already enabled 500 kbit/s
`can0`. Direct node execution uses the same defaults:

```bash
ros2 run lpms_nav3_can lpms_nav3_can_node
```

The package does not set up the bus. Both `ros2 launch` and direct `ros2 run`
publish `/imu/data`, `~/diagnostics`, and `/rt_control/sensors/status`. For a
config-only launch check, pass
`validation_only:=true`. A different standalone setup can use a site YAML based on
[config/lpms_nav3_can.yaml](config/lpms_nav3_can.yaml) via `config_file:=<site-yaml>`.
The `imu_link` label does not establish a physical mounting translation or production
admission. The package contains no systemd/udev or interface-up scripts.
On missing interface/data the node reports diagnostics, never republishes stale
or fabricated samples, and exits cleanly on SIGINT/SIGTERM.

On 2026-10-19, the standalone launch connected to the UP, 500 kbit/s USB-CAN
`can0`, and the then-current `/imu/lpms_nav3_can_node/data` topic delivered a
live IMU message with `frame_id: imu_link` and approximately 9.84 m/s² Z
acceleration. The device's
configured PDO mapping indices were not read back.

## CSV Export

From the repository root, the standalone [CSV tool](../../../tools/ros_topic_to_csv.py)
can subscribe to one live topic or read one topic from a rosbag2 directory:

```bash
source /opt/ros/humble/setup.bash
python3 tools/ros_topic_to_csv.py \
  --topic /imu/data --output ~/imu_live.csv
python3 tools/ros_topic_to_csv.py \
  --bag ~/imu_bag_YYYYMMDD_HHMMSS \
  --topic /imu/lpms_nav3_can_node/data --output ~/imu_from_old_bag.csv
```

Use `--topic /lpms_nav3_can_node/diagnostics` for the current diagnostic topic. The bag example
above uses the old topic name because bags recorded before this rename retain
their original names. For new bags, use `--topic /imu/data`.
The tool accepts a `.db3` path when `metadata.yaml` is in the same directory.
It exports nested scalar fields as columns; fixed-length arrays such as IMU
covariances become indexed columns, and variable-length arrays become JSON in
one CSV cell. `capture_timestamp_ns` is the rosbag recording timestamp in bag
mode and the local receive timestamp in live mode. Existing CSV files are never
overwritten. Stop live capture with Ctrl+C; for complete archival capture, keep
the original rosbag because a live subscription can miss samples.

Tests use decoded synthetic frames and a deliberately nonexistent interface.
No live or virtual CAN creation is performed automatically. `ENABLE_COVERAGE=ON`
instruments library/node targets for separate coverage analysis.
Diagnostics are verified using same-process intra-process delivery.
