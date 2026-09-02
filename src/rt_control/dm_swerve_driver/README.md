# dm_swerve_driver

ROS 2 Humble lifecycle driver for a four-module DaMiao swerve chassis using MIT mode over one SocketCAN bus.

Implemented features:

- DaMiao MIT, feedback, register and special-command codec
- RAII SocketCAN interface with filtered receive, `sendmmsg` batch writes and bounded collect
- 100 Hz absolute-time control thread; executor callbacks only update timestamped mailboxes
- command discretization, swerve IK/optimization/desaturation/alignment gating
- position-increment odometry with gyro yaw and wheel-derived yaw fallback
- PMAX/VMAX/TMAX startup readback with configured fallback, p_m steering seed and enable retry
- ks/kv/ka drive feedforward and bounded steering angular-velocity feedforward
- non-latching command, IMU, motor-feedback and whole-bus degradation with automatic recovery
- odometry, TF, joint states, diagnostics and enable/disable/clear/rezero services
- fake 8-motor bus tests, vcan integration hook and read-only hardware audit tool

The codec intentionally truncates float-to-integer mappings after clamping, matching the DaMiao reference algorithm rather than rounding to the nearest integer.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths /home/kkozia/damiao/dm_swerve_driver
colcon test --base-paths /home/kkozia/damiao/dm_swerve_driver
colcon test-result --verbose
```

For an in-package build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The real vcan I/O test skips if `vcan0` or kernel vcan support is absent. On a capable host:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
ctest --test-dir build -R test_socketcan_vcan --output-on-failure
```

## Run

All values in [swerve_params.yaml](config/swerve_params.yaml) are placeholders until calibrated.

```bash
source install/setup.bash
ros2 launch dm_swerve_driver swerve_driver.launch.py \
  params_file:=/absolute/path/to/swerve_params.yaml
```

The launch file configures and activates the lifecycle node automatically. The only hard-stop paths are explicit disable, process shutdown and SIGINT:

```bash
ros2 service call /swerve_driver/disable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/enable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/clear_faults std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/rezero_steering std_srvs/srv/Trigger {}
```

`rezero_steering` is accepted only while control is disabled.

## ROS interfaces

| Kind | Name | Type |
|---|---|---|
| Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Subscribe | configured IMU topic | `sensor_msgs/msg/Imu` |
| Publish | `/swerve_driver/odom` | `nav_msgs/msg/Odometry` |
| Publish | `/joint_states` | `sensor_msgs/msg/JointState` |
| Publish | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |
| Publish | `/tf` (optional) | odom → base transform |
| Service | `~/enable`, `~/disable`, `~/clear_faults`, `~/rezero_steering` | `std_srvs/srv/Trigger` |

## Simulation and audit tools

Run the SocketCAN fake motor process with optional delay and loss injection:

```bash
ros2 run dm_swerve_driver fake_motor_sim --interface vcan0
ros2 run dm_swerve_driver fake_motor_sim --help
```

Audit real motors without enabling them:

```bash
ros2 run dm_swerve_driver dm_swerve_bringup_check --help
ros2 run dm_swerve_driver dm_swerve_bringup_check \
  --interface can0 --steering 1:17 --motor 5:21
```

## Hardware completion boundary

The software implementation and fake-bus verification are complete. Real motor and vehicle validation cannot be performed without the physical CAN adapter, motors and chassis. Follow:

- [Calibration procedure](doc/calibration.md)
- [Hardware bring-up order](doc/hardware_bringup.md)
- [Current hardware validation status](doc/hardware_validation_status.md)

Do not mark Phase 6 hardware acceptance complete until the现场记录 template is populated and signed.
