# dm_swerve_driver

ROS 2 Humble lifecycle driver for a four-module DaMiao swerve chassis using MIT mode over one SocketCAN bus.

Implemented features:

- DaMiao MIT, feedback, register and special-command codec
- RAII SocketCAN interface with filtered receive, `sendmmsg` batch writes and bounded collect
- 100 Hz absolute-time control thread; executor callbacks only update timestamped mailboxes
- command discretization, stateful flip hysteresis, steering slew, unified desaturation and alignment gating
- position-increment odometry, measured twist, gyro/wheel yaw fallback and quality-scaled covariance
- strict PMAX/VMAX/TMAX, feedback, p_m and enable startup gates
- ks/kv/ka drive feedforward and bounded steering angular-velocity feedforward
- single-motor stop, classified recovery limits and manually cleared fault latching
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

All geometry, gearing, limits and feedforward values in
[swerve_params.yaml](config/swerve_params.yaml) are placeholders until calibrated. This is an
intentional bring-up prerequisite, not a claim that the checked-in values fit a real chassis.

```bash
source install/setup.bash
ros2 launch dm_swerve_driver swerve_driver.launch.py \
  params_file:=/absolute/path/to/swerve_params.yaml
```

The launch file configures and activates the lifecycle node automatically. Activation succeeds
only after all eight motors pass limit-register readback, initial feedback, steering absolute-angle
initialization and enable confirmation. For steering gear ratio greater than one, missing or
inconsistent `p_m` also rejects activation and the log directs the operator to rezero.

`can.allow_fallback_limits` defaults to `false`. It can only be enabled with a `vcan*` or `fake*`
interface for tests; production CAN cannot bypass limit readback. `can.write_timeout_register` defaults
to `true`, but the assumed 50 us/count conversion and persistence semantics still require the
single-motor hardware check described below.

Control services:

```bash
ros2 service call /swerve_driver/disable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/enable std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/clear_faults std_srvs/srv/Trigger {}
ros2 service call /swerve_driver/rezero_steering std_srvs/srv/Trigger {}
```

`rezero_steering` is accepted only while control is disabled. It succeeds only after every steering
motor acknowledges `save_zero` by MST_ID and an independent feedback poll verifies position within
`steering.rezero_tolerance_rad`.

## Safety behavior

- A motor reaching `safety.feedback_silent_cycles`, becoming disabled, or reporting an ERR gates
  all four drive commands to zero while steering holds its current measured angle.
- A CAN write/collect exception also enters a transport-faulted zero-speed window; it clears only
  after one complete, freshly timestamped feedback set is received.
- Under-voltage and communication-lost errors are cleared and re-enabled no faster than once per
  second. Each motor gets `safety.auto_recovery_limit` attempts (default 3); the count persists
  across automatic recovery and exceeding it latches the fault.
- Encoder/read-encoder, over-voltage, over-current, MOS/coil over-temperature, overload and unknown
  hardware errors latch immediately.
- A latched fault keeps all drive commands at zero. Only `~/clear_faults` can unlock it, and only
  after fresh enable acknowledgements from all eight motors; a partial clear remains latched.
  `~/disable` followed by `~/enable` preserves both the latch and recovery counters.
- `/cmd_vel` timeout and IMU loss remain non-latching. A fresh command recovers the watchdog;
  timeout hold mode keeps the measured steering angle, while non-hold mode slews toward zero.
  IMU loss switches yaw to wheel odometry and inflates the configured odometry covariance.
- The 20-degree steering alignment gate remains an intentional, tunable control-quality mechanism;
  navigation-layer continuity is handled by navigation tuning rather than bypassing the gate.

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

Do not mark hardware acceptance complete until the 现场记录 template is populated and signed. In
particular, software tests cannot prove physical cable-pull stopping; the firmware TIMEOUT and
physical disconnect test close that requirement.
