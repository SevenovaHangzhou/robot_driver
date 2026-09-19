# robot_hw_can

This ROS 2 Humble `hardware_interface::SystemInterface` controls two DaMiao
DM-J4310-2EC motors over native classic CAN / Linux SocketCAN. It does not
depend on CANopen, CiA402, Lely, or `robot_hw_canopen`. The configured Linux
interface is `can2` (the PCI card's physical L2/CAN2 port, originally named
`pciecan2`), and both motors
share that bus. The recorded bench identities are CAN IDs 1/2 and feedback
Master IDs `0x11`/`0x12`. The interface must already be UP at the verified
motor bitrate; this package never changes host bitrate or motor flash.

## Connections to ros2_control

Include `urdf/damiao.ros2_control.xacro` in the robot's *authoritative* model
composition and call `damiao_head_system` with the two approved logical joint
names, their measured position bounds in rad, positive velocity limits in
rad/s and explicit timeouts. This package does not insert unapproved head
joints into the current 14-axis model. The macro's ID/interface defaults are
the recorded bench wiring; override them if the installation changes. Do not
reuse `turn`, which is an existing EtherCAT joint.

The plugin exports one standard `position` command and `position`, `velocity`,
`effort` states per joint. Additional diagnostic states are `fault_code`,
`enabled`, `feedback_age_ms`, `mos_temperature`, and `motor_temperature`.
`rt_control_bringup/config/controllers.yaml` owns one two-joint position trajectory
controller and one local state broadcaster for both head joints, using logical names
`head_motor_1_joint` and `head_motor_2_joint`. The optional bringup path checks
that both names agree with the hardware configuration. The shared hardware plugin
requires both position interfaces to start and stop together. The isolated
`config/controllers.example.yaml` remains an illustration only.
Leave the trajectory controller inactive at startup; activating it enables
**both** motors, so activation requires a separately authorized hardware
procedure. Deactivation sends disable commands. Other ROS programs can use
standard `FollowJointTrajectory` and joint states; they must not send a second
set of motor-control CAN frames to the same IDs.

## Safety and lifecycle

- `on_configure` opens a filtered, nonblocking CAN_RAW socket; reads CAN ID
  `0x08`, Master ID `0x07`, CAN timeout `0x09`, bitrate code `0x23`, mode
  `0x0A`, and PMAX/VMAX/TMAX `0x15..0x17` for *each* motor. It checks the
  configured IDs, requires 1 Mbit/s (code 4), a nonzero motor watchdog and mode 2;
  sends disable frames, checks returned feedback and seeds hold commands.
  It does not change motor modes, IDs, bitrate, speed settings, zeros or flash.
- `on_activate` keeps motors disabled. The position-controller interface
  switch requires both joints and fresh fault-free disabled feedback with the
  measured positions inside the caller-supplied bounds, then seeds position commands,
  sends hold targets and enables.
- `read` receives a bounded number of matching feedback frames; enabled
  faults and stale feedback fail the hardware cycle. `write` sends two
  position-velocity float32 frames and enforces position bounds without
  silently clamping them. Inactive write polls disabled feedback at the
  configured interval. Cleanup, error, shutdown and destructor attempt
  disable on both motors; failed CAN writes cannot guarantee physical disable.
- The motor's own CAN timeout, external E-stop/STO, mechanical limits and
  host signal/launch shutdown remain independent requirements. No test here
  certifies bus load, shared IMU coexistence, direction, bounds, zero or
  recovery. BQ-150 still blocks production/shared-bus admission.

`configure_timeout_ms`, `feedback_timeout_ms`, `disabled_poll_interval_ms`,
`max_rx_frames_per_cycle`, per-joint `velocity_limit` and joint position
min/max are mandatory. Select and verify these for the final bus, motor
watchdog and mechanical design. Replace the example YAML's update-rate marker
after shared-bus timing has been approved. The `use_mock_hardware` macro option
supports controller wiring tests without touching CAN.

## Bringup integration

Normal bringup leaves the head absent. Set `RT_CONTROL_HEAD_CAN_CONFIG` to an
absolute YAML file path when using the native launcher, or pass
`head_can_config:=/absolute/path/to/head.yaml` to the bringup launch. The file
must contain `can_interface: can2`, three positive timeouts in milliseconds,
`max_rx_frames_per_cycle` in 1..256, and exactly two `joints` with `name`,
`can_id`, `master_id`, `min`, `max` (radians), and `velocity_limit` (rad/s).
The joint names must be `head_motor_1_joint` and `head_motor_2_joint` in that
order. Recorded CAN IDs are 1/2 and Master IDs are 17/18; verify them on the
installed bus before enabling. The native launcher always names L2 `can2` but
leaves it down unless the head file is selected. With the file selected, it
requires L2 to already be at 1 Mbit/s, then brings it up without changing its
bitrate. CAN0/CAN1 remain at 500 kbit/s. This is an opt-in integration, not an
approved motor-motion
recipe: there are no measured head mechanical limits, direction/zero or
verified shared-bus timeouts in the repository. The current Robot Model has no
matching physical head joints, so joint states cannot update head TF until
the model owner approves the geometry. Never substitute the motor register's
electrical range for the installed mechanism's safe joint limits.

For a hardware-free build and test:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_hw_can
colcon test --packages-select robot_hw_can
colcon test-result --verbose
```
