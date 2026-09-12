# swerve_driver

Protocol-independent four-module swerve algorithms and a ROS 2 Humble
`swerve_driver/SwerveController` plugin. The module order is FL, FR, RL, RR.
See [NOTICE.md](NOTICE.md) for the exact source revision of the migrated math.

## Implemented

- Chassis discretization, inverse/forward kinematics, equivalent-angle flipping
  with hysteresis, steering slew, speed desaturation and alignment/cosine gating.
- Measured wheel-position-increment odometry, measured twist and optional IMU
  yaw/rate with wheel-yaw fallback and continuity on source changes.
- Eight ros2_control commands: four steering positions (CSP) and four wheel
  velocities (CSV). No standalone CAN thread, EtherCAT master or motor enable writer.
- Explicit geometry/travel/feedback validation, stop-on-invalid-feedback behavior,
  local command timeout and activation-generation isolation of queued commands.
- Installed plugin and non-runnable [configuration template](config/controller.draft.yaml).

The new package does not contain the old motor protocol, `rezero`, MIT gain
parameters or wheel torque-feedforward calculation. Legacy gain parameters are
rejected rather than silently accepted. Kinco's internal loops are tuned through
the hardware configuration/commissioning process, not by this controller.

## Hardware Interface Contract

| Component | Commands claimed | States consumed |
| --- | --- | --- |
| Each steering joint | `position` | `position`, `status_word`, `mode_of_operation_display` |
| Each drive joint | `velocity` | `position`, `velocity`, `status_word`, `mode_of_operation_display` |
| Each external steering encoder | none | `position`, `feedback_age_ms` |
| `ethercat_domain` | none | `process_data_age_ms` |

All unit conversions, encoder gearing, signs and static offsets belong to the
hardware adapter/calibration. Steering joint position is the **motor-derived
estimate in output-axis radians**. External encoder position is the separately
measured output-axis angle in the same calibrated continuous coordinate system.
Neither state overwrites the other. Drive position and velocity are wheel radians
and radians/second. The controller uses wheel radius to obtain metres/metres per second.

Steering must report mode 8, drive mode 9, and all eight drives must report
Operation Enabled. The controller claims no `control_word`; `enable_manager`
retains motor enable/disable/reset ownership. A shared master does not require a
second transport owned by this package.

The external encoders are mandatory observations, not an outer steering position
loop. Their angle is used for module optimization, cosine/alignment gating and
odometry. A zero chassis command holds motor position; it does not steer the motor
toward an encoder discrepancy. Invalid position feedback uses the last valid
motor position, not the previous planned target, as the available hold reference.

## Commands And Lifecycle

Private endpoints:

| Endpoint | Type | Meaning |
| --- | --- | --- |
| `~/cmd_vel` | `geometry_msgs/msg/Twist` | Planar chassis velocity |
| `~/odom` | `nav_msgs/msg/Odometry` | Measured motion, published at 50 Hz |
| `~/imu` (optional) | `sensor_msgs/msg/Imu` | Already aligned, configured base-frame IMU |
| `~/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Controller execution state |

No odometry TF, global joint-state publisher or enable/reset/zero service is
created. Future module bringup must route commands from the existing authorized
`/cmd_vel_safe` path and odometry to `/wheel/odom`, without publishing a new
cross-domain contract or bypassing Motion. Existing production remappings are unchanged.

Command freshness follows N-04: at most 500 ms since local receipt of Twist,
with no fabricated message timestamp or frame. Non-finite/non-planar commands
stop drive output. Linear/angular/wheel limits and wheel acceleration are bounded
by explicit parameters; steering targets outside declared travel are rejected.

Any invalid/stale required feedback, mode/state mismatch or excessive difference
between motor-derived and measured steering angles gates all drive commands to
zero. Steering holds the available valid motor-position reference. This is a
software command policy, not proof of braking distance or physical stopping.
There is no automatic encoder-source substitution, NMT recovery or re-enable.
Recovery requires valid feedback and a newly received command; an old command
cannot resume after feedback failure or controller reactivation.

Configuration and activation perform allocation and ROS entity setup; they must
be scheduled in the supervised no-motion startup/switching phase. `update()` uses
fixed arrays and bounded computation. DDS callbacks communicate through fixed-size
realtime buffers; odometry publication uses the standard realtime publisher and
diagnostics run outside update. Target-host timing and lifecycle jitter need HIL validation.

## Remaining Hardware Integration

This package contains a controller, **not the Kinco or CANopen hardware backend**.
The actual Kinco profiles and a CANopen sensor provider exporting the above
timestamped interfaces still need to be connected in their hardware-owner packages.
CANopen SYNC is specified at 4000 us, with one synchronous TPDO response per encoder
per SYNC. The controller does not emit SYNC or synthesize fresh encoder feedback.
Device support, bus load, feedback age and phase relative to EtherCAT DC remain
unverified, and the allowed discrepancy must account for measured timing skew.

Actual robot joint names, calibrated geometry/scales/limits, ring topology and
scope-specific controller lifecycle composition remain prerequisites. The
alfa_v3 module launch stays static-only; installing this package does not enable
the draft physical profiles or alter the old-machine launch.

`calibration_verified` defaults to false, other commissioning values have no
usable implicit defaults, and the draft template contains TBD values. Do not
use the synthetic tests or URDF fixture as a production configuration. An optional
IMU requires an explicit aligned frame and timeout/norm/jump-validation parameters.

## Verification

In an isolated workspace with the project's pinned dependencies available:

```bash
colcon build --packages-up-to swerve_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select swerve_driver
colcon test-result --verbose
```

`SWERVE_ENABLE_COVERAGE=ON` instruments the two package libraries with GCC coverage.
Tests use synthetic geometry and in-memory or GenericSystem hardware. They cover
the original math, control-interface behavior, lifecycle isolation, faults and
an actual ROS command-to-feedback odometry round trip without accessing devices.
