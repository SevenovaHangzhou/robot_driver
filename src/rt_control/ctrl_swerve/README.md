# ctrl_swerve

Protocol-independent four-module swerve algorithms and a ROS 2 Humble
`ctrl_swerve/SwerveController` plugin. The module order is FL, FR, RL, RR.
See [NOTICE.md](NOTICE.md) for the exact source revision of the migrated math.

## Implemented

- Chassis discretization, inverse/forward kinematics, mechanical-interval steering
  branch selection with hysteresis, bounded steering slew, speed desaturation and
  alignment/cosine gating.
- Measured wheel-position-increment odometry, measured twist and optional IMU
  yaw/rate with wheel-yaw fallback and continuity on source changes.
- Fixed-size least-squares residual checking rejects an inconsistent wheel from
  measured twist and position odometry, reports its FL/FR/RL/RR identity and
  inflates covariance while the rejection is active.
- Navigation contract: four steering-position (CSP) and four wheel-velocity (CSV)
  commands. The controller owns no standalone bus thread, EtherCAT master or motor
  enable/controlword writer.
- Explicit geometry/travel/feedback validation, stop-on-invalid-feedback behavior,
  local command timeout and activation-generation isolation of queued commands.
- Installed plugin and non-runnable [configuration template](config/controller.draft.yaml).

The new package does not contain the old motor protocol, `rezero`, MIT gain
parameters or wheel torque-feedforward calculation. Legacy gain parameters are
rejected rather than silently accepted. Kinco's internal loops are tuned through
the hardware configuration/commissioning process, not by this controller.

## Chassis Velocity S-Curve

Optional `max_body_acceleration` and `max_body_jerk` limit the norm of acceleration
and jerk of `(vx, vy, omega * R)`, where `R` is the largest module distance from the
body origin. Units are m/s² and m/s³. Both parameters are read during configure;
set both to finite positive values to enable, or both to `.inf` to retain the
existing behaviour. The template enables 1.0 m/s² and 4.0 m/s³ for the authorized
Advantech deployment; these are initial tuning values, with real stopping distance
and navigation interaction still awaiting operator acceptance. No new command topic is introduced.

The implementation uses Ruckig's fixed three-axis velocity interface before
inverse kinematics. Component limits are the norm limits divided by √3. Phase
synchronization preserves ratios for collinear states, including starts from rest;
arbitrary moving retargets use time synchronization when phase synchronization is
not possible. There is no nonzero startup seed. Steering is planned while wheels
wait at zero, and acceleration starts only after the alignment gate releases.

Normal zero-velocity commands use the same S-curve to stop. The 500 ms command
watchdog, alignment stops, and invalid-feedback stops retain their existing stop
policies; they are not delayed to preserve comfort jerk bounds. Cosine compensation,
wheel-speed saturation and per-wheel acceleration limits still apply downstream,
so the body shaper is not a guarantee of exact wheel coordination in every transient.
Odometry continues to use measured wheel feedback, never the smoothed command.

For reference only, a local simulation using 1.0 m/s² and 4.0 m/s³ stops from
0.3 m/s in approximately 0.77 s over 0.115 m. These are software results, not measured
hardware stopping performance. Navigation must account for the resulting delay and
stopping distance. Persistent rotation still requires command/feedback diagnosis.

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

Steering must report mode 8 and drives mode 9. All eight axes must report Operation
Enabled. The controller claims no `control_word`; `enable_manager` retains motor
enable/disable/reset ownership. A shared master does not require a second transport owned
by this package.

The external encoders are mandatory observations, not an outer steering position
loop. Their angle is used for module optimization, cosine/alignment gating and
odometry. A zero chassis command holds motor position; it does not steer the motor
toward an encoder discrepancy. Invalid position feedback uses the last valid
motor position, not the previous planned target, as the available hold reference.

## Commands And Lifecycle

Private endpoints:

| Endpoint | Type | Meaning |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Planar chassis velocity |
| `~/odom` | `nav_msgs/msg/Odometry` | Measured motion, published at 50 Hz |
| `~/imu` (optional) | `sensor_msgs/msg/Imu` | Already aligned, configured base-frame IMU |
| `~/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Controller execution state |

No odometry TF, global joint-state publisher or enable/reset/zero service is
created. Future module bringup must route commands from the existing authorized
`/cmd_vel` path and odometry to `/wheel/odom`, without publishing a new
cross-domain contract or bypassing Motion. Existing production remappings are unchanged.

Command freshness follows N-04: at most 500 ms since local receipt of Twist,
with no fabricated message timestamp or frame. Non-finite/non-planar commands
stop drive output. Linear/angular/wheel limits and wheel acceleration are bounded
by explicit parameters. Each wheel evaluates the positive-speed and reversed-speed
equivalent angle inside its calibrated mechanical interval. A pure translation uses
one feasible aggregate branch for all four wheels when possible. Target interpolation
is linear inside the interval, so `+179 deg -> -179 deg` is never treated as a two-degree
cross-limit move. There is no continuous-joint mode or lower-layer branch reselection.

`steering_min/max` are per-wheel physical endpoints. `steering_limit_margin` shrinks
the commandable target interval, while `steering_limit_tolerance` only admits small
measurement uncertainty outside the physical endpoint before clamping the planning
observation back into the safe interval. It never expands a command target. All four
safe intervals must span at least 180 degrees and no physical interval may exceed
360 degrees. These values and `steering_angle_deadband` remain calibration outputs.

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

`slip_residual_threshold` is a positive wheel-vector residual in m/s and must be
derived from recorded healthy and induced-slip data. A rejected wheel does not
accumulate a deferred position correction when it recovers. Residual checking
cannot detect common-mode wheel error that remains exactly consistent with a
different rigid-body chassis velocity; an independent IMU or external pose source
is still required to observe that failure mode. `slip_covariance_scale` must be at
least one and is multiplied with the existing IMU-fallback and missing-module scales.

## Remaining Hardware Integration

The CANopen
steering-encoder provider likewise still needs its actual EDS/node/interface configuration.
CANopen SYNC is specified at 4000 us, with one synchronous TPDO response per encoder per
SYNC. The controller does not emit SYNC or synthesize fresh encoder feedback. Device
support, bus load, feedback age and phase relative to EtherCAT DC remain unverified, and
the allowed discrepancy must account for measured timing skew.

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
colcon build --packages-up-to ctrl_swerve --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ctrl_swerve
colcon test-result --verbose
```

`SWERVE_ENABLE_COVERAGE=ON` instruments the two package libraries with GCC coverage.
Tests use synthetic geometry and in-memory or GenericSystem hardware and cover the
math, control and lifecycle behavior. They provide no device evidence.
