# ctrl_swerve

Protocol-independent four-module swerve algorithms and a ROS 2 Humble
`ctrl_swerve/SwerveController` plugin. Modules are indexed in wiring order
(`module_1..4`); front/rear and left/right labels are derived from each module's
calibrated `module_x/module_y`, never from the index. On `Alfa_V3-Advantech-01`
module 1..4 are FR, RR, FL, RL (hw-config-20261116-02#F1).
See [NOTICE.md](NOTICE.md) for the exact source revision of the migrated math.

## Implemented

- Chassis discretization, inverse/forward kinematics, mechanical-interval steering
  branch selection with hysteresis, bounded steering slew, speed desaturation and
  alignment/cosine gating.
- Measured wheel-position-increment odometry, measured twist and optional IMU
  yaw/rate with wheel-yaw fallback and continuity on source changes.
- Fixed-size least-squares residual checking rejects an inconsistent wheel from
  measured twist and position odometry, reports its location-derived FL/FR/RL/RR label and
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

`max_linear_acceleration` bounds the XY acceleration norm; `max_angular_acceleration`
bounds yaw acceleration separately. `max_body_jerk` retains the norm bound on
`(vx, vy, omega * R)`, with `R` the farthest module radius. These replace the old
combined `max_body_acceleration` parameter; regenerate deployed artifacts rather
than reusing old runtime YAML. The template uses 1 m/s translation, 1 m/s² XY
acceleration, 0.8 rad/s ordinary yaw, π/8 rad/s² ordinary yaw acceleration and
4 m/s³ equivalent-radius jerk. Actual stopping distance requires hardware acceptance.

Ruckig uses fixed three-axis velocity inputs before inverse kinematics. Steering
is planned while wheels wait at zero, then velocity shaping starts after the
selected alignment gate releases. The default `steer_while_driving` mode retains
cosine compensation and a 90-degree target-minus-actual error gate; the 5/3-degree
threshold/release hysteresis applies only to explicitly selected `align_first`.

Normal zero-velocity commands use the same S-curve to stop. The 500 ms command
watchdog, alignment stops, and invalid-feedback stops retain their existing stop
policies; they are not delayed to preserve comfort jerk bounds. Cosine compensation,
wheel-speed saturation and per-wheel acceleration limits still apply downstream,
so the body shaper is not a guarantee of exact wheel coordination in every transient.
Odometry continues to use measured wheel feedback, never the smoothed command.

Navigation must account for the shaping delay and measured stopping distance.
On rotation authorization revocation, the advertised limits change immediately.
The user-approved reference policy (2026-11-25, #339) retains shaping state:
outward angular acceleration may briefly persist while the new acceleration
bound and jerk trajectory converge. Angular velocity is not hard-clipped and
must converge to the ordinary limit; translation from a pure spin waits until
both ordinary angular velocity and acceleration bounds permit it. Emergency
stops retain priority. This replaces the initial PR336 policy that immediately
cleared outward angular acceleration, which did not match the published predictor.

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

Endpoints (N-04 and R-OUT-02 are public contract names):

| Endpoint | Type | Meaning |
| --- | --- | --- |
| `/rt/base/cmd_vel` | `geometry_msgs/msg/Twist` | Planar chassis velocity (N-04) |
| `/rt/base/odom` | `nav_msgs/msg/Odometry` | Measured motion, published at 50 Hz (R-OUT-02) |
| `/rt/base/terminal_rotation_request` | `std_msgs/msg/Bool` | Navigation intent with 300 ms local monotonic lease |
| `/rt/base/terminal_rotation_active` | `std_msgs/msg/Bool` | Actual admitted rotation state, 50 Hz |
| `/rt/base/execution_state` | `std_msgs/msg/Float64MultiArray` | Internal execution snapshot, v1 / 91 fields, 100 Hz |
| `/rt_control/chassis/execution_state` | `std_msgs/msg/Float64MultiArray` | Explicitly requested same-snapshot compatibility endpoint |
| `/rt_control/chassis/joint_states` | `sensor_msgs/msg/JointState` | Normalized physical-module feedback, 100 Hz; not URDF joint state |
| `~/imu` (optional) | `sensor_msgs/msg/Imu` | Already aligned, configured base-frame IMU |
| `~/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Controller execution state |

No odometry TF, global joint-state publisher or enable/reset/zero service is created.

Rotation admission requires a fresh zero velocity command, healthy enabled feedback,
all measured wheel speeds <= `rotation_stop_speed` (0.01 m/s), and zero drive output
continuously for `rotation_stop_hold` (0.05 s). Only admitted pure rotation uses
`max_rotation_speed` / `max_rotation_acceleration` (1.5 rad/s / 1.5 rad/s²).
Translation, false/expired intent, stale command, invalid feedback, faults and
deactivation revoke authorization. Every activation needs a newly received request.

Application schema authority is `robot_interfaces/contract/navigation-execution-v1.md`
([correction #339](https://github.com/SevenovaHangzhou/robot_driver/issues/339)). The
91-field wire format follows the published FollowMPPI v3.0.4 consumer: 27 header
fields and four 16-field modules, each with an explicit physical module ID. The
undeployed 31+4×15 proposal is withdrawn; never publish both layouts on the same
topic. Final steering goals and limited references are distinct; global translation
branch, per-module holding and effective gate/release/hysteresis restore prediction state;
execution wheel speeds are m/s. Normalized JointState uses neutral interleaved
`module_N_steering` / `module_N_wheel` names, steering heading θ, wheel rad/s, and
NaN for unavailable observations. Do not feed it to robot_state_publisher. Module
order is the physical wiring order, not a guessed wheel quadrant or model suffix.
No model, mapping, motor direction, calibration, existing wheel observations or
odometry interface changes are included. Publish disabled snapshots while configured;
consumers must also apply freshness checks. Interface review and coordinated release
must precede downstream SHA pinning and navigation deployment.

Steering velocity is an observation derived only from fresh calibrated encoder
samples, not a reference trajectory or a repeated ROS publication. The current
CANopen interface exposes receive age rather than a device sequence; a strict age
fall at the controller cycle identifies a newly received sample. At least two valid
samples are needed. Missing feedback, invalid time and lifecycle reset invalidate
the history, leaving NaN until a new pair exists. This is host receive timing, not
a claim of hardware-synchronized sampling. Verify timing and stopping detection on
the target before deployment; never replace unknown rates with zero.

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

For offline integration with the navigation handoff, configure with
`-DFOLLOW_MPPI_REFERENCE_DIR=/path/to/driver_handoff_20261124/reference_code`.
This builds the supplied real decoder and production prediction sources outside
the repository, and enables consumer/stop checks, source-lock verification and
stepwise replay. The default package tests do not establish cross-repository
compatibility without this option. Keep the reference bundle immutable and record
its source lock with the test evidence; do not vendor a second navigation tree.

`SWERVE_ENABLE_COVERAGE=ON` instruments the two package libraries with GCC coverage.
Tests use synthetic geometry and in-memory or GenericSystem hardware and cover the
math, control and lifecycle behavior. They provide no device evidence.
