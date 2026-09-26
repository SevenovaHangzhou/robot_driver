# swerve_driver

Protocol-independent four-module swerve algorithms and a ROS 2 Humble
`swerve_driver/SwerveController` plugin. The module order is FL, FR, RL, RR.
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
- Default navigation contract: four steering-position (CSP) and four wheel-velocity
  (CSV) commands. The opt-in ELECTRI-133 extension additionally claims the four drive
  position/mode and sequence/mask handoff interfaces described below. Neither path owns
  a standalone bus thread, EtherCAT master or motor enable/controlword writer.
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
| Each drive joint (default NAVIGATION) | `velocity` | `position`, `velocity`, `status_word`, `mode_of_operation_display` |
| Each drive joint (`relative.enabled=true`) | default interfaces plus `position`, `mode_of_operation`, `write_sequence`, `write_mask` | default states plus adapter freshness/sequence/mode-ack states listed in the Tier A section |
| Each external steering encoder | none | `position`, `feedback_age_ms` |
| `ethercat_domain` | none | `process_data_age_ms` |

All unit conversions, encoder gearing, signs and static offsets belong to the
hardware adapter/calibration. Steering joint position is the **motor-derived
estimate in output-axis radians**. External encoder position is the separately
measured output-axis angle in the same calibrated continuous coordinate system.
Neither state overwrites the other. Drive position and velocity are wheel radians
and radians/second. The controller uses wheel radius to obtain metres/metres per second.

Steering must report mode 8. Drives report mode 9 in NAVIGATION and, only after the
acknowledged Tier A handoff, mode 8 in OPERATION. All eight axes must report Operation
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

The source now contains the opt-in `KincoCyclicModeSlave` and connected controller
adapter, but **not a commissioned machine backend**. Production still needs a reviewed
Kinco PDO/profile with real identity/ring placement, conversions, signs, offsets, limits,
watchdog/DC behavior and transition timing. The current profile remains draft and
`relative.profile_verified` must stay false until those facts are confirmed. The CANopen
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

## ELECTRI-133 Tier A: relative execution and cyclic adapter integration

`RelativeMovePlanner` and `RelativeMoveSession` add a fixed-size, allocation-free
C++17 execution core. The separate `chassis_relative_move_mock` executable implements
the RT-private [chassis protocol](../../interfaces/rt_control_interfaces/README.md).
This executable has an ideal in-memory plant and no hardware interface, plugin loader,
bus access, controlword writer or production launch connection. Separately, the
opt-in `relative.enabled` extension connects the same fixed-size session to the real
`SwerveController` loaned interfaces and the `KincoCyclicModeSlave` cyclic adapter.
The extension is source-complete and synthetic-PDO tested, but is absent from production
launch and cannot be enabled until the real profile and calibration gates below close.

Goals are `(dx, dy, dyaw)` in the configured execution-start base frame, metres and
radians, bounded by 0.700 m, 15 degrees and 0.2 m/s translation. Per-goal limits must
be positive and no greater than explicit server limits. One goal can be reserved;
there is no queue or preemption. Each action goal, feedback and result carries an
opaque `corridor_token` string. The adapter copies it unchanged from that goal's
accepted ROS handle, including empty tokens, without safety/lease validation and
without passing strings into the fixed-size core. Both endpoints must rebuild their
generated action types for this wire-layout addition.
The SE(2) logarithm defines fixed body twist ratios.
Steering aligns once using a bounded common ramp; wheels then follow one scalar
trapezoidal/triangular CSP profile. Cancellation decelerates the current alignment
ramp or drive profile, preserves reference continuity and holds the attained targets.
No moving target is reset to a measured position. A zero goal passes all admission
checks, holds all targets and reports progress 1 after stationary settling.

The mode service requires confirmation and fresh wheel **and steering** stationary
feedback for a configured dwell. Its response is deferred until all four drives confirm
CSV/NAVIGATION or CSP/OPERATION; steering remains CSP. Failed readback latches a fault.
The standalone mock still discards `cmd_vel` in every mode. The opt-in controller path,
however, accepts local velocity only on `/cmd_vel`, freezes that admission before the
CSV stop, and tags each sample with the current navigation generation and original local
receipt time. No Twist is retained across mode changes or lifecycle restart. In the
offline mock, NAVIGATION intentionally collapses TRANSIT and DOCKING; this does not
implement or validate their separate production behavior.

CSV to CSP uses a bounded stop plus fresh stationary dwell. It seeds every drive's
position command from actual wheel position, requires an exact per-drive sequence that
the adapter records only after the PDO send hook, then permits the mode-8 write. Relative
targets remain blocked until all four drives provide newer PDO feedback with mode 8,
configured Operation Enabled/status conditions, adapter acknowledgement and Kinco target
valid. The reverse path preserves the executor's stopped CSP target, sends zero velocity
before mode 9, waits for an acknowledged sent-zero cycle and fresh all-four mode/status
confirmation, advances the navigation generation, and requires a newly received command.
Command assignment, elapsed time and `0x6061` readback alone are never treated as proof
that a PDO command was sent. The adapter reports a sent request; physical drive acceptance
is represented only by later `0x6061`/status feedback and remains hardware-dependent.

IMU validity is mandatory for every goal. Wheel travel and external steering supply
translation, IMU supplies yaw, and wheel yaw is retained for independent discrepancy
reporting. IMU/required-feedback loss freezes and invalidates the last coherent pose
snapshot. Slip, steering error and drive/bus faults stop and latch; drive/bus/feedback
loss inhibits output and aborts without claiming a controlled stop. With authority
available, faults and timeouts decelerate on the existing path. Stop failure aborts,
preserves the first fault, and does not claim stationarity. Reset checks healthy,
fresh, stationary feedback and unresolved tracking/slip causes; it never resets or
enables a drive, changes mode, changes targets or replays work.

Success requires wheel and motor/external steering position errors within
`position_tolerance`, their velocities within `velocity_tolerance`, plus stationary
and settling dwell. It is independent of accumulated quality bits for pose residual,
IMU-versus-wheel yaw discrepancy and invalid estimate. No yaw correction, corrective
base move, rephotograph, obstacle logic, ultrasonic consumer or Tier B is included.

All configuration is explicit and immutable after node construction. Geometry,
radii, mechanical intervals and wheel limits are four-element arrays in FL/FR/RL/RR
order. Core distances are metres, angles/output-axis positions are radians, angular
velocities/accelerations use rad/s and rad/s², and timing values use seconds.
`slip_threshold` is a wheel-vector residual in m/s; `pose_translation_tolerance` is
metres. All other error/position thresholds are radians. `imu_max_increment` bounds
the wrapped difference of successive valid base-aligned yaw samples and must be
below pi. Freshness, stationary/alignment/settling/error/slip dwell, update watchdog,
switch/goal/stop timeouts and physical limits have no usable production defaults.
`MoveFeedback.age` represents the oldest required encoder/drive sample, not read time.
A future real adapter must supply verified conversions and actual timestamps/modes.

The installed synthetic fixture is explicitly for this no-device executable:

```bash
# After building and sourcing the isolated ROS overlay; choose an unused test domain.
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 ros2 run swerve_driver chassis_relative_move_mock \
  --ros-args --params-file "$(ros2 pkg prefix swerve_driver)/share/swerve_driver/test/relative_move_mock.yaml"
```

Without `explicit_no_device_mock=true` and every required parameter, startup fails.
The fixture is not robot calibration and cannot mark real hardware verified. Private
endpoints are `~/set_mode`, `~/relative_move`, `~/state`, `~/reset_fault` and the
ignored `/cmd_vel`. Tests can inject `mock_imu_valid`, `mock_freeze_imu`,
`mock_bus_ok`, `mock_drives_ok`, `mock_freeze_feedback`, `mock_hold_position`,
`mock_refuse_mode_switch`, `mock_imu_bias_rate`, `mock_wheel0_velocity_error` and
`mock_steering0_error`; these exist only on the standalone mock.

ROS callbacks use one mutually exclusive callback group. Goal and cancel requests
cross a capacity-one mailbox, and only one deferred mode response is retained.
The core is single-owner and fixed-size; ROS allocations, parameters and action
publication occur outside its update call. The wall-timer mock is not a production
250 Hz scheduling implementation. Timeouts use the monotonic clock from acceptance;
ROS timestamps identify the execution-start and last coherent measured snapshot.
Goal and service admission share one receipt-time freshness calculation: monotonic
elapsed time is added to cached encoder/IMU ages, and an expired update cycle rejects
admission. Mode changes require fresh encoders; reset also requires fresh IMU evidence.
Receipt checks do not advance stationary dwell. Both services return BUSY for an
occupied goal/handoff slot, then validate confirmation/legal mode before freshness;
invalid requests remain INVALID_REQUEST even with stale samples. An overdue timer faults before applying
any pending simulated mode or motion output, retaining the aged observations.
Process restart discards work and starts UNKNOWN/IDLE until readback validation;
it cannot declare an interrupted goal successful. Startup SIGINT and normal SIGINT
release resources without treating requested shutdown as bad configuration.

BQ-145/BQ-150 remain blocking for the real Kinco identity/PDO profile, per-wheel
conversion/sign/offset/limits, steering velocity freshness, stationary and timeout
calibration, physical mode/status behavior, actual IMU frame/freshness and shared-bus
readiness. `enable_manager` retains all controlword ownership; the swerve controller does
not claim a controlword interface. The draft machine profile stays unverified, production
launch/runtime admission stays closed, and no vendored public interface or dependency pin
is changed. The user-selected local velocity entry is `/cmd_vel`; public
`robot_interfaces` migration remains a separate coordinated change.

## Verification

In an isolated workspace with the project's pinned dependencies available:

```bash
colcon build --packages-up-to swerve_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select swerve_driver
colcon test-result --verbose
```

`SWERVE_ENABLE_COVERAGE=ON` instruments the two package libraries with GCC coverage.
Tests use synthetic geometry and in-memory or GenericSystem hardware. They cover
original math/control/lifecycle behavior, the planner/session, private ROS services
and actions, quality/fault/timeout/cancel semantics, restart and executable SIGINT.
The ELECTRI-133 native run passed 122 package cases (12 planner, 17 session,
20 action/service/serialization, 2 process and 71 existing cases); ASan/UBSan passed all 29
planner/session cases. The affected private-interface consumer closure also built
and tested. These results provide no device or production switching evidence.
