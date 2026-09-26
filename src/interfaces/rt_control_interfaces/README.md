# RT-Control private interfaces

This package is owned and consumed only by RT-Control. Existing `RtEnable`,
`PlcIoState` and `JointControlModeResult` remain unchanged. No other domain may
consume the chassis types or endpoints below. ELECTRI-133 Tier A adds a private
execution protocol; it does not add a public `robot_interfaces` endpoint.

## Chassis integration API

| Relative endpoint under the controller's private namespace | Type | Role |
| --- | --- | --- |
| `set_mode` | `ChassisSetMode` | Confirm a stationary NAVIGATION/OPERATION handoff |
| `relative_move` | `ChassisRelativeMove` | Execute one bounded relative SE(2) move |
| `state` | `ChassisState` | Confirmed mode, phase, first latched fault and admission state |
| `reset_fault` | `ChassisResetFault` | Explicitly clear a recoverable software latch |

The intended producer is the chassis controller; consumers are its RT-Control
adapter and explicit offline/mock test clients. This package defines types and
semantics only, not a server or hardware switch. No cross-domain consumer,
Robot Model, Tier B, obstacle planner, ultrasonic processing, rephotograph
workflow or corrective base move is introduced.

Modes, phases and first-fault codes have one spelling in `ChassisState`.
`ChassisMoveState` shares the same phase/fault values and owns the independent
quality bitmask. Service response codes and action result codes belong to their
respective generated Response/Result types. Unknown enum values, unknown quality
bits or an incompatible producer version must not be interpreted as readiness or
success. These are additive private types; the pre-existing `RtEnable`, `PlcIoState`
and `JointControlModeResult` wire types are unchanged.
Producer and private consumers must use the same reviewed package revision.
The added `corridor_token` fields change the action wire layout: rebuild and source
the matching generated interfaces for both server and clients; older generated
chassis action types are not compatible with this revision.

`ChassisRelativeMove` carries `string corridor_token` in Goal, Feedback and Result.
It is opaque correlation data, echoed unchanged from the accepted goal in every
feedback sample and all success/cancel/fault/timeout results, including failure
before execution can start. Empty strings are preserved. The adapter does not
validate or interpret the token as a safety lease, permission or corridor-clearance
claim. Rejected goals have no result and cannot replace the active goal's token;
a subsequent goal receives only its own token. String handling stays outside the
fixed-size execution core.

## Admission and limits

A goal requests the transform `(dx, dy, dyaw)` of the base origin relative to its
execution-start pose: x forward, y left, positive yaw counterclockwise about +z.
The origin and axes are those of the controller's explicitly configured base
frame (the same frame used by its wheel kinematics), not an odom/map frame or
an IMU mounting frame. Producer and private client must agree on that base
frame before admission; frame configuration cannot change during a goal.
Units are metres and radians. `hypot(dx,dy) <= 0.700` and
`abs(dyaw) <= pi/12` (15 degrees), inclusive. The translational velocity norm
must never exceed 0.2 m/s. These are task ceilings, not calibrated hardware
limits. The SE(2) logarithm of the desired transform defines fixed body twist
ratios. It is not three independent x/y/yaw ramps, nor a straight Cartesian
interpolation of those coordinates.

Every goal scalar and every `ChassisMoveLimits` field must be finite. All six
limit fields and `max_duration` must be strictly positive, including for a zero
goal. Each requested limit must be at or below the corresponding configured
server limit; values above a ceiling are rejected, not silently clamped. Wheel
velocity/acceleration are wheel-side rad/s and rad/s² and apply to every wheel;
per-wheel calibrated server ceilings remain binding. Pure translation still
requires angular limits and pure rotation still requires translational limits.
No zero value means "use a default" or "unlimited".

The server must require explicit configuration for body acceleration and angular
velocity/acceleration, per-wheel velocity/acceleration, per-wheel radius and
geometry/base-frame identity, steering range/margin/slew, encoder conversion/direction/zero,
stationary velocity thresholds and dwell, alignment tolerance/dwell/timeout,
execution position/velocity tolerance and settling dwell (wheel/steering
position in rad, velocity in rad/s), feedback/IMU freshness
and validity, slip and steering-error thresholds/dwell, pose-residual and
yaw-discrepancy thresholds, mode-switch timeout, maximum goal duration and
stop timeout/deceleration. None has an inferred production default. Missing,
nonfinite, inconsistent or unverified required production configuration makes
`ready=false`. Mock configuration must be explicitly selected and cannot mark
real hardware verified. Requested goal ceilings can only tighten these limits.

Accept only when OPERATION is confirmed, the controller is ready and stationary,
all required feedback including IMU is fresh/valid, and no fault, switch, stop
or active goal exists. Invalid IMU rejects *every* goal, including translation
and zero motion. Admission reserves the single goal slot atomically: a second
goal is rejected, never queued or used to preempt. There is no deferred goal
waiting for mode, alignment, IMU recovery or enable. A rejected ROS action goal
has no Result payload; do not manufacture a FAULTED result for it. The private
state topic provides current admission context, not a per-request rejection code.

A goal with exactly `dx=dy=dyaw=0` passes the same checks and succeeds without
steering alignment or changing position targets. It reports a valid zero
relative estimate, progress 1 and HOLDING; normal stationary/tolerance checks
still apply. Near-zero nonzero goals use the regular planner's numerically
stable SE(2) logarithm, not an implicit deadband.

## Mode handoff and restart

In the explicit offline mock only, NAVIGATION collapses TRANSIT and DOCKING into
one mode. This does not implement or validate their distinct production behavior,
and is not full production mode coverage. No TRANSIT/DOCKING enum values or real
mode-switching authority are introduced.

Service response precedence is BUSY while a goal or mode handoff owns the slot,
then INVALID_REQUEST for missing confirmation or an illegal requested mode, then
freshness and the remaining admission checks. Stale samples do not change an
otherwise invalid request into NOT_READY/CAUSE_PRESENT.

NAVIGATION uses steering CSP and drive CSV. OPERATION keeps steering CSP and
changes all drives to CSP. `ChassisSetMode.Request.confirm=true` is mandatory
in both directions. The server rejects switching while any goal is active
(including alignment, cancellation, settling or fault stopping), while not
stationary, or while a latch or required readiness condition is unresolved.
Stationary requires fresh drive/wheel and steering velocities below explicitly
configured thresholds for the configured dwell; a stale zero is not evidence.

Serialize handoffs and goal admission. Clear buffered velocity commands and
relative commands when a switch starts and on its completion/failure. Discard
all `cmd_vel` input throughout OPERATION and switching; it is not buffered for
later NAVIGATION use. Returning to NAVIGATION requires a newly received command.
Seed CSP targets from a fresh stationary measured position only during the
handoff, then confirm backend mode readback for every drive before returning
`OK`. `state.mode` reports the confirmed mode, never the requested mode.
During switching `ready=false` and phase is SWITCHING. A failed/expired switch
latches FAULT_MODE_SWITCH; unknown or mixed readback means MODE_UNKNOWN.
There is no automatic fallback to CSV. A confirmed, healthy, stationary
same-mode request is an idempotent OK with no hardware transition; it still
requires confirmation and clears stale command buffers.

Service completion is bounded by configured handoff/reset checks. A client-side
RPC timeout is not proof that a switch failed or was canceled: observe fresh
state and reconfirm before sending motion. Concurrent requests return BUSY.
Reset is an atomic software-latch operation, not an asynchronous hardware reset.
`reset_fault` requires confirmation, no active goal/switch/stop, stationary fresh
feedback and an absent underlying cause. It returns CAUSE_PRESENT otherwise,
or RESTART_REQUIRED when backend recovery requires a separate lifecycle restart.
A healthy no-latch reset is idempotent OK. It never changes mode, restarts a bus,
resets a drive, enables hardware, rewrites calibration or replays a command.

Startup/restart begins with MODE_UNKNOWN, IDLE, no active UUID and `ready=false`;
revalidate configuration, backend readback and fresh feedback before admission.
No accepted goal, accumulated progress, queued command or retained DDS sample is
restored or replayed after restart. An interrupted client goal is not declared
successful and may have no terminal result after process loss; clients must
observe fresh state and explicitly issue a new request. Runtime command/service
delivery must be volatile; state consumers must enforce configured freshness.
Software restart alone cannot establish that a physical fault is cleared.

## Execution, cancellation and terminal states

Immediately before alignment, atomically capture encoder origins and valid IMU
yaw together with ROS `execution_start`. That snapshot defines a frozen local
base frame for the whole goal, including cancellation and its final result;
no TF frame or globally accumulated pose is created. Revalidate feedback at
that boundary. `stamp` is the measurement snapshot time; age, timeouts and
`max_duration` use a monotonic clock, independent of ROS clock jumps.

Align steering once with drives stationary. Then advance all wheel CSP targets
from their captured origins using a common scalar triangular/trapezoidal
progress law and the fixed SE(2) body twist ratios. The scalar satisfies all
body and per-wheel velocity/acceleration ceilings. Keep the selected steering
branch and final steering targets through execution, cancellation and hold.
No active yaw correction, second steering alignment, pose correction or
rephotograph is performed. A cancellation during alignment stops steering
within its configured limits and holds that attained alignment target without
starting drive travel.

Accepting a ROS cancel request means cancellation has begun, not that the
robot is already stopped. During drive travel, phase STOPPING decelerates the
same scalar along the same path with the existing limits, then holds the
resulting commanded CSP positions. Never jump targets to measured positions
while moving. Preserve a feasible braking envelope to stop on the bounded
path; reject an infeasible goal at admission. Complete cancellation only after
configured stationary/settling confirmation. Cancel after a terminal result has
no effect; concurrent finish/cancel is serialized so one terminal outcome wins.

`max_duration` is an acceptance-to-completion budget including alignment,
travel and settling, capped by required server configuration. Reject plans
whose known execution/alignment/settling budget cannot fit it. Expiry initiates
stop and latches FAULT_TIMEOUT; safe stopping may extend beyond that deadline
but is bounded by a separate required stop timeout. The deadline never permits
instant target reset, disabled protection or a false claim of stationary hold.

| Result code | ROS terminal state | Meaning |
| --- | --- | --- |
| SUCCEEDED | SUCCEEDED | Encoder execution completed within configured position/velocity tolerance and settling dwell; final steering held |
| CANCELED | CANCELED | Accepted cancellation completed with verified stationary hold on the path; progress may be below 1 |
| TIMED_OUT | ABORTED | Goal deadline expired, fault latched; state reports final stopping/hold evidence |
| FAULTED | ABORTED | Execution fault or inability to confirm a stop, including a fault during cancellation |

A fault/stop failure takes precedence over CANCELED or SUCCEEDED; after deadline
expiry a successful stop reports TIMED_OUT, while an additional stop/drive/bus
failure reports FAULTED. A terminal FAULT phase does not imply that the chassis
is stationary; consult fresh `ChassisState.stationary`. Stop timeout produces
FAULTED with FAULT_EXECUTION if no earlier fault exists. `fault_code` preserves
the first latched cause, so it can differ from the latest terminal outcome.

## Pose estimate, quality and fault handling

Integrate drive-wheel travel with external measured steering angles for
translation. Use IMU as primary yaw; wheel kinematics independently supplies
`wheel_yaw`. `actual_dx/actual_dy/actual_dyaw` are the estimated displacement in
the frozen frame, not the setpoint or a claim of ground truth. `imu_yaw` and
`wheel_yaw` are continuous relative angles from that same snapshot;
`actual_dyaw=imu_yaw` while valid and
`yaw_difference=imu_yaw-wheel_yaw` with no +/-pi wrapping.

IMU loss during any active goal initiates stop, latches FAULT_IMU_LOST and sets
`estimate_valid=false` and QUALITY_ESTIMATE_INVALID for the remainder of that
goal. Required wheel/external steering feedback loss likewise invalidates the
estimate and latches FAULT_FEEDBACK_INVALID. Freeze pose/yaw fields at the last
coherent valid snapshot after invalidation (zero if none); do not fabricate
continued motion estimates or restore validity mid-goal. `stamp` then remains
the last valid measurement time. State publication time must not masquerade as
a fresh estimate. Independent diagnostics may continue outside this snapshot.

Slip detection, steering tracking errors and drive faults initiate stop and
latch the corresponding fault. Use controlled same-path deceleration only
while required drive authority and feedback remain available. Drive/bus loss
cannot promise controlled stopping or hold: report FAULTED, preserve the fault,
invalidate the pose when required feedback is lost, inhibit further commands,
and defer to the existing hardware safety chain. Do not fall back to measured
target jumps, automatic reset or renewed motion.

Encoder execution success is independent of pose quality. At terminal hold,
QUALITY_POSE_RESIDUAL reports configured translation/yaw residual thresholds
against the requested transform. QUALITY_YAW_DISCREPANCY reports the configured
IMU-versus-wheel difference threshold and is report-only, never a yaw correction
or stop trigger. These bits may accompany SUCCEEDED. Quality bits accumulate
throughout the goal; QUALITY_POSE_RESIDUAL is evaluated only at terminal hold
so normal partial progress is not called inaccurate. Invalid estimates always
set QUALITY_ESTIMATE_INVALID; their frozen values are not residual evidence.
Quality flags are not obstacle clearance, slip absence or a safety certificate.

## Validation boundary

This is an offline interface delivery, risk level high for eventual motion
integration. The generated types can be built and serialized without devices.
The schema cannot enforce finite values or runtime state transitions; the
controller must implement and test every admission, cancellation and fault rule.
Real Kinco CSV/CSP switching, PDOs, calibration, stopping behavior and shared-bus
IMU admission are unproven (BQ-145/BQ-150). Production runtime remains fail-closed.
Mock success cannot remove those blockers or authorize real motion.
