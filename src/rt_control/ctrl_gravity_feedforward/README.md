# Static gravity feedforward

`GravityController` computes `tau_g = dU/dq` from actual URDF joint positions,
including all fixed tool descendants. It writes additive output torque in N·m
through `joint/effort`; the trajectory controller owns `joint/position`.

The independent composition/launch option `gravity_feedforward` defaults to
`false`. The supported machine scopes declare their optional gravity controllers.
`jtc_only` continues to select the rolling view independently. Generate an artifact
with `compose_artifact --gravity-feedforward`, then launch that artifact with
`gravity_feedforward:=true`. This is a startup selection, not a live torque toggle.
Existing artifacts without the new selection key retain the disabled meaning.

## Conversion and commissioning

The protocol owner `hw_ethercat/CspTorqueOffsetSlave` preserves upstream CSP
position/state handling and adds the drive object `0x60B2:00` (Torque offset,
INT16, per thousand of the drive rated current) to RxPDO. For each axis:

```
effort_target_nm = clamp(scale * tau_g, -max_effort_nm, max_effort_nm)
current_offset_permille = torque_sign * permille_per_nm * effort_slew_limited_nm
```

This is an additive drive input. No measured current or PID output is subtracted.
The table uses empirical candidates where selected and otherwise the ideal
reference `permille_per_nm = 1e6 / (Kt * mechanical_ratio * rated_current_mA)`.
An efficiency assumption of 1 is a nominal reference, not measured efficiency.
No diagnostic intercept or friction/damping estimate is added.

The per-unit conversion is an exported snapshot of the external single motor
registry, with its SHA-256 and basis stored per axis. Startup preflight reads
`0x6075` as uint32 mA and rejects any mismatch before runtime nodes start. It does
not change that object. Actual PDO mapping changes require the normal hardware
commissioning authorization.

The first candidate policy is user-confirmed on 2026-11-17: scale 0.1, each
axis limited to +/-100 permille, normal continuous command slew at 20 permille/s.
The exporter divides the current limits/rates by each axis gain to produce the
N·m policy. INT16 output has one-permille quantization. These are commissioning
settings; the gains are not independently accepted force/current calibration.

## Model and lifecycle

All controller parameters are read during configure:

- `robot_description`, `base_link`, `tip_link`, `joints`: the expanded authoritative
  model and exact chain order. Fixed off-chain tool bodies are included once.
- `gravity_in_base`: three gravity components. With `gravity_in_root=true`, the
  vector is instead expressed in the URDF root and transformed through the model.
  Only fixed/prismatic ancestors are admitted; a rotating unmeasured base is rejected.
- `scale` in [0,1], positive per-axis `max_effort_nm` and `max_effort_rate_nm_s`.
- `feedback_age_interfaces`, `max_feedback_age_ms`, `health_interfaces`, `health_values`:
  all selected PDO domains and existing bus health signals, with the current motion policy's freshness limit.

Activation requires complete interfaces, enabled axes and valid feedback. The
controller only claims effort; enable_manager treats it as an enable companion,
including deactivation and orderly shutdown. Configure/activate/update failure,
disabled axes, stale/invalid feedback and lifecycle exit clear its commands.

The hardware plugin additionally gates the offset on complete EtherCAT data,
operational state, CSP position takeover and the requested enable control word.
It writes zero for invalid input or a closed gate. `gravity_command_epoch` is a
software state counter incremented on forced zero; the controller restarts that
axis's fractional slew accumulator when it changes. This avoids replaying a
previous nonzero offset after a short bus interruption without losing sub-integer
ramp progress to PDO quantization. Fault clearing is immediate; normal slew limits
do not delay it. No software callback can guarantee delivery over a lost bus.

`torque_offset_permille_sent` records the exact integer encoded in the outgoing
buffer. `gravity_effort_sent` reports its N·m equivalent using the configured gain,
not measured joint torque. Runtime-LOG automatically includes these state interfaces.

The first candidate is for the existing unloaded tools. It has no additional
payload, friction, damping or dynamic acceleration model. Deployment must retain
the current authoritative model, the confirmed joint limits and the existing
plan-stop implementation. The driver checkout's model copy and the active robot
model have different hashes; compare the generated deployment before switching.

## Verification

Build and test `ctrl_gravity_feedforward`, `hw_ethercat` and `integ_composer` with
the repository-pinned upstream dependencies and patches. The tests cover the
conversion, gating, wire mirrors, slew restart, model sign and tool inclusion,
controller lifecycle and independent opt-in composition. Synthetic test limits
are never installed into machine policy.

`test/mock_lifecycle.py` runs the real controller manager with GenericSystem in an
explicitly isolated localhost ROS domain. It checks concurrent position/effort
ownership, activation, deactivation to zero and reactivation.

`gravity_preview URDF BASE TIP GX GY GZ` is an offline numeric tool. The optional
`test/compare_static_gravity.py` compares it to the supplied external static-gravity
library. Neither local numeric checks nor mock checks qualify real robot operation.
