# Static gravity feedforward

Status: **draft**. This package is not selected by any production scope. It does not
enable drives, modify PDO maps, or authorize hardware operation. Physical current
conversion and the startup composition remain blocked by D-004.

The composition and runtime launch switch is `gravity_feedforward` (default
`false`), independent of the arm's `jtc_only` rolling selection. For
`rt_control_scope_runtime.launch.py`, pass
`gravity_feedforward:=false` to keep it off. The artifact generator's opt-in flag
is `--gravity-feedforward`. Requesting `true` currently rejects with D-004 before
any node or bus operation, on mock and real paths alike; it is a reserved runtime
selection until the hardware mapping and lifecycle integration are qualified.
This startup switch is not a controller parameter or a live torque toggle.

`GravityController` reads actual URDF joint positions and writes additive joint
output torque in N·m through `joint/effort`. The trajectory controller retains
`joint/position`. An effort interface must specifically represent the additive
CSP feedforward input; a CST target-torque backend is not interchangeable.

The calculation follows the selected external `static_gravity` convention:
`tau = dU/dq`, including installed fixed tools once. KDL supplies the existing
gravity solver; parameters come from the expanded authoritative Robot Model,
not copied arm constants or a second URDF. Every fixed off-chain descendant is
folded into its owning link, including a massive tool that is a sibling of a
massless TCP. Moving off-chain branches and mimic joints are rejected.

`gravity_in_base` is expressed in `base_link`. Base attitude must match this
vector. This draft has no live IMU/attitude input, additional payload interface,
friction model or inverse-dynamics acceleration term. Changing assembly, payload
or base attitude requires configuration with an appropriate authoritative model
and gravity vector. Position input must already include the approved zero/axis
calibration; the controller applies no additional installation offset.

## Controller parameters

All values are read during configure. There are no usable default safety limits.

| Parameter | Meaning |
| --- | --- |
| `robot_description` | Expanded URDF XML supplied by the integration owner |
| `base_link`, `tip_link` | Ancestor and descendant defining the chain |
| `joints` | Exact moving-joint order from base to tip |
| `gravity_in_base` | Three components, m/s², in the base frame |
| `max_effort_nm` | Positive finite absolute additive torque limits, per joint |
| `max_effort_rate_nm_s` | Positive finite slew limits, per joint; starts at zero |
| `feedback_age_interface` | Existing hardware feedback-age state, milliseconds |
| `max_feedback_age_ms` | Explicit positive feedback-age limit |
| `health_interfaces`, `health_values` | Existing hardware health states and exact ready values, including bus link and every participating slave |

Each joint must expose `position`, `status_word` and an additive `effort` command.
Activation requires complete interfaces, finite positions, Operation Enabled on
every joint, fresh feedback and all supplied health states ready. URDF effort
limits are not used as sustained feedforward limits.

Update uses preallocated buffers and KDL solver storage, with no parameter reads,
logging, network/file operations or locks. It clamps the computed torque and
limits its rate. Invalid positions/results, invalid periods, stale feedback, a
disabled/faulted joint or failed health state zero all commanded torques and
return ERROR. The fault remains latched until lifecycle reactivation. Deactivate,
cleanup, error and shutdown clear commands. Process termination and bus loss
still require the existing hardware shutdown and protection chain; controller
callbacks cannot guarantee delivery after communication has failed.

## Offline validation

Build only this package and its semantic-component dependency in an isolated
workspace using `colcon build --packages-select hw_semantic_components
ctrl_gravity_feedforward`, then `colcon test --packages-select
ctrl_gravity_feedforward`. The tests cover gravity sign, fixed-tool inclusion,
arbitrary gravity, configuration rejection, command binding, startup ramp,
saturation, incomplete/not-ready activation, invalid feedback, fault latching and
lifecycle clearing. Synthetic test limits are not production recommendations.

`gravity_preview URDF BASE TIP GX GY GZ` reads joint vectors (rad) from stdin and
writes torque vectors (N·m) to stdout. It uses no ROS node or hardware access.

The optional `test/compare_static_gravity.py` checks this executable against a
freshly built external `static_gravity` library. It accepts explicit paths and
verifies that the library signature matches its parameter JSON. This external
reference is a verification input, not a copied or runtime-linked dependency.
Local comparison results do not certify actuator current scaling or real-time
timing on the target machine.
