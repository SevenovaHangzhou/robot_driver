# Blocked questions

Only tasks listed under each question are blocked. Unrelated tasks continue in unattended mode.

## BQ-001 — T-005 sanctioned legacy deltas [RESOLVED 2026-07-20]

- Evidence: `/home/kkozia/robot_driver@6bc94cd` has Ti5 RxPDO/TxPDO `0x1600/0x1A00`, extra velocity/effort/mode/gap channels, and no `0x60C2` startup SDOs. Frozen REQ-ECAT-002/004 and the hardware mapping require position-only `0x1601/0x1A01` plus `0x60C2:01=4`, `0x60C2:02=-3`.
- Conflict: REQ-MIG-001 requires `diff_legacy.py` to report 100% against that named legacy baseline, but faithfully implementing the higher-priority frozen mapping necessarily produces semantic differences.
- Decision: the frozen requirements are correct and authoritative; Ti5 uses CSP position control only. T-005 shall apply a narrowly scoped frozen-requirement overlay to `6bc94cd`: change Ti5 PDO assignments to `0x1601/0x1A01`, retain only `0x6040/0x607A` and `0x6041/0x6064`, add `0x60C2:01=4` and `0x60C2:02=-3`, and allow `auto_state_transitions: false` on slave profiles. File restructuring is compared by logical axis. Every other semantic difference remains a gate failure.
- Unblocked scope: BQ-001 no longer blocks T-005. T-005 still depends on T-004/BQ-009.

## BQ-002 — ZeroErr preload and trajectory admission [RESOLVED 2026-07-20]

- Evidence: local upstream `EcCiA402Drive` writes the target-position default only when `mode_of_operation_display_ != MODE_NO_MODE` and does not gate `0x000F` on a completed write. Frozen ZeroErr TPDO has no `0x6061`; the authoritative fork solved this with a source patch, while AMB-007 requires a zero-patch target and an `enable_manager` that claims only `control_word`/`status_word`.
- Superseding decision: implement two independent safeguards. First, actuator initialization automatically reads raw `0x6064`, writes the same raw value to `0x607A` before permitting `0x000F`, and holds that position while no trajectory command exists; the reviewed minimal preload gate from the authoritative ICube fork is authorized for this purpose. Motion does not send a preparatory hold FJT. Second, the existing `dual_arm_jtc` performs a `trajectory_start_consistency_check` directly in its FJT goal-validation path; this is not an autonomy or hardware-driver responsibility.
- Frozen proximity tolerance: for each of the 13 EtherCAT joints, `abs(first_point - actual_position) <= 0.017453292519943295 rad` (1 degree). For updown, `abs(target - actual_position) <= 0.05 m`.
- Frozen feedback freshness: EtherCAT joint position feedback must be no older than `500 ms` at JTC admission time; stale or missing feedback rejects the goal. BQ-014 later withdraws the separate updown `500 ms` feedback-age admission gate because the user accepts the continuously updating PDO behavior. The updown `0.05 m` position-consistency requirement remains in force pending BQ-015's exact single-point semantics.
- Frozen responsibility/boundary: modify the existing JTC minimally; do not add an admission package, node, proxy, internal controller, or Action rename. The public endpoint remains `/dual_arm_jtc/follow_joint_trajectory`. Goal validation rejects missing-joint, stale-feedback, or first-point-proximity failures and reports the offending joint, error, and feedback age. This follows system architecture §8.6 (`FollowJointTrajectory: motion -> rt-control`) and §8.2 (controller plugins reside in the rt-control container). Updown remains on its independent command path; its equivalent check uses the frozen `0.05 m`/`500 ms` values and is finalized with that path.
- Unblocked scope: BQ-002 no longer blocks T-010. The preload gate provides exact pre-enable acknowledgement and ZeroErr command pass-through without mapped `0x6061`.
- Blocked scope: T-010 safety completion and T-013 enable authorization remain blocked only on those exact upper-layer contract details. Configuration and non-enable tasks continue.

## BQ-003 — CANopen operation-mode command ownership [RESOLVED 2026-07-20]

- Evidence: the exact design skeleton requires `operation_mode: 1/3/3` in `bus.yml`, but pinned ros2_canopen Humble `Cia402System` does not consume that key. It changes mode only through edge-triggered `position_mode_cmd`/`velocity_mode_cmd` hardware command interfaces.
- Conflict: REQ-CAN-002 freezes PP/PV modes, but no specified controller owns the three mode-command edges or confirms them before target commands.
- Decision: operation-mode selection belongs to the CANopen hardware activation flow, not the updown or diff-drive controllers. Node 1 uses Profiled Position (`0x6060=1`) and Nodes 2/3 use Profiled Velocity (`0x6060=3`), with `0x6061` confirmation. Upper controllers do not pulse mode command interfaces themselves. BQ-005 subsequently supersedes the earlier hardware-specific ordering: mode/state transitions follow the unmodified upstream ros2_canopen standard implementation, and the drive configuration is adapted to it.
- Unblocked scope: BQ-003 no longer blocks T-007 mode wiring. T-014 activation still depends on BQ-005's implementation-vehicle decision.

## BQ-004 — Updown 0x6081 command ownership and ordering [RESOLVED 2026-07-20]

- Evidence: the EDS maps 0x6081 in RPDO2, and ros2_canopen exposes it only through generic `tpdo/index`, `tpdo/subindex`, `tpdo/data`, `tpdo/ons` one-shot interfaces. Its typed velocity interface means a velocity-mode target, not PP profile velocity.
- Conflict: AMB-015 requires each updown move to carry target position plus 0x6081 maximum velocity, but the specification does not assign ownership or ordering of the generic PDO write relative to the position command.
- Decision: every updown move supplies a target position and that move's maximum velocity. The updown controller owns those two motion-intent values; it does not misuse the normal velocity command interface (`0x60FF`). The CANopen execution path writes `0x6081` through the already mapped PDO before submitting `0x607A`. BQ-010 subsequently removes the former SDO readback/confirmation requirement; no readback failure gate remains. BQ-005 supersedes the fixed `0x003F`/next-cycle `0x000F` rule: set-point triggering and clearing follow the unmodified upstream Profiled Position bit-12 acknowledgement handshake.
- PP clarification (2026-07-20): statusword bit 12 is set-point acknowledgement, not a velocity field. Per-move profile velocity is the separate `0x6081` object; profile acceleration/deceleration use standard `0x6083/0x6084`. Use the drive's standard PP profile generator for acceleration/deceleration and do not implement a custom ramp. The approved numeric `0x6083/0x6084` values remain commissioning `TBD` and must be read back/verified rather than guessed.
- Unblocked scope: BQ-004 no longer blocks the ownership/order semantics. T-007/T-014 concrete wiring follows the implementation vehicle selected in BQ-005.

## BQ-005 — CANopen frozen activation/PP semantics versus stock Motor402 [RESOLVED 2026-07-20]

- Evidence: stock `Motor402::handleInit()` enables before PP/PV preload, uses an unconditional fault-reset arm, and does not implement the frozen retry/deadline sequence. Stock `ProfiledPositionMode` waits on statusword bit 12 and does not guarantee the required one-cycle `0x003F` then `0x000F` pulse.
- Conflict: these behaviors are not equivalent to REQ-CAN-004/005, and bus configuration alone cannot change them.
- Superseding decision: use the official pinned ros2_canopen Humble implementation unchanged and adapt the LD2 drive configuration toward its standard CiA402 behavior. Standard PP set-point acknowledgement through statusword bit 12 replaces the former fixed one-cycle pulse rule. Upstream mode/state initialization replaces the former hardware-specific activation ordering and timing where they differ. Do not create a private ros2_canopen patch set or replacement SystemInterface for these behaviors.
- Narrow exception: BQ-006 later authorizes an interface-exposure-only dependency patch for the existing EMCY callback and master broadcast NMT Stop operation. That exception does not authorize any change to CiA402 activation, mode/state transitions, Profiled Position behavior, or protocol semantics.
- Required deliverable: before drive reconfiguration/commissioning, produce a drive-adaptation mapping table covering every required OD/PDO/status behavior, current value/evidence, target standard behavior, proposed drive-side change, persistence method, and verification. It must explicitly cover `0x6040/0x6041` PP handshake bits, `0x6060/0x6061`, `0x607A/0x6064`, `0x6081`, `0x6502` Homing advertisement/automatic-init consequence, safe no-target hold behavior, and relevant PDO/heartbeat/EMCY exposure. Unknown vendor settings remain TBD rather than guessed.
- Unblocked scope: no code-design blocker remains for choosing the CANopen implementation vehicle. T-014 production activation remains gated by the approved drive-side mapping, readback, and hardware verification.

## BQ-006 — CANopen communication/fault handling after watchdog removal [RESOLVED 2026-07-20]

- Superseding decision: delete the standalone `rt_watchdog` package and do not implement any of its former four centralized inputs: `/heartbeat/motion`, `/heartbeat/autonomy`, a separate 4000 ms CANopen-heartbeat age, or a separate 3000 ms updown-PDO age. These former AMB-009/REQ-SAFE-001 timing requirements are explicitly superseded.
- Approved normal no-command behavior: both track commands remain continuously zero; updown automatically holds its current position.
- Approved CANopen boundary: the unmodified upstream ros2_canopen/Lely implementation owns CANopen heartbeat consumption, NMT error control, EMCY receipt, and their diagnostics. Do not add a raw-SocketCAN observer or reconstruct heartbeat/EMCY protocol handling elsewhere.
- Approved track coupling: a heartbeat timeout or EMCY from either track must stop both tracks.
- Approved heartbeat mechanism (2026-07-20): use upstream Lely `master.heartbeat_consumer: true` and `master.stop_all_nodes: true`, with left/right track nodes marked `mandatory: true`. A heartbeat error-control event on either track broadcasts NMT Stop to every CANopen node; the user explicitly approved that updown also enters NMT Stopped. Updown is not itself marked mandatory, so this decision does not make an updown heartbeat fault a group-stop trigger.
- Approved EMCY behavior (2026-07-20): on a track EMCY, use ros2_canopen's EMCY event and request the same all-node NMT Stop; no raw CAN parsing, new watchdog, or reconstructed EMCY protocol is allowed.
- Integration audit/correction: `NodeCanopenBaseDriver::register_emcy_cb()` is public at the lower node-interface layer, but the pinned `Cia402Driver` facade used by `Cia402System` does not forward that method. `Cia402System` registers only NMT/RPDO callbacks, its initialization hook is private, and the pinned master implementation does not instantiate the `set_nmt` service claimed by its documentation. Consequently the previously described no-patch callback hookup is not available through the actual public `Cia402System` surface.
- Authorized implementation exception (2026-07-20): a narrowly scoped patch to the pinned ros2_canopen dependency may expose the existing EMCY callback through the `Cia402Driver`/`Cia402System` integration and expose the existing master broadcast NMT Stop operation. The callback applies the broadcast only to EMCY events from left/right track node IDs. The patch must not change the CiA402 state machine, heartbeat/EMCY parsing, activation behavior, Profiled Position implementation, or any protocol semantics; it must not add a new package/node or raw-CAN observer.
- Unblocked scope: BQ-006 no longer blocks T-011/T-012 implementation. T-014 remains gated by the dependency patch tests, generated-DCF inspection, drive-adaptation mapping/readback, and hardware validation.

## BQ-007 — Docker Compose frozen fields versus build/version wiring [RESOLVED 2026-07-20]

- Corrected evidence: `docker/compose.yaml` has not been created; only `docker/rt-control/.gitkeep` exists. The frozen design fragment specifies `build.context: .`, Dockerfile `docker/rt-control/Dockerfile`, and volume `./docker/cyclonedds.xml`. If that fragment is created under `docker/compose.yaml` and invoked normally, those relative paths resolve from `docker/` and are wrong unless Compose is explicitly given the repository root as project directory. The same spec requires `versions.env` to be the single IgH version source and the image tag to equal the git SHA; normal Compose wiring needs `build.args` and `image` fields absent from the frozen field list.
- Conflict: either editing the Compose field set or relying on an external wrapper/build command is a deployment-contract choice not covered by the specification. The unresolved T-009 CPU set also cannot be committed as a literal.
- Decision: retain every frozen runtime security/transport field, and authorize only the additional reproducible-build wiring: `build.args` for `IGH_VERSION`, `image` with a git-SHA tag supplied by the wrapper, and environment-substituted `cpuset` so no CPU number is invented. Invoke Compose from the repository root with `--project-directory . --env-file versions.env -f docker/compose.yaml`; the wrapper derives the image tag from the current commit. No `privileged` field or broader device/capability permission is authorized.
- Unblocked scope: BQ-007 no longer blocks T-008. Container-vs-host IgH version comparison and the concrete CPU set remain T-009/T-014 environment validations.

## BQ-008 — T-003 named URDF feature source is unavailable [RESOLVED 2026-07-20]

- Corrected evidence: the unauthenticated GitHub page returns 404 because the repository/ref requires authentication. Using the workstation's authenticated GitHub account, `feature/joint-naming-unify-underscore-20260718` exists at commit `8297e386b3fa8e8184820f7de5c91226c726bd94`, and `ros2_ws/src/alfa_robot_description` contains its package manifest, CMake, config, launch, meshes, tests, and URDF directories.
- Decision: freeze commit `8297e386b3fa8e8184820f7de5c91226c726bd94` and its `ros2_ws/src/alfa_robot_description` subtree as the T-003 source explicitly supplied by the user. Do not substitute the older local package or infer renames.
- Unblocked scope: BQ-008 no longer blocks T-003. The source must be fetched/read-only by exact commit during implementation; no remote write or push is authorized.

## BQ-009 — T-004 slave-profile count and filename mapping conflict [RESOLVED 2026-07-20]

- Evidence: T-004 requires exactly 10 `slaves/*.yaml`, while spec §4 names `{zeroerr_j1..j5,left_j6,right_j6,ti5_j2,ti5_j3,turn}.yaml`. Expanding that set labels J2/J3 as both ZeroErr and Ti5, contradicting the hardware authority (J2/J3 are Ti5 only). The authoritative xacro actually uses eight profiles: ZeroErr J1/J4/J5/left-J6/right-J6/turn and Ti5 J2/J3; its ninth source file is an unused generic `joint6.yaml`. Its Ti5 J2 and J3 files are distinct but each is shared unchanged by left and right axes.
- Conflict: retaining shared profiles yields eight active files; splitting both Ti5 profiles per physical axis yields ten, but that changes the specified filename set and duplicates identical stored configuration. Retaining or repurposing the unused generic J6 profile would create an unreferenced or hardware-inapplicable artifact. The specification does not choose among these structures.
- Decision: authorize the eight active profiles proven by the authoritative xacro and supersede the erroneous "exactly 10 files" constraint. The frozen mapping is: left/right J1 -> `zeroerr_j1.yaml`; left/right J2 -> `ti5_j2.yaml`; left/right J3 -> `ti5_j3.yaml`; left/right J4 -> `zeroerr_j4.yaml`; left/right J5 -> `zeroerr_j5.yaml`; left J6 -> `left_j6.yaml`; right J6 -> `right_j6.yaml`; turn -> `turn.yaml`. Do not migrate the unused generic `joint6.yaml`, and do not create `zeroerr_j2.yaml` or `zeroerr_j3.yaml` aliases.
- Unblocked scope: BQ-009 no longer blocks T-004 or its dependent tasks. Numeric content and logical-axis comparison remain governed by the frozen hardware export and the narrowly sanctioned BQ-001 overlay.

## BQ-010 — Updown 0x6081 “write confirmation” has no stock PDO acknowledgement [RESOLVED 2026-07-20]

- Evidence: the frozen mapping sends `0x6081` to Node 1 in mapped RPDO2 (`COB-ID 0x301`), but neither approved updown TPDO carries `0x6081`. Pinned `Cia402System` exposes the generic `tpdo/index`, `tpdo/subindex`, `tpdo/data`, and `tpdo/ons` one-shot command interfaces, so the controller can transmit `0x6081` without remapping. However, that path exports no transmit-result/acknowledgement interface and stock `Cia402System::write()` discards the boolean return from `tpdo_transmit()`.
- Conflict: BQ-004 requires the execution path to write **and confirm** `0x6081` before submitting `0x607A`, while BQ-005 permits no CANopen dependency change except BQ-006's EMCY/NMT-Stop interface exposure. A PDO readback confirmation is impossible with the approved live mapping because `0x6081` is not returned in a TPDO.
- Decision: retain the mapped PDO write of `0x6081` for every new updown move, but remove SDO readback and all device-readback gating. In the same `Cia402System::write()` pass, the generic PDO one-shot is processed before the Profiled Position `set_target()` call, so the required software ordering is `0x6081` transmit request then `0x607A` target request. No extra delay, retry, or acknowledgement mechanism is added.
- Unblocked scope: BQ-010 no longer blocks the updown portion of T-007. T-014 still verifies the live PDO mapping and motion behavior during commissioning, without making runtime motion conditional on `0x6081` readback.

## BQ-011 — No stock controller accepts the approved composite updown command [RESOLVED 2026-07-20]

- Evidence: the approved command contains both target position and per-move `0x6081` maximum velocity and must apply admission checks before commanding motion. Pinned `Cia402DeviceController` does not implement its target service (the code is commented out) and does not claim the `position` command interface. `CanopenProxyController` can send the generic PDO only. The standard position/forward controllers cannot combine the low-level PDO one-shot with the PP position target or enforce their ordering/admission as one operation.
- Conflict: T-007 names an `updown_position` controller but neither the specification nor the package skeleton assigns an implementation vehicle. Running two stock controllers or adding a bridge node would split one approved motion command across independently scheduled paths. Modifying upstream `Cia402DeviceController` would broaden the BQ-005 dependency patch exception.
- Decision: implement a minimal `updown_position_controller` plugin inside the existing `robot_hw_canopen` package. It is a controller-manager-loaded C++ class/library, not a new package, process, standalone node, or proxy. It claims only the updown position command/state plus the existing generic PDO one-shot interfaces and performs the approved admission/order logic in one controller. The external message type/topic and feedback-age source are frozen separately.
- Unblocked scope: the implementation vehicle no longer blocks the updown portion of T-007.

## BQ-012 — External updown command message type is not frozen [RESOLVED 2026-07-20]

- Evidence: the contract requires one event-level command containing target position and per-move `0x6081` maximum velocity, but the specification leaves its carrier to T-007. No standard ROS 2 message directly expresses exactly those two updown motion-intent fields. `Float64MultiArray` encodes them only by positional convention, while `JointTrajectory` introduces trajectory/time semantics that this single PP move does not implement.
- Additional unit evidence: the hardware authority freezes updown position conversion (`m <-> counts`) but does not state the LD2 interpretation or SI conversion of `0x6081`; the EDS declares only an unsigned-32 profile velocity with no unit. Therefore `m/s -> 0x6081 raw` must not be inferred.
- Proposed decision: add `robot_interfaces/msg/UpdownCommand.msg` with `float64 position` (m) and `uint32 profile_velocity_raw` (the exact `0x6081` value), and make it the input type of `updown_position_controller`. The topic name is frozen separately.
- Benefit: field meaning and position unit are explicit; the controller can range-check the raw unsigned value and does not guess an unverified velocity conversion. The contract still hides PDO indices, controlword, and one-shot mechanics.
- Drawback: raw profile velocity exposes one drive-specific quantity to motion/autonomy and is less portable when the updown actuator changes. The consumer must also depend on and rebuild against the existing `robot_interfaces` package; this is a domain-interface change requiring coordination.
- Alternative cost: `Float64MultiArray` avoids a custom message but leaves order and units implicit; `JointTrajectory` reuses a standard type but falsely suggests interpolation/timing support and carries unused fields.
- Decision: add `robot_interfaces/msg/UpdownCommand.msg`; BQ-015 later expands its final field set to `float64 expected_start_position`, `float64 position` (both metres), and `uint32 profile_velocity_raw` carrying the exact unsigned value written to `0x6081`. Do not infer or advertise an SI velocity conversion until verified drive evidence exists.
- Unblocked scope: the message carrier no longer blocks the updown portion of T-007. Its topic name and feedback-age source are frozen separately.

## BQ-013 — External updown command topic name is not frozen [RESOLVED 2026-07-20]

- Evidence: spec §13 explicitly leaves the updown command topic name to T-007. The approved controller instance is `updown_position`, while the public contract should describe the mechanism-independent updown function rather than the controller implementation.
- Proposed decision: freeze the public topic as `/updown/command`, carrying `robot_interfaces/msg/UpdownCommand`. The controller uses its private `~/command` subscription remapped by bringup to that public topic.
- Benefit: the external contract stays stable if the controller class or instance name changes; the functional name is concise and does not expose CANopen/PP details. A private controller subscription still follows normal controller composition and is easy to remap under a robot namespace.
- Drawback: bringup must maintain one explicit remapping, and an absolute topic requires launch-time namespace handling for future multi-robot deployments. Using `/updown_position/command` would remove that remap but couple the domain contract to the controller instance name.
- Decision: freeze `/updown/command` as the public topic carrying `robot_interfaces/msg/UpdownCommand`; bringup remaps it to the `updown_position` controller's private `~/command` subscription. Future multi-robot bringup must apply the robot namespace explicitly rather than silently changing this single-robot contract.
- Unblocked scope: the topic contract no longer blocks the updown portion of T-007.

## BQ-014 — Stock Cia402System cannot prove the approved 500 ms updown-position freshness [RESOLVED 2026-07-20]

- Evidence: BQ-002 requires the updown `0x6064` position feedback used for command admission to be no older than 500 ms. Pinned `Cia402System` exports the cached position value but no PDO receive timestamp, sequence, or age. `/joint_states` republishes that cache periodically even when CAN PDO updates have stopped, and CANopen heartbeat proves node liveness rather than `0x6064` freshness.
- Conflict: the existing interfaces cannot distinguish a stationary but freshly reported position from a stale cached position. Inferring freshness from value changes would falsely reject a stationary axis and falsely accept some repeated traffic. BQ-005 currently permits dependency patches only for BQ-006's EMCY/NMT-Stop exposure.
- Proposed decision: extend the already authorized interface-only ros2_canopen patch to timestamp actual `0x6064` RPDO receipt with a monotonic clock and export a read-only `position_feedback_age_ms` state interface. `updown_position_controller` rejects a command when this age exceeds 500 ms or has never been initialized. Do not alter PDO parsing, mapping, CiA402 state transitions, or PP behavior.
- Benefit: enforces the approved safety gate using the exact transport event; no extra node, polling, SDO, or guessed heartbeat equivalence is introduced. It also distinguishes a valid stationary axis from stale cached data.
- Drawback: this adds another small downstream ros2_canopen patch and state-interface ABI to maintain/rebase and test. Live commissioning must verify that the configured updown TPDO updates the timestamp as expected.
- Alternative cost: dropping the freshness check preserves a more upstream-pure dependency, but a command could be admitted using an arbitrarily old cached position while heartbeat remains healthy.
- Decision: withdraw only the updown `500 ms` feedback-freshness admission gate and do not add a `position_feedback_age_ms` state interface or related ros2_canopen patch. The user accepts that the deployed PDO updates normally. This does not withdraw the EtherCAT JTC's separate 500 ms feedback-freshness check, the updown `0.05 m` position-consistency requirement, live PDO mapping verification, or ros2_canopen heartbeat/EMCY handling.
- Unblocked scope: no feedback-age interface blocks the updown portion of T-007/T-014.

## BQ-015 — The approved two-field updown command has no trajectory initial point for the 0.05 m check [RESOLVED 2026-07-20]

- Evidence: the original 0.05 m rule compares a commanded trajectory's initial point with actual updown position. The approved `UpdownCommand` currently contains only the final PP target and `0x6081` velocity. A single PP target has no separate initial point.
- Conflict: comparing the final target with actual position changes the rule into a maximum move distance of 0.05 m per command, which is not the same as rejecting a stale planned trajectory. Skipping the check would silently remove the approved 0.05 m safeguard.
- Proposed decision: extend `UpdownCommand` with `float64 expected_start_position` (m). On receipt, compare it with actual `0x6064`; reject when the absolute difference exceeds 0.05 m, then execute the separate final `position` target. The message becomes `expected_start_position`, `position`, and `profile_velocity_raw`.
- Benefit: preserves the intended stale-plan/start-state check without limiting a legitimate move to 0.05 m; the rejection can report the expected start, actual position, and error directly.
- Drawback: the motion/autonomy producer must provide a trustworthy expected start and the just-approved message gains a third field. If the producer simply copies current feedback immediately before publishing, the check adds little independent protection.
- Alternative cost: keeping two fields and comparing final target to actual imposes 5 cm incremental moves; removing the check is simpler but abandons the approved updown start-consistency protection.
- Decision: add `float64 expected_start_position` in metres to `UpdownCommand`. Admission requires `abs(expected_start_position - actual_position) <= 0.05 m`; passing that check releases the separate final `position` target. Do not compare the final target to actual position as a 5 cm step-size limit.
- Unblocked scope: the updown start-consistency semantics no longer block T-007.

## BQ-016 — A one-way updown topic has no business-level rejection result [RESOLVED 2026-07-20]

- Evidence: `/updown/command` is a topic, so DDS delivery does not tell the producer whether `updown_position_controller` accepted or rejected the 0.05 m start-consistency check. FJT can return a rejected action goal, but this separate PP path has no action/service response. A log or 1 Hz diagnostic is not correlated to a particular command.
- Upstream capability audit: ros2_canopen provides `canopen_interfaces/srv/COTargetDouble` on a standalone/in-container CiA402 driver's `~/target` endpoint and returns only `bool success` from `Motor402::setTarget()`. It carries neither `0x6081` nor expected start, performs no 0.05 m admission check, supplies no reason/completion result, and would bypass the approved composite controller path. The controller-layer `Cia402DeviceController` declares a target service member but its implementation is commented out. The standard forward-position controller is topic-only and position-only.
- Conflict: the approved behavior says an inconsistent command is rejected, but the external contract does not specify how motion/autonomy learns that outcome. Silently dropping the command could leave the state machine waiting for motion that never started.
- Proposed decision: add a producer-supplied `uint64 command_id` to `UpdownCommand` and publish a numeric, allocation-free `robot_interfaces/msg/UpdownCommandStatus` on `/updown/command_status`. The result echoes `command_id`, reports accepted/rejected plus a fixed reason code, and includes expected start, actual position, and absolute error. `accepted` means only software admission and submission, not physical move completion.
- Benefit: every asynchronous command has a deterministic correlated admission result; rejection is machine-readable without polling logs, diagnostics, or controller internals. Numeric reason codes keep the RT publication bounded and allocation-free.
- Drawback: adds one request field, one custom status message, and one public topic; the producer must allocate unique IDs and handle asynchronous results. It still does not report target-reached completion, which would require a separate contract.
- Alternative cost: diagnostics-only reporting has fewer interfaces but is delayed and cannot reliably correlate a rejection with a particular command; switching to a service/action would give a native response but overturn the already approved topic contract.
- Decision: retain only the `/updown/command` topic. Do not add `command_id`, a command-status message/topic, or a command service. A rejected command is not forwarded to the drive, and motion/autonomy receives no per-command admission result. This explicitly accepts the one-way contract and supersedes the proposed correlated-result mechanism.
- Unblocked scope: no admission-result interface blocks T-007.

## BQ-017 — Overlapping updown PP commands have no frozen policy [RESOLVED 2026-07-20]

- Evidence: `/updown/command` is event-level but the contract does not say what happens when a new command arrives before the prior PP move reaches its target. Upstream `ProfiledPositionMode` asserts the Immediate bit and can accept a different target after the normal bit-12 acknowledgement cycle. The approved controller has no completion/status contract or command queue.
- Conflict: silently choosing overwrite, reject-while-busy, or queue changes observable motion behavior. Reject-while-busy additionally needs a reliable target-reached signal and would be invisible to the producer under BQ-016; queueing needs depth, overflow, cancellation, and stale-start policies not covered by the specification.
- Proposed decision: use latest-valid-command replacement. Every new command independently passes the 0.05 m expected-start check; if it passes, its `0x6081` and final position replace the active target through the upstream Immediate PP behavior. Do not queue. A failed admission leaves the currently active target unchanged.
- Benefit: matches the open-source PP implementation, adds no target-reached patch/queue/state machine, and allows a correctly replanned command to redirect an in-progress move. The existing expected-start check prevents a command planned from a stale position from replacing the target.
- Drawback: a valid new command can change direction or destination before the prior move completes, and the producer receives no explicit notice that replacement occurred. It is unsuitable if commands are intended as an ordered sequence that must all execute.
- Alternative cost: reject-while-busy is simpler conceptually but needs target-reached exposure and silently drops commands; queueing preserves order but introduces unapproved buffering/cancellation semantics and can execute stale motion later.
- Decision: use latest-valid-command replacement with no queue. Every new command independently passes the 0.05 m expected-start check; an accepted command replaces the active `0x6081` and final target through upstream Immediate PP behavior. A rejected command leaves the currently active target unchanged.
- Unblocked scope: overlapping-command semantics no longer block T-007/T-014.

## BQ-018 — `/updown/command` QoS is not frozen [RESOLVED 2026-07-20]

- Evidence: the interface is an event-level command topic with latest-valid-command replacement and no acknowledgement. QoS therefore determines whether a command may be lost, accumulated, or replayed after controller restart. The specification does not define reliability, history depth, or durability.
- Proposed decision: use `RELIABLE`, `KEEP_LAST(1)`, `VOLATILE` QoS for both producer and controller subscription. Do not use transient-local durability or a queue depth greater than one.
- Benefit: reliable delivery reduces silent command loss; depth one matches latest-command replacement and prevents a backlog of stale moves; volatile durability prevents a retained command from replaying when the controller restarts or reconnects.
- Drawback: reliable DDS can deliver a command later after transient network congestion rather than dropping it immediately. The 0.05 m expected-start gate limits but does not eliminate that delayed-command risk, and reliable delivery still does not provide business-level acceptance.
- Alternative cost: best-effort minimizes retransmission/delay but can silently lose the only event command; transient-local helps late joiners but can replay an old motion command, which is unsafe for this interface.
- Decision: use matching `RELIABLE + KEEP_LAST(1) + VOLATILE` QoS on the `/updown/command` publisher and controller subscription. Do not retain/replay commands or accumulate more than the latest sample.
- Unblocked scope: the updown input QoS no longer blocks T-007.

## BQ-019 — Humble JTC has no configurable/subclass hook for the approved FJT admission check [RESOLVED 2026-07-20]

- Evidence: BQ-002 requires the existing `/dual_arm_jtc/follow_joint_trajectory` goal path itself to enforce the 13-joint first-point check, with no wrapper/proxy/package or action rename. In Humble `JointTrajectoryController`, `goal_received_callback()` and `validate_trajectory_msg()` are protected but non-virtual, and `on_configure()` binds the action server directly to the base-class callback. No parameter/plugin hook can inject an additional goal validator. The installed binary is 2.52.1; the supplied local Humble upstream reference is `ros2_controllers@cbcf66218ff43353f9fb5fe7a2c33f458d578d73` (package version 2.53.2).
- Clarification (2026-07-20): the user was referring to the admission check agreed in this conversation, not a controller found in a repository. The frozen behavior is: when `dual_arm_jtc` receives an FJT goal, compare all 13 actual joint positions with the trajectory's first point and reject the whole goal if any absolute error exceeds exactly `0.017453292519943295 rad` (1 degree). This logic belongs inside the existing JTC goal-admission path and is not a separate controller/package/proxy.
- Historical-controller audit: `/home/kkozia/robot_driver_review@07c87e5` separately configures `dual_arm_command_permit_controller`, but it is a standard `ForwardCommandController` publishing alternating lease tokens to 13 custom `command_permit` hardware interfaces. It never reads a `FollowJointTrajectory` goal, never compares its first point, and cannot reject the action goal. It also exists only in the later reference checkout, not the authoritative `robot_driver@6bc94cd`, and depends on extensive non-authoritative EtherCAT-plugin gate changes. The authoritative plugin's separate `position_command_sync_tolerance_=0.0175` check only controls the first hardware-command takeover after preload; a JTC trajectory can first interpolate through the current position and then continue to a distant point, so that check is not an FJT start-consistency rejection either.
- Conflict: the approved check cannot be implemented by controllers.yaml or a subclass while retaining the original action server. It requires a small patch to the upstream JTC source, but the current dependency manifest does not source-build ros2_controllers and no exact patched version is frozen.
- Proposed decision: pin the supplied Humble upstream commit `cbcf66218ff43353f9fb5fe7a2c33f458d578d73`, carry a narrowly scoped patch inside the existing `joint_trajectory_controller` goal-validation callback, and source-build that package as an overlay. Preserve the standard controller class, action endpoint, interpolation, execution, cancellation, and result behavior; add only parameterized first-point/freshness validation and focused tests.
- Benefit: implements the check at the actual FJT admission point, preserves all normal open-source JTC behavior and the public action name, and avoids the separate package/node/proxy previously rejected by the user. The dependency and patch are reproducibly pinned.
- Drawback: replaces the installed 2.52.1 binary with source-built 2.53.2 plus a downstream patch; the patch must be maintained/rebased and the upstream version difference must be regression-tested. Checking out ros2_controllers also increases dependency/build footprint even if only JTC is selected for build.
- Alternative cost: a wrapper/proxy avoids patching upstream but violates BQ-002; subclassing does not intercept the bound callback; omitting the check violates the approved requirement.
- Decision: use the supplied official Humble `ros2_controllers` source at exact commit `cbcf66218ff43353f9fb5fe7a2c33f458d578d73` (`joint_trajectory_controller` 2.53.2) and apply the minimal source-overlay patch described above. Do not modify `/opt/ros/humble`, the read-only legacy baseline, the controller/action name, or unrelated JTC execution behavior.
- Unblocked scope: the JTC source/version vehicle no longer blocks the 13-axis FJT admission implementation. The exact EtherCAT feedback-freshness source is frozen separately.

## BQ-020 — Existing EtherCAT `read()` cannot prove the approved 500 ms JTC feedback freshness [RESOLVED 2026-07-20]

- Evidence: the pinned ICube `EthercatDriver::read()` returns `OK` whenever `EthercatBusManager::read()` does not return `kError`, while the bus manager returns `kCompleted` immediately after `EcMaster::readData()`. `readData()` calls `processData()` even when the EtherCAT domain Working Counter is `ZERO` or `INCOMPLETE`; `checkDomainState()` only logs and caches that state. Therefore a successful controller-manager read cycle can repeatedly expose an old process image. The driver exports joint position values but no receive timestamp, sequence, Working Counter validity, or age interface.
- Conflict: BQ-002 requires the 13-axis FJT admission path to reject feedback older than 500 ms. Neither the JTC callback time nor the last cached-position access time proves that a new valid EtherCAT frame arrived. Treating an unchanged position as stale would also reject a stationary but healthy robot.
- Proposed decision: make the already required ICube source overlay export one read-only domain-level state interface named `ethercat_domain/process_data_age_ms`. Update its monotonic timestamp only after a receive cycle whose domain state is `EC_WC_COMPLETE`; initialize it as non-finite/stale. The patched JTC 2.53.2 claims this one extra state interface, snapshots it with the 13 actual positions in its normal update path, and rejects a new FJT goal when the snapshot is missing/non-finite or older than 500 ms. Do not create a node/watchdog, infer freshness from position changes, or turn an incomplete Working Counter directly into a hardware-component error.
- Benefit: the gate measures an actual complete EtherCAT process-data event, accepts healthy stationary feedback, adds only one scalar for the whole synchronized domain, and keeps a transient bus issue scoped to rejecting new trajectories rather than forcing an unapproved hardware lifecycle transition.
- Drawback: this adds a small downstream ICube interface patch in addition to the JTC patch, and the new state-interface name/configuration must be maintained. Because freshness is domain-level, any configured slave that makes the domain Working Counter incomplete causes admission of the whole 13-axis trajectory to fail; it does not identify one stale joint.
- Alternative cost: withdrawing the 500 ms gate avoids the extra interface/patch but permits a new FJT goal to be admitted against an arbitrarily old cached EtherCAT position. Returning hardware `ERROR` after 500 ms avoids the JTC interface but changes controller-manager lifecycle and active-motion fault behavior beyond the approved admission-only requirement.
- Decision: add the single read-only domain-level state interface `ethercat_domain/process_data_age_ms` exactly as proposed. This interface is used only by the FJT admission check; it does not create a watchdog or change hardware-component lifecycle/fault behavior.
- Unblocked scope: the freshness portion of the JTC overlay may proceed with the 500 ms threshold.

## BQ-021 — A rejected ROS 2 action goal cannot return the approved detailed FJT failure result [RESOLVED 2026-07-20]

- Evidence: `rclcpp_action::GoalResponse::REJECT` produces a SendGoal response containing only `accepted=false` (plus the protocol timestamp); the client receives a null goal handle and no action result. Although `control_msgs/action/FollowJointTrajectory` defines `INVALID_GOAL`, `INVALID_JOINTS`, and `error_string`, those fields are available only after the server first accepts the goal and later terminates it. Stock JTC already rejects invalid goal messages in `goal_received_callback()` and reports details only in its server log.
- Conflict: BQ-002 says a stale or first-point-inconsistent goal is rejected and also reports the offending joint, position error, and feedback age. Strict goal rejection and returning those details through the existing FJT action result are mutually exclusive under the ROS 2 action protocol.
- Proposed decision: preserve strict admission rejection. Return `GoalResponse::REJECT` before any trajectory becomes active, and emit one structured `ERROR` log containing the rejection reason, offending joint, absolute position error in radians, 1-degree limit, and domain feedback age in milliseconds. The FJT client sees only that the goal was rejected. Do not add another public status topic/service.
- Benefit: an invalid trajectory is never represented as accepted or briefly active; this matches stock JTC validation semantics, preserves the existing endpoint, and gives local commissioning/diagnostics a precise reason without another interface.
- Drawback: motion/MoveIt cannot obtain the detailed reason programmatically from the action response and receives only generic goal rejection; correlation relies on the contemporaneous server log.
- Alternative cost: accepting then immediately aborting with `INVALID_GOAL` and a detailed `error_string` gives the client a machine-readable result, but changes the contract from rejection to accepted-then-failed and enters the accepted-goal callback path. A separate result topic/service preserves strict rejection plus machine-readable details but adds a new public interface and correlation contract.
- Decision: preserve strict admission rejection exactly as proposed. Emit one structured JTC `ERROR` log with the rejection reason and available offending-joint/error/limit/feedback-age fields; do not accept-then-abort and do not add a public result interface. Motion/MoveIt receives only the standard rejected-goal indication.
- Unblocked scope: FJT rejection reporting no longer blocks the JTC patch.

## BQ-022 — Stock ros2_canopen diagnostics do not provide the legacy CANopen age/counter fields [RESOLVED 2026-07-20]

- Evidence: the pinned upstream `NodeCanopen402Driver` uses `diagnostic_updater` and publishes the native keys `device_state`, `nmt_state`, `emcy_state`, `cia402_mode`, and `cia402_state`. The EMCY text contains the received emergency error code/register/manufacturer bytes. The implementation does not timestamp heartbeat receipt, export `heartbeat_age_ms`, or count SDO timeouts, and its CiA402 diagnostic callback has no `sdo_timeout_count` field. Lely's heartbeat error-control and the approved all-node stop still operate independently of these missing observability fields.
- Conflict: the original T-012 diagnostics table requires `heartbeat_age_ms`, `last_emcy_code`, and `sdo_timeout_count` for every CANopen node, but BQ-006 supersedes the separate heartbeat-age watchdog and directs communication-fault handling to the upstream ros2_canopen/Lely implementation. Recreating exact ages/counters requires more downstream instrumentation than the already approved callback/broadcast exposure patch.
- Proposed decision: use ros2_canopen's native per-node diagnostics as the CANopen diagnostic authority. Preserve its device/NMT/EMCY/CiA402 fields and the EMCY error data, but remove the requirements for custom `heartbeat_age_ms` and `sdo_timeout_count`; do not reconstruct them in `rt_diagnostics`. Normalize/aggregate the native statuses under the rt-control diagnostic tree without changing their underlying safety decisions.
- Benefit: keeps communication monitoring and diagnostics aligned with the selected open-source implementation, avoids another protocol-timing/counter patch, and prevents duplicated fault state from disagreeing with Lely's actual decisions.
- Drawback: operators cannot see a continuously increasing heartbeat age or an accumulated SDO-timeout counter in rqt; they see the resulting native NMT/device/EMCY state instead. The diagnostic key set differs from the original frozen table.
- Alternative cost: instrumenting ros2_canopen to expose exact heartbeat timestamps and SDO-timeout counters preserves the old table but expands the maintained dependency patch and duplicates information not used by the approved safety path.
- Decision: use ros2_canopen's native per-node diagnostics as proposed. Preserve and aggregate its device/NMT/EMCY/CiA402 fields and EMCY error data; remove the custom `heartbeat_age_ms` and `sdo_timeout_count` requirements and do not reconstruct them in `rt_diagnostics`.
- Unblocked scope: the CANopen diagnostics row of T-012 may proceed using the upstream-native fields; heartbeat/EMCY stop behavior remains governed by BQ-006.

## BQ-023 — Per-slave EtherCAT diagnostics cannot cover unconfigured ring positions 0 and 13 [RESOLVED 2026-07-20]

- Evidence: the frozen topology has 15 responding ring positions `0..14`, but only positions `1..12` and `14` are configured motion slaves. ICube obtains per-slave `ec_slave_config_state_t` only from the `ec_slave_config_t` handles created by `ecrt_master_slave_config()` for configured modules. Positions 0 and 13 have no module/config handle, and REQ-ECAT-010 explicitly forbids guessing their identities. The master-level API still reports the aggregate `slaves_responding` count.
- Conflict: the original T-012 table says `ethercat/slave_<pos>` diagnostics for every slave, while the approved configuration can truthfully publish per-slave AL/online/operational state only for the 13 known motion positions before T-013 commissioning identifies positions 0 and 13.
- Proposed decision: publish per-slave runtime diagnostics for the 13 configured motion positions only, and use `ethercat/master.slaves_responding == 15` plus the T-013 preflight `ethercat slaves -v` archive to cover the two unconfigured topology members. Do not create guessed/passive configurations for positions 0 and 13 and do not poll the privileged `ethercat` CLI continuously from `rt_diagnostics`. Add per-position diagnostics for 0 and 13 only in a later frozen hardware revision if their identities and supported monitoring vehicle are explicitly approved.
- Benefit: reports every state ICube actually owns, preserves the no-guess hardware rule, avoids changing the EtherCAT domain/configuration for non-motion members, and still detects a missing member through the aggregate responding count.
- Drawback: during runtime, a drop from 15 responders cannot be attributed specifically to position 0 versus 13 from the diagnostic tree; detailed attribution requires the commissioning CLI/archive. The emitted per-slave set is 13 entries rather than the original table's implied 15.
- Alternative cost: configuring diagnostic-only slaves after guessing or prematurely freezing vendor/product IDs violates REQ-ECAT-010; invoking and parsing `ethercat slaves` at 1 Hz adds privilege, process creation, and a second state source outside ICube.
- Decision: publish per-slave runtime diagnostics only for configured positions `1..12` and `14`, and cover the full ring with `ethercat/master.slaves_responding == 15` plus the T-013 `ethercat slaves -v` preflight archive. Do not guess/configure positions 0 and 13 or poll the privileged CLI from `rt_diagnostics`.
- Unblocked scope: the EtherCAT per-slave coverage of T-012 may proceed for the 13 configured motion positions and the aggregate master count.

## BQ-024 — EtherCAT diagnostic snapshot transport and WC-error counting are not frozen [RESOLVED 2026-07-20]

- Evidence: ICube caches the current EtherCAT master state, domain Working Counter state, and 13 configured slave states internally, but exports none of them as ROS diagnostics or ros2_control interfaces. The existing Joint State Broadcaster claims all available read-only state interfaces and publishes arbitrary non-motion interfaces on `/dynamic_joint_states`; only prefixes with position/velocity/effort enter `/joint_states`. BQ-020 already authorizes the domain process-data age interface.
- Conflict: `rt_diagnostics` is a separate node and cannot access ICube's private C++ objects. Publishing directly from the hardware plugin, exporting a stable read-only state-interface ABI, or polling a CLI are materially different designs. The frozen `wc_error_count` also does not define whether to count incomplete cycles or only state transitions.
- Proposed decision: extend the ICube overlay with these numeric read-only state interfaces: `ethercat_domain/process_data_age_ms`; `ethercat_master/link_up`; `ethercat_master/slaves_responding`; `ethercat_master/wc_error_count`; and `ethercat_slave_<pos>/al_state` for positions `1..12` and `14`. Keep each joint's existing `status_word` as the CiA402 source. JSB publishes the synthetic interfaces on `/dynamic_joint_states`, and `rt_diagnostics` converts the snapshots to the approved 1 Hz tree. Define `wc_error_count` as the cumulative number of activated read cycles whose domain state is not `EC_WC_COMPLETE`, initialized to zero on process start; diagnostics warns while the counter is increasing rather than forever after a historical error.
- Benefit: keeps all bus reads in ICube, gives both JTC and diagnostics a single transport-validity source, uses the standard ros2_control/JSB path, leaves `/joint_states` unchanged, and makes a one-second WC outage correspond to roughly 250 bad process-data cycles rather than one ambiguous transition.
- Drawback: adds a maintained state-interface ABI and extra fields to `/dynamic_joint_states`; `rt_diagnostics` depends on JSB being active. Counting bad cycles makes the number increase quickly during an outage and does not equal the number of distinct outage episodes.
- Alternative cost: counting only COMPLETE-to-incomplete transitions hides outage duration and repeated bad frames; publishing diagnostics directly from the hardware plugin adds ROS presentation/timers to the bus component; CLI polling adds privilege and a second state source.
- Decision: export the proposed read-only state-interface set through ICube and JSB `/dynamic_joint_states`, with `rt_diagnostics` producing the 1 Hz diagnostic tree. Define `wc_error_count` as one increment for every activated read cycle whose domain state is not `EC_WC_COMPLETE`, reset on process start; WARN only while the counter increases.
- Unblocked scope: the EtherCAT snapshot transport and WC counter semantics no longer block T-012 or the ICube overlay.

## BQ-025 — Humble controller_manager does not expose the frozen runtime overrun counter [RESOLVED 2026-07-20]

- Evidence: the pinned/local Humble ros2_control source (`controller_manager` 2.53.1) calculates the measured loop period and runs `read() -> update() -> write() -> sleep_until()`, but does not count deadline misses or publish control-loop statistics. Its changelog records that controller-manager diagnostics were backported and then reverted in 2.26.0 because the change was ABI-breaking. No service, topic, state interface, or public getter supplies the frozen `overrun_count` to `rt_diagnostics`.
- Conflict: the original T-012 table requires a runtime `cyclic/overrun_count`, while retaining it requires source-building and patching core ros2_control/controller_manager in addition to the already approved JTC and ICube overlays. Inferring 250 Hz loop overruns from JSB's 100 Hz topic is invalid, and no overrun threshold/tolerance is frozen.
- Proposed decision: remove the runtime `cyclic/overrun_count` diagnostic row for this Humble implementation and keep the already required host `cyclictest`, 250 Hz configuration check, realtime scheduling/affinity startup logs, and commissioning stop criterion (`cyclictest > 100 us`). Do not infer a counter from `/joint_states` or `/dynamic_joint_states`, and do not patch controller_manager solely for this field.
- Benefit: keeps the core ros2_control runtime upstream-pure, avoids a high-blast-radius dependency overlay and an invented threshold, and retains the explicit host realtime acceptance test that is already the hardware gate.
- Drawback: after commissioning there is no persistent ROS diagnostic showing application-loop deadline misses; operators must use logs and rerun the host timing test when investigating runtime scheduling problems.
- Alternative cost: a controller_manager runner patch can count exact scheduled-deadline misses and publish them, but requires freezing a ros2_control version, counter/reset/threshold semantics, an RT-safe export path, and regression testing the core control loop.
- Decision: remove the runtime `cyclic/overrun_count` diagnostic row for the Humble implementation. Retain the 250 Hz configuration check, realtime scheduling/affinity startup evidence, host `cyclictest`, and the `>100 us` commissioning stop criterion; do not patch controller_manager or infer a counter from published state topics.
- Unblocked scope: the absence of a runtime cyclic counter no longer blocks T-012; host timing verification remains part of T-009/T-015.

## BQ-026 — Recovery after the approved CANopen all-node NMT Stop is not defined [RESOLVED 2026-07-20]

- Evidence: BQ-006 configures Lely to broadcast NMT Stop when a mandatory track heartbeat fails and adds the same broadcast for a track EMCY. The pinned master does not instantiate the documented `set_nmt` service. `Cia402System` exposes per-node `nmt/start` command interfaces, but no approved controller or service claims them. The deleted watchdog's former five-second automatic recovery no longer exists, and BQ-006 does not define a replacement recovery transition.
- Conflict: silently auto-starting nodes after communication returns can resume a system whose original EMCY/heartbeat cause is not cleared, while leaving all nodes stopped forever without a documented recovery path makes the approved group stop operationally incomplete.
- Proposed decision: make the group stop fail-latched for this phase. After correcting and acknowledging the physical/communication fault, recover by restarting the rt-control container (or equivalently performing the full controller-manager hardware cleanup/configure/activate sequence), which reruns the standard ros2_canopen boot, mode selection, initialization, and zero/hold preload. Do not auto-restart nodes and do not add a partial NMT-Start service/controller in the current scope.
- Benefit: recovery always passes through the same verified cold-start initialization and safe zero/hold preload, cannot silently resume old commands, and adds no new safety-critical service or partial re-enable state machine.
- Drawback: recovery is manual and disruptive: all rt-control controllers and all three CANopen nodes restart, active goals/commands are lost, and downtime is longer than a targeted NMT recovery.
- Alternative cost: a dedicated recovery service could validate all three node states, broadcast/start nodes, rerun CiA402 initialization, and report failures without restarting the container, but it requires a new public interface, authorization rules, concurrency semantics, and another tested safety FSM. Automatic restart is simpler operationally but can re-enable after an uncleared fault.
- Decision: make the CANopen group stop fail-latched. After correcting and acknowledging the fault, recover only through an rt-control container restart or the equivalent full hardware cleanup/configure/activate sequence; rerun standard boot/mode/init and zero/hold preload, never replay old commands, and add no automatic or partial NMT recovery interface in this phase.
- Unblocked scope: the post-fault operating procedure is frozen; T-011/T-014 must verify the full-restart recovery path.

## BQ-027 — NMT Stop alone does not guarantee mechanical track stop after communication loss [RESOLVED 2026-07-21]

- Evidence: CANopen NMT Stop changes the node communication state and disables normal PDO processing; it does not by itself standardize the drive's mechanical deceleration or clear a previously latched nonzero `0x60FF`. If a track has actually lost CAN communication, the master cannot deliver a later zero-speed or `0x0002` command to that node. The current bus configuration has no master heartbeat producer, so each slave's configured `heartbeat_consumer: true` currently creates no effective drive-side supervision. The LD2 EDS exposes heartbeat consumer object `0x1016` and quick-stop option `0x605A`, but not the vendor `0x5010` watchdog candidate; the exact loss-reaction configuration is not frozen.
- Conflict: BQ-006's master-side heartbeat/EMCY detection and all-node NMT Stop can coordinate healthy nodes, but cannot alone prove that an already disconnected track stops. The requirement that either track fault stops both therefore needs an independent drive-side communication-loss reaction and commissioning proof.
- Proposed decision: make the safety design two-layered. Keep Lely master-side heartbeat/EMCY detection and all-node NMT Stop. In the drive-adaptation mapping, require both track drives to monitor the rt-control master heartbeat and enter a vendor-confirmed controlled stop/Quick Stop on master-heartbeat loss or NMT Stop, with no automatic motion resume; require the updown drive to enter its vendor-confirmed safe stop/hold/brake behavior on the same all-node stop. Do not authorize any OD value by guess: the exact `0x1016`, master producer period, timeout, `0x605A`, and vendor reaction settings remain blocked until the LD2 manual/live readback and low-speed supported commissioning test prove them. If the LD2 cannot provide the required reaction, track motion remains blocked and an independent hardware safety mechanism is required.
- Benefit: covers both halves of the failure: the master stops the healthy peer, while the disconnected drive stops itself. It avoids pretending that an undeliverable software command can stop a lost node and turns the requirement into an explicit drive configuration/readback/test gate.
- Drawback: requires drive-side configuration changes, a master heartbeat producer, vendor documentation, and destructive-to-motion commissioning tests. Final stopping latency depends on the later approved heartbeat/drive reaction values, and the solution remains non-safety-rated unless implemented by rated hardware.
- Alternative cost: relying only on broadcast NMT Stop is simpler but cannot guarantee the disconnected motor stops; trying to send zero/Quick Stop only from ros2_canopen has the same reachability limitation on the failed link.
- Decision: adopt the proposed two-layer design. Keep Lely master-side heartbeat/EMCY detection and group NMT Stop, and require independently verified drive-side master-heartbeat/NMT-stop reactions for both tracks and a safe stop/hold/brake reaction for updown. Freeze no OD value without vendor evidence/readback; if the LD2 cannot provide the reaction, require an independent hardware safety mechanism.
- Blocked scope: all powered track motion in T-014/T-015 remains blocked until the drive-side reaction is documented, configured, read back, and tested; software configuration work may continue.

## BQ-028 — Bidirectional CANopen heartbeat timing is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-027 requires both the Lely master to monitor nodes and each LD2 drive to monitor the rt-control master. The current live record says drive `0x1017=1000 ms`, the supplied EDS default is 2000 ms, and the master heartbeat producer is disabled (`0 ms`). The former separate 4000 ms watchdog was deleted, so it is not a valid fallback. CANopen heartbeat supervision needs explicit producer periods and consumer deadlines in both directions.
- Proposed decision: use symmetric heartbeat timing: the master and nodes 1/2/3 each produce heartbeat every `100 ms`, and both master-side and drive-side consumers expire after `500 ms` (multiplier 5). Treat 500 ms as the communication-fault detection bound before the configured stop reaction begins; physical stopping time additionally includes the drive's approved deceleration/Quick Stop behavior. Apply the values to all three production nodes so the approved all-node behavior has one timing contract.
- Benefit: detects a lost link within half a second while tolerating approximately five consecutive missed heartbeat periods. Four heartbeat producers at 10 Hz add negligible load on a 500 kbit/s bus, and a symmetric contract is straightforward to archive and test.
- Drawback: changes each drive's current recorded `0x1017=1000 ms`, produces more heartbeat traffic, and a host or CAN stall longer than 500 ms causes a latched group stop requiring full restart. It does not itself guarantee the mechanical stopping distance.
- Alternative cost: retaining 1000 ms producers requires a consumer deadline longer than one second and therefore slower fault detection; using only one or two missed periods reacts faster but raises nuisance-trip risk under transient scheduling/bus delays.
- Decision: instead use the user-approved symmetric values: the master and nodes 1/2/3 each produce heartbeat every `1000 ms`, and both master-side and drive-side consumers expire after `5000 ms` (multiplier 5). The 5000 ms bound covers communication-fault detection only; configured drive stopping time follows afterward.
- Benefit of the approved values: matches the recorded live drive producer period, tolerates five consecutive missed heartbeats, minimizes nuisance trips and CAN heartbeat traffic, and keeps one symmetric timing contract.
- Drawback of the approved values: a communication loss can leave detection pending for up to five seconds before any configured stop reaction begins, producing a substantially longer worst-case stopping distance than the proposed 100/500 ms values.
- Unblocked scope: heartbeat timing is frozen; configuration ownership is resolved by BQ-029.

## BQ-029 — Ownership of heartbeat OD writes [RESOLVED 2026-07-21]

- Evidence: enabling drive-side supervision requires node `0x1016` consumer entries, and making Lely's master-side supervision deterministic requires the configured node `0x1017` producer period to match the generated DCF. The earlier T-006/TBD-011 decision omitted `heartbeat_producer` and prohibited automatic `0x1017` writes because the old external watchdog was the authority. BQ-006/BQ-027 supersede that architecture.
- Decision: ros2_canopen/dcfgen owns the heartbeat configuration at every startup. Configure the master and nodes 1/2/3 for `heartbeat_producer: 1000 ms`, enable consumers in both directions, and use multiplier 5 to produce `5000 ms` consumer deadlines. Authorize generated startup writes for drive `0x1016` and `0x1017` only; this does not authorize PDO remapping or writes to other unresolved OD objects. Inspect the generated DCF/bin, archive the pre-change values, and read back the effective values before powered motion.
- Benefit: the running network cannot silently depend on stale manually persisted heartbeat values; every boot deterministically reapplies the approved contract through the standard ros2_canopen configuration path.
- Drawback: startup now modifies each drive's communication OD, superseding the former no-write rule; a failed/mismatched SDO configuration must block activation. Repeated boot configuration also depends on the drives accepting these writes reliably.
- Unblocked scope: bus.yml and the drive-adaptation table may include the exact heartbeat producer/consumer configuration; live activation remains gated by generated-DCF inspection and SDO readback.

## BQ-030 — Partial-batch EtherCAT enable failure has no rollback action [RESOLVED 2026-07-21]

- Evidence: the five enable batches are sequential, so a timeout/fault in a later batch can leave earlier batches in Operation Enabled. The frozen service response identifies the failed batch/joint/statusword, and normal `/rt/disable` has an approved three-stage sequence, but no source defines what the enable FSM itself does with already enabled axes after a batch failure. “Atomic service” defines one request/result boundary but not the hardware rollback command.
- Conflict: leaving earlier batches enabled after returning `ok=false` violates the expected all-or-nothing operating state. Blindly starting the same control word on all 13 axes is also invalid because a partial enable leaves axes in different CiA402 states. Quick Stop was reserved for emergency motion/fault paths and is unnecessary for axes that have not accepted a motion command.
- Corrected proposed decision (2026-07-21): on any enable-stage timeout, invalid state, preload refusal, or fault, stop advancing immediately and preserve the original failure fields. Enter a per-axis state-aware standard disable rollback in the RT update loop: Operation Enabled axes receive `0x0007` until Switched On (`0x006F == 0x0023`); Switched On axes receive `0x0006` until Ready to Switch On (`0x006F == 0x0021`); Ready axes receive `0x0000` until Switch On Disabled (`0x004F == 0x0040`); axes already farther down join at the matching step. Use the approved `disable_stage_timeout: 4.0s` for each downward stage. Do not use Quick Stop, automatically fault-reset, or resume enable. An axis already in Fault/Fault Reaction receives the non-enabling `0x0000`, remains reported as Fault, and requires explicit fault resolution/full restart; it is not falsely reported as having reached `0x0040`.
- Benefit: rolls every successfully enabled axis down through the approved standard CiA402 sequence, works with the mixed states created by a partial batch, avoids the Ti5 direct-shutdown defect, and preserves the original enable failure diagnosis.
- Drawback: the rollback FSM is per-axis rather than one broadcast phase and can take up to the configured stage timeouts. Faulted axes cannot complete the normal statusword confirmations without a later explicit fault resolution, so the group remains latched FAILED even after healthy axes are disabled.
- Alternative cost: leaving earlier batches enabled is operationally ambiguous and unsafe; forcing every axis to begin at `0x0007` can stall axes that never reached Operation Enabled; immediately forcing `0x0000` on enabled healthy axes skips the approved standard sequence; Quick Stop is not the normal disable path.
- Decision: use the corrected per-axis, state-aware standard disable rollback exactly as proposed. Preserve the original enable failure, use the 4.0-second stage timeout, do not Quick Stop or auto-reset, and leave Fault axes latched for the explicit `/rt/reset_fault` service.
- Unblocked scope: enable-manager failure rollback and retry gating are fully specified for T-010.

## BQ-031 — Explicit EtherCAT Fault Reset service scope is not frozen [RESOLVED 2026-07-21]

- User requirement (2026-07-21): retain an explicit service interface for Fault Reset. It belongs to the existing `enable_manager`, which already exclusively owns the 13 EtherCAT `control_word` command interfaces; do not create another package/node or allow rollback to invoke it automatically.
- Evidence: an axis in CiA402 Fault (`statusword & 0x004F == 0x0008`) does not reach Switch On Disabled merely by receiving `0x0000`. It requires an explicit controlword bit-7 rising edge and confirmation that Fault cleared. The existing `/rt/enable` and `/rt/disable` services have no request fields and do not provide a target-selection contract.
- Proposed decision: add `/rt/reset_fault` as one group-level service implemented by `enable_manager`. One call inspects all 13 axes, applies a bounded standard Fault Reset pulse only to axes currently in Fault while holding every other axis non-enabled, and succeeds only when every targeted Fault axis leaves Fault and reaches Switch On Disabled; otherwise it reports the first failed joint, raw statusword, and reset stage. Reuse the existing structured `robot_interfaces/srv/RtEnable` response shape for this empty-request group operation. Reject the call while enable/disable/reset is already running; never continue automatically into enable.
- Benefit: gives operations one explicit, auditable recovery action for the same 13-axis atomic group, preserves structured failure reporting, and cannot accidentally reset an arbitrary joint name supplied by a caller. It also keeps controlword ownership in one controller.
- Drawback: every currently faulted EtherCAT axis is reset together; selective single-axis maintenance is unavailable. The service type name `RtEnable` is semantically broader than this endpoint even though its response fields fit.
- Alternative cost: a target-joint request allows selective maintenance but requires a new service schema, rules for empty/duplicate/unknown joints, and can leave the group in a partially fault-cleared state; using `std_srvs/Trigger` loses the approved structured joint/statusword failure result.
- Deferred separate decision: the exact low/high pulse duration and timeout are intentionally left to the next question after the service scope is approved.
- Decision: add `/rt/reset_fault` as the proposed 13-axis group-level service in `enable_manager`. Reset only axes currently in Fault, keep every other axis non-enabled, require every targeted axis to reach Switch On Disabled, return structured first-failure information, reject concurrent FSM operations, and never continue automatically into enable. Reuse the existing empty-request `robot_interfaces/srv/RtEnable` response shape for the endpoint.
- Unblocked scope: the service ownership and group scope are frozen; its pulse/timeout behavior remains BQ-032.

## BQ-032 — EtherCAT group Fault Reset pulse and timeout are not frozen [RESOLVED 2026-07-21]

- Evidence: CiA402 requires a controlword bit-7 rising edge to leave Fault, but does not require the application to hold a fixed 200 ms pulse. The control loop is 250 Hz (4 ms). The authoritative legacy fork contains configurable 50-cycle low/high holds for a different automatic-reset path, while the new service is explicit and must not retry or continue into enable. No approved service-specific pulse duration exists.
- Proposed decision: for each group-service call, hold `0x0000` for one full 4 ms RT cycle on all currently faulted axes to guarantee bit 7 is low; then write `0x0080` to those axes and hold it only until each axis leaves Fault or a single `fault_reset_timeout: 4.0s` expires. As soon as an axis leaves Fault, return that axis to `0x0000` and require `statusword & 0x004F == 0x0040`. Make exactly one rising-edge attempt per service call, with no automatic retry. On timeout, keep all controlwords at `0x0000`, latch FAILED, and return the first still-faulted joint/raw statusword/stage.
- Benefit: follows the standard edge semantics, is state-confirmed rather than delay-confirmed, avoids an arbitrary fixed high pulse, and reuses the already approved 4-second safety-stage budget. A failed reset cannot chatter or flow into enable.
- Drawback: some vendor firmware may require a longer minimum low/high pulse despite the standard edge model; that must be detected during commissioning. Holding `0x0080` until state change can last up to four seconds on a non-clearing fault, although no enable command is issued.
- Alternative cost: copying the legacy fixed 50-cycle low plus 50-cycle high sequence adds 400 ms of unneeded fixed delay and was designed for automatic plugin reset rather than an explicit state-confirmed service; allowing repeated edges within one call can mask a persistent fault and create reset chatter.
- Decision: implement the proposed single-attempt, state-confirmed reset sequence: one 4 ms `0x0000` cycle, then `0x0080` until each targeted axis exits Fault or the 4.0-second timeout, followed by `0x0000` and confirmation of `0x0040`. Do not retry or enable automatically; timeout latches FAILED and reports the first remaining fault.
- Unblocked scope: the Fault Reset execution path of T-010 is fully specified, subject to commissioning confirmation of drive pulse acceptance.

## BQ-033 — EtherCAT output-process watchdog timeout is not frozen [RESOLVED 2026-07-21]

- Evidence: the pinned ICube `GenericEcSlave::setup_syncs()` already configures SyncManager 2 (master-to-drive RPDO) with `EC_WD_ENABLE` whenever the slave YAML omits an explicit `sm` section. Therefore the output-process watchdog is enabled by the open-source implementation for all current motion profiles; no additional software watchdog node is needed for this mechanism.
- Remaining gap: ICube never calls IgH `ecrt_slave_config_watchdog()`. IgH consequently leaves registers `0x0400` (watchdog divider) and `0x0420` (watchdog intervals) at the slave/SII defaults. Neither the five authority documents nor a matching ESI/live register archive records those defaults for the ZeroErr and Ti5Robot drives. The effective timeout and each drive's resulting CiA402/mechanical reaction are therefore unproven. A broken EtherCAT link cannot be made safe by sending a later host-side hold/disable command, so this drive-local behavior must be verified before powered motion.
- Proposed decision A (recommended, closest to upstream): retain ICube's existing `EC_WD_ENABLE` behavior and do not patch its watchdog-timing API. Before powered motion, read/archive `0x0400` and `0x0420` for both drive families and perform a supported low-risk link/process-data interruption test that records the measured trip latency, AL/CiA402 state, brake behavior, and mechanical stop/hold result. Reject commissioning if the observed timeout/reaction is not acceptable; any later timing change then requires vendor/ESI evidence and a new approval.
- Benefit of A: preserves the tested open-source path, adds no dependency patch or guessed register write, and verifies the behavior that the hardware actually executes rather than assuming it from configuration. It also respects the hardware mapping's prohibition on inventing watchdog values.
- Drawback of A: the timeout may differ by drive family or firmware and is not deterministically reprogrammed at startup; an unacceptable factory/persisted default will block powered motion and require a follow-up design decision.
- Alternative B: extend the ICube overlay with explicit `watchdog_divider`/`watchdog_intervals` configuration and call `ecrt_slave_config_watchdog()` before master activation, after an exact timeout and per-drive reaction have been justified by vendor/ESI evidence.
- Benefit of B: makes the watchdog timing deterministic and reproducible on every startup.
- Drawback of B: creates another safety-critical downstream ICube patch and writes ESC watchdog registers; no authoritative numeric value exists today, so choosing B immediately creates a second blocked decision and carries nuisance-trip risk at the 4 ms process-data period.
- Question: approve A—keep the upstream-enabled watchdog with slave/SII timing, but block powered EtherCAT motion until the two drive families' effective registers and physical reaction are archived and tested—or choose B and defer implementation until an explicit timeout is separately approved?
- Decision: approve A. Retain the pinned ICube implementation's existing `EC_WD_ENABLE` behavior and the slave/SII watchdog timing; do not add an `ecrt_slave_config_watchdog()` timing patch or write guessed divider/interval values. Before any powered EtherCAT motion, read and archive `0x0400`/`0x0420` for both ZeroErr and Ti5Robot families and complete a supported low-risk interruption test recording the effective trip latency, AL/CiA402 state, brake behavior, and mechanical stop/hold result. An unacceptable result blocks motion and requires a separate evidence-backed timing/reaction decision.
- Unblocked scope: software may retain upstream SyncManager watchdog configuration without another ICube timing patch; powered T-014/T-015 EtherCAT motion remains gated by the register archive and physical-reaction test.

## BQ-034 — The frozen 13-DoF planning axis conflicts with the adjudicated JTC joint set [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-IF-007` says MoveIt plans 13 DoF as the 12 arm joints plus updown, executes the 12 arm axes through JTC and extracts updown as a PP endpoint; waist/turn is outside that planning group. System architecture v1.2 says the same: left arm 6 + right arm 6 + lift/updown form the 13-DoF group, while waist/turn is discrete and workflow-separated. The supplied URDF confirms that `turn` is a continuous rotary joint and `updown` is a distinct prismatic joint mounted after it. Hardware mapping confirms `turn` is the thirteenth EtherCAT CSP drive while `updown` is CANopen Node 1 PP.
- Conflict: the later AMB-002/BQ-002 implementation record instead configures `dual_arm_jtc` as 12 arms plus `turn`, applies the one-degree JTC admission check to 13 EtherCAT axes, and places updown entirely on a separate command path. That changes which physical axis participates in coordinated planning and directly contradicts the frozen requirement that the user previously declared correct.
- Proposed decision A (recommended, follows frozen requirements): restore the planning/execution split to 12 arm joints in `dual_arm_jtc`; motion supplies the arm FJT plus the separately extracted updown PP command from the same 13-DoF plan. Keep both admission checks inside rt-control: the patched JTC checks the 12 arm first-point errors against `0.017453292519943295 rad`, while `updown_position_controller` checks `expected_start_position` against actual updown position within `0.05 m`. Keep `turn` outside this JTC/planning group; freeze its separate command controller/interface in the next question. This supersedes only the conflicting joint-set portions of AMB-002/BQ-002/BQ-019, not the approved preload, freshness, JTC-patch, or updown-controller behavior.
- Benefit of A: matches the frozen REQ and system architecture, preserves updown's real PP semantics instead of streaming it as CSP, and keeps the already approved checks in rt-control. It also avoids falsely treating turn as the coordinated lift axis.
- Drawback of A: motion must split the 13-DoF planned result into a 12-arm FJT and one `/updown/command`; their start coordination is not atomic at the ROS interface, and the separate turn command contract still needs one decision. The JTC admission check covers 12 radian joints rather than the currently recorded 13.
- Alternative B: retain the current adjudication—12 arms plus turn in one 13-axis EtherCAT JTC, with updown independent.
- Benefit of B: all EtherCAT position axes share one stock JTC execution timeline and it needs no new turn command path.
- Drawback of B: it contradicts frozen `REQ-IF-007` and the architecture's kinematic planning model; MoveIt would coordinate waist rotation instead of vertical lift, while the planned lift dimension would not follow the frozen execution split.
- Rejected hybrid: placing CANopen PP updown directly in a streaming JTC with the 12 CSP arm axes would send interpolated position samples to a point-to-point drive and overturn the approved PP endpoint semantics.
- Question: approve A and restore the frozen `12 arms + updown` planning model, or explicitly choose B and authorize a frozen-requirement change?
- Decision: choose B and explicitly supersede the conflicting joint-membership portion of frozen `REQ-IF-007` and system architecture v1.2. `dual_arm_jtc` controls exactly the 12 arm joints plus EtherCAT `turn` as one 13-axis FJT group; CANopen `updown` remains completely independent on its approved PP command path. The JTC first-point consistency check consequently covers those 13 EtherCAT radian joints at the exact one-degree tolerance, while the separate updown controller retains its `0.05 m` expected-start check. Do not later reinterpret the thirteenth JTC axis as updown.
- Benefit of the selected design: all thirteen EtherCAT CSP axes execute on one JTC timeline, and updown retains its hardware-native point-to-point command semantics.
- Drawback of the selected design: this is an explicit frozen-requirement/architecture change—vertical lift is no longer part of the 13-DoF coordinated FJT group, while turn is; any motion/MoveIt configuration derived from the older `12 arms + updown` model must be updated consistently.
- Unblocked scope: retain the existing BQ-002/BQ-019 13-EtherCAT-axis JTC design and the independent BQ-011 through BQ-018 updown path. T-003/T-007 and motion-facing configuration must use the selected membership consistently.

## BQ-035 — Service results when `/rt/disable` preempts an in-progress `/rt/enable` are not frozen [RESOLVED 2026-07-21]

- Evidence: spec section 8 requires an enable request received in `ENABLED` to be idempotently successful and requires a disable received during enable to finish the current batch and then transition to `DISABLING`. `RtEnable.srv` is also required to return the final structured outcome. However, neither the enable response after this preemption nor the callback execution mechanism is specified.
- Runtime constraint: the pinned `ros2_control_node` uses a `MultiThreadedExecutor` and runs read/update/write in a separate RT thread, but services created in a controller's default mutually-exclusive callback group execute serially. If `/rt/enable` synchronously waits there for its final result, `/rt/disable` cannot enter until enable has already returned. Returning immediately would allow preemption but would turn `ok` into mere request acceptance and lose the required final hardware failure fields.
- Proposed decision A (recommended): keep all three services synchronous to the terminal hardware result. Put `/rt/enable` and `/rt/disable` in separate mutually-exclusive callback groups so they may overlap in the non-RT executor. Each callback transfers only an atomic/request record to the RT update FSM and polls a sequence-tagged atomic result outside RT; the RT path takes no lock and creates no string. When disable arrives during enable, complete the currently active enable batch as specified, do not start another batch, then run the approved state-aware group disable. The original enable call returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, `stage="preempted_by_disable"`; the disable call waits for the downward sequence and returns its actual final structured success/failure. A preemption is not falsely attributed to a drive or batch.
- Benefit of A: preserves final-result service semantics, makes the already frozen preemption behavior executable, keeps RT free of locks/services/allocations, and lets the caller that requested disable know whether all axes actually reached the disabled terminal state.
- Drawback of A: both service calls can remain open for several seconds, so clients need an adequate RPC timeout; implementation needs separate callback groups plus sequence IDs/atomics. The enable caller receives failure even though the interruption was intentional and the final machine state may be safely disabled.
- Alternative B: return `ok=true` from enable as soon as the current batch finishes and then process disable.
- Benefit of B: an enable call is not reported as failed merely because a later disable superseded it.
- Drawback of B: `ok=true` would claim successful enable even though the full five-batch operation never reached `ENABLED`, violating the atomic operation meaning and potentially allowing an orchestration layer to advance incorrectly.
- Alternative C: make services return immediate request acceptance and observe final state only through diagnostics.
- Benefit of C: callbacks never wait and default serial callback behavior is harmless.
- Drawback of C: changes the approved `RtEnable` response from a hardware result into an enqueue acknowledgement, so its failure batch/joint/statusword fields cannot fulfill `REQ-IF-001`.
- Question: approve A, including `stage="preempted_by_disable"` for the interrupted enable response, or choose a different return semantic?
- Decision: approve A exactly as proposed. Services remain synchronous to their terminal hardware result and use separate mutually-exclusive callback groups. A disable arriving during enable finishes the current enable batch, prevents any next enable batch, and runs the approved state-aware group disable. The interrupted enable returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="preempted_by_disable"`; the disable call returns the actual terminal disable result. RT communication remains lock-free and allocation-free through numeric sequence-tagged request/result records, with strings formed only in the non-RT callback.
- Unblocked scope: the opposite-direction enable-to-disable preemption and its two service responses are frozen for T-010.

## BQ-036 — Duplicate same-direction enable/disable calls during an active operation are not frozen [RESOLVED 2026-07-21]

- Evidence: spec section 8 defines an enable call in the already terminal `ENABLED` state as idempotently successful, but says nothing about a second `/rt/enable` received while the first call is still `ENABLING`, or a second `/rt/disable` received while `DISABLING`. The empty request carries no command ID, so the controller cannot distinguish a transport/client retry from a separately intended duplicate operation.
- Proposed decision A (recommended): coalesce duplicate same-direction calls into the active operation. Do not restart a batch/stage, reset any timeout, or create a second FSM. Each waiting callback attaches to the active numeric operation sequence and returns the same final structured result when that operation finishes. An enable already in terminal `ENABLED` and a disable already in terminal disabled/IDLE return immediate idempotent success. This rule does not change BQ-035's opposite-direction disable preemption or BQ-031's rejection of reset while another operation is active.
- Benefit of A: makes ordinary service retries idempotent, prevents a retry from extending safety timeouts or replaying controlword edges, and gives every caller the real final hardware result.
- Drawback of A: multiple callers can wait on one long operation and cannot tell whether they initiated it; all receive the same failure/preemption result even if they called at different stages.
- Alternative B: immediately reject the duplicate with `ok=false`, `stage="operation_in_progress"`.
- Benefit of B: ownership is explicit and the second caller returns promptly.
- Drawback of B: a harmless retry appears as an operational failure, and clients need additional retry/state logic despite the service having no request ID or status query dedicated to this purpose.
- Question: approve A—coalesce same-direction calls without restarting or extending the active FSM—or choose B and reject them as busy?
- Decision: choose B. A same-direction request received while its operation is active returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. It does not attach to the operation, restart any batch/stage, replay a controlword edge, or reset/extend a timeout. Calls made after the corresponding terminal state has been reached retain immediate idempotent success.
- Benefit of the selected policy: only the original caller owns and waits for the active operation, while duplicate traffic has no timing or controlword side effect.
- Drawback of the selected policy: an ordinary client retry is reported as busy/failure and must be retried after the original call completes; without a command ID, it cannot recover the original response through the duplicate call.
- Unblocked scope: same-direction concurrency is frozen for `/rt/enable` and `/rt/disable` in T-010.

## BQ-037 — `/rt/enable` received while the group is disabling is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-035 defines the safety-direction transition from an active enable into disable. Neither the frozen specification nor the service contract defines the reverse case: a new `/rt/enable` received while the standard downward sequence is still `DISABLING`. Reversing immediately would leave axes at mixed CiA402 states; queueing it would cause later motion authority to arise from a request made before disable reached its terminal state.
- Proposed decision A (recommended): reject enable immediately while `DISABLING` with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. Continue the existing disable sequence unchanged. After all axes reach Switch On Disabled and the disable call returns, the caller must issue a fresh `/rt/enable`; that new call reruns preload validation and the complete five-batch sequence.
- Benefit of A: the downward safety transition cannot be reversed or followed by an old queued request; every later enable is based on terminal disabled state and fresh position feedback/preload checks.
- Drawback of A: an operator/orchestrator must wait and retry explicitly, increasing recovery latency and requiring it to handle a busy response.
- Alternative B: queue one enable request and start it automatically as soon as disable succeeds.
- Benefit of B: reduces caller orchestration and can shorten turnaround.
- Drawback of B: a stale request can re-enable automatically after the caller's context has changed, and a failed disable creates ambiguous ownership/cancellation semantics. It also bypasses the requirement for a fresh post-disable service decision.
- Question: approve A—reject enable while disabling and require a fresh call after terminal disable—or choose B and queue one automatic re-enable?
- Decision: approve A. While the group is `DISABLING`, `/rt/enable` returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`; it neither reverses nor queues behind the downward sequence. A fresh enable call is required only after terminal disable, and that call reruns the full preload validation and five-batch enable sequence.
- Unblocked scope: disable-to-enable opposite-direction concurrency is frozen for T-010.

## BQ-038 — `/rt/disable` behavior during an active Fault Reset is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-031/032 keep all non-faulted axes non-enabled during `/rt/reset_fault`, while targeted Fault axes receive one low cycle followed by `0x0080` until they clear or time out. BQ-031 rejects a reset request when another operation is active, but does not specify the reverse case of a disable request arriving during reset. A disable is normally the higher-priority safety-direction request, yet a still-faulted axis cannot satisfy the normal disable terminal confirmation `statusword & 0x004F == 0x0040` without a successful reset.
- Proposed decision A (recommended): allow `/rt/disable` to preempt Fault Reset. On the next RT update, cancel the reset attempt and set every controlword to `0x0000`; do not issue another bit-7 edge. The reset call returns `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="preempted_by_disable"`. The disable call evaluates/executes the state-aware downward sequence for every non-faulted axis. It returns success only if all 13 axes reach `0x0040`; if a canceled target remains in Fault, disable returns the first such joint/raw statusword with `stage="fault_requires_reset"`, while keeping all commands at `0x0000`. A later explicit reset call is required.
- Benefit of A: `/rt/disable` always has priority and immediately removes the reset bit; no recovery action continues after an operator asks to disable. The response remains truthful about Fault axes that cannot reach the requested terminal state.
- Drawback of A: disable can intentionally interrupt a nearly completed reset and then return failure even though no axis is enabled and all controlwords are zero. Operations must distinguish “electrically commanded non-enable” from “all axes confirmed Switch On Disabled.”
- Alternative B: reject disable with `stage="operation_in_progress"` and let the bounded reset finish, after which the caller retries disable.
- Benefit of B: avoids interrupting the single reset edge and usually leaves axes in a state where terminal disable can be confirmed.
- Drawback of B: a safety-direction disable request cannot stop the ongoing reset attempt for up to the remaining four-second timeout.
- Question: approve A—disable preempts reset immediately, with a truthful failure if a Fault axis cannot confirm `0x0040`—or choose B and reject disable as busy until reset completes?
- Decision: approve A. `/rt/disable` preempts an active Fault Reset on the next RT update, cancels the reset attempt, writes `0x0000` to every axis, and emits no further bit-7 edge. The reset response is `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, `stage="preempted_by_disable"`. Disable completes the normal state-aware descent for non-faulted axes and succeeds only if all thirteen confirm `0x0040`; any remaining Fault produces the first joint/raw statusword with `stage="fault_requires_reset"`, while all controlwords stay at `0x0000`.
- Unblocked scope: disable priority over reset and the truthful terminal response are frozen for T-010.

## BQ-039 — `/rt/enable` received during an active Fault Reset is not frozen [RESOLVED 2026-07-21]

- Evidence: Fault Reset is a bounded recovery operation that explicitly must not continue automatically into enable. The specification rejects a new reset while another FSM operation is running, but does not say whether an enable arriving during reset is rejected, queued, or starts immediately after reset clears the final Fault.
- Proposed decision A (recommended): reject `/rt/enable` immediately while Fault Reset is active with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. Do not queue it and do not alter the reset attempt. After reset completes successfully and all targeted axes confirm `0x0040`, the caller must make a fresh enable request, which reruns preload validation and the complete five-batch sequence.
- Benefit of A: preserves the approved “reset never auto-enables” boundary, prevents a stale enable from taking effect after delayed recovery, and ensures enabling always reflects a fresh post-reset operator/orchestrator decision.
- Drawback of A: adds one explicit round trip and requires callers to wait/retry after reset.
- Alternative B: queue one enable and start it automatically after successful reset.
- Benefit of B: shorter recovery workflow.
- Drawback of B: semantically couples reset to automatic enable, contrary to BQ-031/032, and introduces cancellation/stale-request behavior if reset is slow or the operating context changes.
- Question: approve A—reject enable during reset and require a fresh post-reset call—or choose B and queue automatic enable?
- Decision: approve A. An enable call received during active Fault Reset returns immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`. It is not queued and does not alter reset. A fresh post-reset enable call is required and reruns preload validation plus the complete five-batch sequence.
- Unblocked scope: enable concurrency during reset is frozen for T-010.

## BQ-040 — Fault Reset called when no axis is in Fault has no defined result [RESOLVED 2026-07-21]

- Evidence: `/rt/reset_fault` is a group service with an empty request that scans all thirteen EtherCAT axes and targets only axes whose masked statusword is Fault. BQ-031/032 define success when every targeted axis clears Fault and reaches `0x0040`, but do not define the empty-target case. Emitting the approved low/high reset sequence when no axis is in Fault would be unnecessary and could create a bit-7 edge on a drive that was not meant to be reset.
- Proposed decision A (recommended): treat an empty target set as immediate idempotent success: return `ok=true`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="already_clear"`. Do not start the RT reset FSM and do not change any controlword.
- Benefit of A: repeated/operator-safe reset calls have no hardware side effect, and the service means “the group has no outstanding Fault requiring reset” on return.
- Drawback of A: a caller cannot use `ok=false` to detect that no reset action was actually performed; it must inspect `stage="already_clear"` if that distinction matters.
- Alternative B: return `ok=false`, `stage="no_fault_present"` without writing anything.
- Benefit of B: clearly tells the caller its requested action had no target.
- Drawback of B: makes an already-safe postcondition look like a recovery failure and complicates idempotent operations/scripts.
- Question: approve A—no Fault means side-effect-free idempotent success—or choose B and report it as a no-target failure?
- Decision: approve A. If no axis is in Fault, `/rt/reset_fault` returns immediately with `ok=true`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="already_clear"`; it starts no RT reset state and changes no controlword.
- Unblocked scope: the empty-target reset result is frozen for T-010.

## BQ-041 — EtherCAT single-axis runtime Fault has no group-stop owner after watchdog removal [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-SAFE-003` requires a Fault to stop/hold all thirteen EtherCAT axes and then reach terminal disable. The standalone watchdog was removed by BQ-006. Pinned upstream ICube decodes each drive's statusword but neither turns a CiA402 Fault into a controller-manager hardware error nor stops peer axes. `dual_arm_jtc` does not claim statusword/controlword, so it continues sampling and writing the active trajectory for healthy axes. The authoritative fork's preload/takeover patch is not a runtime group-stop mechanism: `position_command_takeover_allowed_` becomes permanently true after initial synchronization and is not cleared on a later Fault.
- Conflict: without a new owner, one EtherCAT drive can enter Fault while the other twelve remain Operation Enabled and follow the old FJT, directly violating the group safety requirement. Merely writing Quick Stop controlwords also leaves the JTC goal and its cached command alive for a later re-enable.
- Proposed decision A (recommended): make the existing `enable_manager` the EtherCAT group-Fault owner. During `ENABLING` or `ENABLED`, the first observed `Fault Reaction Active` or `Fault` statusword latches a group fault in the RT update and immediately commands `0x0002` to every non-faulted axis (the faulted axis receives non-enabling `0x0000`). It signals a non-RT callback/worker to request strict deactivation of `dual_arm_jtc` through controller_manager, so the active FJT is terminated and its old position command cannot resume. After JTC deactivation is confirmed, `enable_manager` drives healthy axes to terminal Switch On Disabled and leaves Fault axes at `0x0000`, reporting the original first-fault joint/statusword. Recovery remains explicit: correct the cause, call group `/rt/reset_fault`, then make a fresh `/rt/enable`; no old trajectory is replayed. The exact Quick-Stop-to-terminal transition and JTC reactivation order are deferred to the next questions rather than guessed.
- Benefit of A: detects the fault in the existing 250 Hz control update, stops healthy peers through the controller that already owns every controlword, and explicitly destroys the stale active trajectory before recovery. It adds no watchdog package, raw bus observer, or hardware-plugin Fault policy.
- Drawback of A: `enable_manager` gains a non-RT controller-manager interaction and fault recovery becomes disruptive: the FJT is aborted, JTC must later be reactivated, and any controller-switch failure needs a latched diagnostic/restart path. This remains a software protective stop, not a rated safety function.
- Alternative B: patch ICube to return hardware `ERROR` when any drive statusword enters Fault and rely on controller_manager lifecycle/error handling to deactivate controllers.
- Benefit of B: uses controller-manager hardware-error propagation rather than a custom controller-to-manager request.
- Drawback of B: is a broader hardware-plugin semantic change, does not by itself specify the peer-axis Quick Stop/terminal-disable controlwords or structured first-fault response, and can tear down the whole EtherCAT hardware component before `/rt/reset_fault` can operate.
- Rejected minimal behavior: issuing group `0x0002` while leaving JTC active does stop present drive operation but preserves the old goal/command, creating a possible stale-command takeover after reset/re-enable.
- Question: approve A and assign EtherCAT group-Fault handling to `enable_manager`, including non-RT JTC deactivation and explicit reset/re-enable recovery, or choose B and move fault escalation into the ICube hardware lifecycle?
- Decision: approve A. `enable_manager` owns EtherCAT group-Fault reaction. The first `Fault Reaction Active` or `Fault` observed during `ENABLING`/`ENABLED` latches the first-fault joint/statusword, commands group Quick Stop/non-enable in RT, and triggers non-RT strict deactivation of `dual_arm_jtc`. Recovery never replays the old FJT and requires explicit cause correction, group reset, fresh enable, and a new trajectory. Do not convert a drive Fault into an ICube hardware lifecycle `ERROR` for this mechanism and do not recreate a watchdog package.
- Unblocked scope: the group-Fault owner and JTC-abort requirement are frozen; the exact Quick-Stop-to-terminal sequence and controller reactivation remain the next decisions.

## BQ-042 — Quick-Stop-to-terminal-disable sequence after an EtherCAT group Fault is not frozen [RESOLVED 2026-07-21]

- Evidence: normal `/rt/disable` starts from Operation Enabled and uses the approved `0x0007 -> 0x0006 -> 0x0000` sequence. After BQ-041's emergency `0x0002`, healthy drives normally enter Quick Stop Active (`statusword & 0x006F == 0x0007`), whose standard non-reenabling exit to Switch On Disabled is Disable Voltage (`0x0000`), not replaying the normal three-step path. Depending on the configured drive quick-stop option, an axis may instead already fall to a lower/terminal state. The EtherCAT quick-stop acknowledgement duration and the behavior when JTC deactivation fails were not frozen.
- Proposed decision A (recommended): on the first group Fault, hold healthy Operation Enabled axes at `0x0002` until each reports Quick Stop Active or an already-lower safe state, using the existing `disable_stage_timeout: 4.0s` rather than inventing a new timeout. Fault/Fault-Reaction axes receive `0x0000`. Once an axis is Quick Stop Active, command the standard `0x0000` Disable Voltage and require `statusword & 0x004F == 0x0040`; axes already in Switched On or Ready follow the BQ-030 state-aware downward steps, and axes already at `0x0040` are complete. Run the non-RT strict JTC-deactivation request in parallel. Even if JTC deactivation fails/times out, still complete the hardware terminal-disable sequence because position commands cannot move a drive held at controlword `0x0000`; then latch `stage="jtc_deactivate_failed"`, reject reset/enable recovery in this process, and require a full rt-control restart so no cached JTC trajectory survives. Preserve the original first drive Fault as the primary diagnostic and add the JTC-switch failure as a recovery blocker.
- Benefit of A: follows the correct CiA402 exit for Quick Stop Active, does not leave healthy drives powered in Quick Stop indefinitely, and still reaches the safest available hardware state if ROS controller switching fails. Reusing four seconds avoids an unapproved new safety timeout.
- Drawback of A: `0x0000` removes drive voltage/torque after the configured Quick Stop; the exact mechanical brake/coast behavior remains drive-dependent and must be verified. A JTC-deactivation failure forces a disruptive full container restart even if all axes reached `0x0040`.
- Alternative B: if JTC deactivation is not confirmed, keep healthy axes indefinitely at `0x0002` and do not proceed to `0x0000`.
- Benefit of B: preserves the drive's Quick Stop active torque/braking behavior while the software trajectory owner is uncertain.
- Drawback of B: fails the approved terminal-disable requirement, may leave drives energized indefinitely, and makes recovery dependent on a controller-manager operation that is not needed to physically remove drive voltage.
- Question: approve A—state-confirm Quick Stop, then use standard Disable Voltage to terminal state even if JTC deactivation fails, with restart required on that failure—or choose B and remain latched in Quick Stop until JTC is confirmed inactive?
- Decision: approve A. Healthy enabled axes hold `0x0002` for state confirmation using the existing four-second stage timeout, then Quick Stop Active exits through standard `0x0000` Disable Voltage to confirmed `0x0040`; lower-state axes join the BQ-030 path and Fault axes stay at `0x0000`. JTC deactivation runs in parallel. A JTC deactivation failure does not prevent terminal hardware disable, but latches `jtc_deactivate_failed`, blocks reset/enable in that process, and requires a full rt-control restart.
- Unblocked scope: the emergency Quick-Stop-to-terminal-disable path and controller-switch failure policy are frozen for T-010.

## BQ-043 — The authoritative preload patch is one-shot across disable/re-enable cycles [RESOLVED 2026-07-21]

- Evidence: BQ-002/frozen `REQ-ECAT-005` require raw `0x6064 -> 0x607A` preload before permitting `0x000F`. The reviewed authoritative fork sets `target_position_preloaded_` after its first successful write and sets `position_command_takeover_allowed_` after the first close controller command, but never clears either flag when a drive leaves Operation Enabled, is normally disabled, enters Fault, or is reset. Therefore a second enable in the same process can retain a stale target/preload proof and can immediately pass an old JTC command. This fork behavior is not sufficient for the approved repeated recovery workflow.
- Proposed decision A (recommended): treat preload and controller takeover as per-enable-session state. On any confirmed transition out of Operation Enabled, and also during controller activation/initialization before the first enable, clear `target_position_preloaded_`, its stored preload value, `position_command_takeover_allowed_`, and `position_command_synchronized_`. While the axis is non-enabled and valid fresh process data is available, force raw actual `0x6064` into `0x607A`; mark preload complete only after that PDO write has executed. Keep overriding the position command with this held preload until a newly activated JTC writes a finite command within the exact one-degree threshold of current actual position. Only then allow position-command takeover for that enable session. Fault Reset alone never restores the old takeover permission.
- Benefit of A: enforces preload on every normal re-enable and Fault recovery, prevents an old JTC command from surviving across a disabled interval, and matches the literal “before permitting `0x000F`” requirement rather than merely protecting process startup.
- Drawback of A: this is a deliberate correction to the authoritative fork rather than a byte-for-byte copy. A displaced unpowered axis causes the next enable to hold its new physical position, so the caller must send a newly admitted trajectory; takeover may remain blocked until JTC is active and has seeded a close hold command.
- Alternative B: retain the fork's one-shot latches and rely on JTC deactivation/restart to clear trajectory state.
- Benefit of B: smallest source delta from the authoritative fork.
- Drawback of B: a normal disable/re-enable without process restart can reuse stale `0x607A` and permanent command permission; it violates the approved preload requirement and can produce a jump if an axis moved while disabled.
- Question: approve A and make preload/takeover proof session-scoped, or retain the fork's one-shot behavior B?
- Decision: approve A. Preload value/proof and position-command takeover/synchronization are cleared on every confirmed exit from Operation Enabled and during initial activation. Every enable session performs a new raw actual-to-target preload and permits controller takeover only after a newly seeded finite JTC command is within the exact one-degree threshold. Fault Reset never restores the prior session's permission.
- Unblocked scope: repeated normal enable and Fault-recovery preload semantics are frozen; JTC lifecycle ordering remains separate.

## BQ-044 — JTC lifecycle ordering around normal enable/disable and Fault recovery is not frozen [RESOLVED 2026-07-21]

- Evidence: pinned JTC 2.53.2 rejects FJT goals while INACTIVE, aborts its active goal in `on_deactivate()`, and seeds a hold trajectory in `on_activate()`. Its parameter `set_last_command_interface_value_as_state_on_activation` can instead seed from stale command-interface values; that behavior is unsafe after a disabled interval. BQ-043 now requires a new close command before each session's takeover. Leaving JTC continuously active across `/rt/disable` retains its last hold/trajectory command and provides no lifecycle event to reseed it from current actual position.
- Proposed decision A (recommended): couple `dual_arm_jtc` lifecycle to the existing enable manager without adding another public service. Bringup configures/spawns JTC INACTIVE and sets `set_last_command_interface_value_as_state_on_activation: false`. For normal `/rt/disable`, non-RT logic strictly deactivates JTC (aborting any active FJT) and then the RT FSM completes the approved state-aware hardware disable; if deactivation fails, hardware disable still completes but recovery is restart-only, matching BQ-042. For `/rt/enable`, require JTC to be INACTIVE, perform fresh per-session preload and all five hardware enable batches while the plugin holds the preload target, then strictly activate JTC. Activation reads the thirteen actual positions and seeds a current-position hold; the enable service returns `ok=true` only after JTC is ACTIVE. If JTC activation fails or its final state is ambiguous, execute BQ-030 rollback to terminal hardware disable, return `stage="jtc_activate_failed"`, and require restart if the controller state cannot be confirmed INACTIVE. Fault recovery uses exactly the same reset -> fresh enable -> final JTC activation sequence.
- Benefit of A: no trajectory can be accepted during partial hardware enable or while disabled; every successful enable ends with a freshly seeded stock JTC hold, and every disable destroys the old active goal. It uses existing lifecycle behavior rather than another command-permit topic/interface.
- Drawback of A: every normal enable/disable now includes a controller-manager switch and aborts any active FJT. Hardware is Operation Enabled holding its preload briefly before JTC activation; activation failure adds rollback latency and may force a full restart if lifecycle state is uncertain.
- Alternative B: leave JTC ACTIVE across hardware disable/re-enable and add a new enable-state admission/command-reseed gate to the JTC patch.
- Benefit of B: avoids controller lifecycle switching and keeps the action server continuously active.
- Drawback of B: broadens the JTC patch and needs a new cross-controller state channel; old goals/commands must be explicitly invalidated and the action can otherwise accept motion while hardware is only partially enabled.
- Question: approve A—JTC inactive while hardware is disabled/being enabled, activated only after all thirteen axes hold in Operation Enabled, and deactivated on every disable—or choose B and keep JTC continuously active with an additional gate?
- Decision: approve A. Bringup leaves `dual_arm_jtc` INACTIVE and configures `set_last_command_interface_value_as_state_on_activation: false`. Normal disable deactivates JTC and aborts its active FJT before/alongside terminal hardware disable. Enable performs fresh preload and all five hardware batches first, then activates JTC from actual state; success is returned only after JTC is confirmed ACTIVE. Activation failure invokes BQ-030 rollback and ambiguous lifecycle state requires restart-only recovery.
- Unblocked scope: JTC lifecycle coupling and stale-goal disposal are frozen for T-007/T-010.

## BQ-045 — EtherCAT process-data freshness during preload and enable is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-020 adds read-only `ethercat_domain/process_data_age_ms`, updated only after `EC_WC_COMPLETE`, because ICube otherwise continues exposing a cached process image when Working Counter is incomplete. BQ-002 applies a 500 ms maximum age when admitting FJT goals. The new per-session preload reads cached `0x6064` and writes `0x607A`, but no decision currently prevents a stale cached actual position/statusword from being used to advance the five-batch enable FSM.
- Proposed decision A (recommended): let `enable_manager` also read the already approved `ethercat_domain/process_data_age_ms` interface; this adds no PDO or new state interface. Before starting preload/enable and throughout `ENABLING`, require the value to be finite and `<= 500 ms`. If it is missing/non-finite/older than 500 ms at any enable update, stop advancing immediately, return/record `stage="stale_process_data"`, and execute BQ-030 state-aware rollback for any axes already advanced. While disabled and data is fresh, the BQ-043 preload continuously refreshes raw `0x607A` from the latest `0x6064`, so a previously cached write cannot authorize the next transition. After the service has completed and the system is fully `ENABLED`, retain frozen `REQ-ECAT-009`: age/WC loss is diagnostic WARN and rejects new FJT admission but does not introduce a new host-side automatic runtime stop; the drive-side SyncManager watchdog from BQ-033 remains the broken-link protection.
- Benefit of A: the safety-critical enable decision and preload cannot be based on an arbitrarily old process image, and it reuses the same transport-age proof/tolerance already approved for trajectory admission. It does not alter the frozen runtime WC policy.
- Drawback of A: any configured slave causing domain WC to remain incomplete for over 500 ms aborts the whole enable or rolls back a partially enabled group. The enable manager gains one additional read-only state-interface dependency.
- Alternative B: use statusword/position values without checking domain age during enable.
- Benefit of B: fewer controller dependencies and transient WC issues do not interrupt enable.
- Drawback of B: cached statusword and `0x6064` can look valid indefinitely, so the service could report a successful preload/transition against data received before the current enable attempt.
- Question: approve A and apply the existing 500 ms complete-domain freshness gate throughout `ENABLING`, without changing the post-enable WARN-only runtime policy, or choose B and allow cached values during enable?
- Decision: choose B. `enable_manager` does not claim or check `ethercat_domain/process_data_age_ms` during preload or `ENABLING`; it advances from the exposed cached `0x6064` and statusword values. The existing 500 ms domain-age interface remains only in the patched JTC's new-goal admission path, and runtime WC/age handling remains WARN-only plus the drive-side SyncManager watchdog.
- Benefit of the selected policy: enable has no extra domain-age dependency and transient/incomplete WC age alone does not abort or roll back the five-batch sequence.
- Drawback explicitly accepted: because ICube exposes the cached process image when WC is incomplete, preload and enable confirmation can use position/statusword data older than the current service attempt; the service has no transport-freshness proof for those values.
- Unblocked scope: no EtherCAT age interface is added to `enable_manager`; BQ-020's JTC-only consumer remains unchanged.

## BQ-046 — Stock `Cia402System` has no automatic non-RT init/mode activation path [RESOLVED 2026-07-21]

- Evidence: BQ-003 assigns Node 1 PP and Nodes 2/3 PV selection to CANopen hardware activation. Pinned `Cia402System::on_activate()` only delegates to `CanopenSystem` and never calls `init_motor()` or `set_operation_mode()`. `operation_mode` in `bus.yml` is not consumed. The available `init_cmd`/mode command interfaces are processed in `Cia402System::write()`; commanding `init_cmd` there invokes blocking `Motor402::handleInit()/switchState()` from the controller-manager read/update/write thread, violating frozen `REQ-RT-004`. Stock `Cia402DeviceController` merely exposes services that pulse those same command interfaces and does not move the blocking work out of `write()`.
- Conflict: BQ-005 otherwise permits no activation-semantic dependency patch, so the approved “hardware activation owns mode/init” behavior currently has no executable owner. Leaving it manual would make startup order and partial-node activation external to the hardware lifecycle.
- Proposed decision A (recommended): extend the existing narrowly pinned ros2_canopen overlay only at `Cia402System::on_activate()`/non-RT setup. For each configured joint, call the existing upstream `init_motor()` unchanged, then select the joint parameter `operation_mode` (`1` for Node 1; `3` for Nodes 2/3) using the existing upstream `set_operation_mode()` confirmation. After mode selection, seed Node 1's PP target from its cached actual `0x6064` and seed both track PV targets to zero through the existing upstream `set_target()` API; no custom CiA402 transition or PP handshake is introduced. Activation fails if any init/mode/seed API reports failure and reports the first node/stage. Keep `Motor402::handleInit()`'s upstream order—including enable before mode/target—because BQ-005 explicitly chose it; powered activation remains blocked until the drive-adaptation table/readback proves that this interval cannot cause motion and that updown holds safely with no new external command.
- Benefit of A: puts blocking initialization in a non-RT lifecycle callback, requires no new package/process/controller, makes every hardware activation deterministic, and reuses the selected upstream state machine/mode/target APIs rather than reimplementing CiA402.
- Drawback of A: expands the downstream ros2_canopen patch beyond BQ-006's callback exposure. Because upstream `handleInit()` enables before the safe targets are seeded, software alone still cannot prove no movement during that interval; production remains dependent on drive configuration and commissioning evidence. Target seeding is accepted by the upstream API but has no new device readback acknowledgement.
- Alternative B: keep ros2_canopen untouched and add a separate non-RT startup coordinator that invokes each driver's stock init/mode services in order, then seeds the targets.
- Benefit of B: no ros2_canopen source change.
- Drawback of B: adds another node/process or launch-time service orchestration, creates partial-start/retry ownership outside the hardware lifecycle, and conflicts with the approved placement of mode selection in hardware activation.
- Rejected command-interface path: a controller pulsing `init_cmd` causes the blocking init call in the RT `write()` path and is therefore not allowed.
- Question: approve A and authorize this narrow `Cia402System::on_activate()` orchestration patch while retaining upstream `Motor402` semantics, or choose B and introduce an external non-RT activation coordinator?
- Decision: approve A. The pinned overlay may orchestrate existing upstream `init_motor()`, configured mode selection, and initial safe target seeding from non-RT `Cia402System::on_activate()`. Node 1 uses PP/current-position seed; Nodes 2/3 use PV/zero seed. Upstream `Motor402` transition and PP handshake semantics remain unchanged, and powered activation stays gated by drive-adaptation/readback testing of the enable-before-target interval.
- Unblocked scope: CANopen init/mode ownership has an executable non-RT hardware-lifecycle path; matching deactivation remains separate.

## BQ-047 — Stock CANopen hardware deactivation stops communication without disabling motors [RESOLVED 2026-07-21]

- Evidence: pinned `CanopenSystem::on_deactivate()` is an empty success and `Cia402System::on_deactivate()` only delegates to it. Cleanup/shutdown cancels the executor and destroys the device container; it does not stop targets or run a CiA402 motor shutdown. Upstream `Motor402::handleShutdown()` already changes to No Mode and uses its standard `switchState(Switch_On_Disabled)` path, but `NodeCanopen402Driver`/`Cia402Driver` do not expose that method to `Cia402System`. The approved normal path reserves Quick Stop for emergency/fault handling rather than routine deactivation.
- Proposed decision A (recommended): extend the same narrow overlay with an internal `shutdown_motor()` facade that only forwards to existing upstream `Motor402::handleShutdown()`. In non-RT `Cia402System::on_deactivate()` (and the still-live portion of shutdown/cleanup), first seed Node 1 PP target from cached actual position and both track PV targets to zero, allow the already frozen `20 ms` CANopen stage hold for those targets to be processed, then call `shutdown_motor()` for all three nodes and require their standard transition to Switch On Disabled. Do not use Quick Stop on this normal path. Attempt all nodes even after one failure; if any target/shutdown step fails, request the already approved all-node NMT Stop, return lifecycle ERROR, and record the first node/stage. Only after these best-effort motor steps does communication teardown proceed. Repeated deactivation when nodes are already Switch On Disabled is idempotent.
- Benefit of A: normal container/controller-manager shutdown uses the existing upstream CiA402 state machine, sets safe motion targets before mode removal, and does not silently drop CAN communication while drives remain enabled. It adds no public service or custom controlword sequence.
- Drawback of A: deactivation blocks its non-RT lifecycle callback for at least the 20 ms target hold plus upstream state-transition waits. Updown hold uses the cached `0x6064` without a freshness gate per BQ-014/BQ-045 policy. Abrupt SIGKILL/power loss can still bypass lifecycle cleanup and depends on drive-side heartbeat/NMT reaction.
- Alternative B: leave stock deactivation/cleanup unchanged and rely solely on loss-of-master heartbeat after communication teardown.
- Benefit of B: no additional ros2_canopen patch or shutdown latency.
- Drawback of B: routine orderly stop intentionally leaves motor state uncontrolled until the 5000 ms heartbeat consumer expires, and may retain nonzero track targets during that interval.
- Question: approve A and use upstream `handleShutdown()` through a narrow non-RT facade after current/zero target seeding, or choose B and rely on the five-second drive heartbeat timeout for routine teardown?
- Decision: approve A. Normal CANopen lifecycle deactivation seeds updown current position and both track zero targets, allows the frozen 20 ms processing hold, then calls a narrow internal facade to upstream `Motor402::handleShutdown()` for all three nodes before communication teardown. It attempts every node, uses all-node NMT Stop and lifecycle ERROR on any failure, and does not use Quick Stop on the routine path.
- Unblocked scope: deterministic non-RT CANopen motor shutdown is frozen for T-007/T-014 and container lifecycle handling.

## BQ-048 — Updown travel limit is `0.92 m` in hardware authority but `0.99 m` in description source [RESOLVED 2026-07-21]

- Evidence: the hardware mapping export, explicitly designated as the unique authority for hardware numbers, freezes Node 1 updown's current range as `[0.0, 0.92] m`. The supplied description source writes `<limit lower="0" upper="0.99">` for the active `updown` prismatic joint and documents `0.99`. This is a real 70 mm mismatch. If description/MoveIt advertises 0.99 while rt-control follows hardware authority, plans in `(0.92, 0.99]` are kinematically accepted but physically outside the approved range.
- Proposed decision A (recommended, follows source priority): apply a documented hardware-authority overlay when migrating the active robot description: set the active updown URDF upper limit to exact text `0.92` and use `[0.0, 0.92] m` as the rt-control updown target-admission range. Preserve the vendor/raw source file as provenance rather than silently rewriting it; record the single intentional source delta and require motion/MoveIt configuration to consume the corrected active description. Do not change the conversion factor `1,000,000 counts/m`.
- Benefit of A: planning, admission, and the declared production hardware range agree, so rt-control rejects unreachable/unauthorized lift targets before transmitting them. It obeys the declared hardware-number authority.
- Drawback of A: changes one number from the pinned description source and reduces its advertised travel by 70 mm. If `0.99 m` is later proven mechanically valid, the hardware authority and all consumers must be revised together rather than merely changing URDF.
- Alternative B: retain `0.99 m` in the active description and allow commands to that limit.
- Benefit of B: preserves the supplied description byte value and its nominal larger workspace.
- Drawback of B: overturns the hardware authority without commissioning evidence and can command beyond the currently approved production range.
- Question: approve A and freeze the active updown range at `[0.0, 0.92] m`, or choose B and explicitly supersede the hardware mapping with `0.99 m`?
- Decision: supersede both prior values and freeze the production updown range as exact `[0.0, 0.8] m`. Apply `upper="0.8"` to the active migrated URDF and use the same inclusive range in rt-control command admission and motion/MoveIt configuration. Preserve the original vendor/raw `0.99` artifact as provenance and record that the hardware mapping's prior `0.92` value has been explicitly replaced by this user-approved production limit. Keep the exact position conversion at `1,000,000 counts/m`.
- Benefit of the selected range: leaves an additional 120 mm margin below the former hardware-authority limit and makes every active consumer use one conservative value.
- Drawback of the selected range: reduces usable lift travel by approximately 13% relative to `0.92 m` and is an explicit hardware-number authority change that must be propagated beyond this repository.
- Unblocked scope: T-003/T-007 and updown command validation use `0.0..0.8 m`; no implementation may retain `0.92` or active `0.99` as the production command limit.

## BQ-049 — Updown non-finite/out-of-range command handling is not frozen [RESOLVED 2026-07-21]

- Evidence: `UpdownCommand` carries two `float64` fields and one raw `uint32`. BQ-048 now freezes the final target range at `[0.0, 0.8] m`, while BQ-015 requires the expected start to be within `0.05 m` of actual position. The contract does not define NaN/infinity, target clamping, an expected start outside the production range, or a `profile_velocity_raw` numeric range. No authoritative minimum/maximum/zero semantics for LD2 `0x6081` exists.
- Proposed decision A (recommended): reject rather than clamp whenever `expected_start_position`, final `position`, or actual `0x6064` is non-finite; reject the final target unless it is inclusively within `[0.0, 0.8] m`; then apply the existing `abs(expected_start_position - actual) <= 0.05 m` check. Do not independently range-limit `expected_start_position`: this allows an axis whose measured position is slightly outside the production range to be commanded back to a valid final target, while the proximity check still proves the command was planned from that actual state. Pass the full `uint32 profile_velocity_raw` value—including zero—unchanged to `0x6081` until vendor/readback evidence defines valid bounds; do not invent a clamp or default. Any rejection leaves the currently active PP target/velocity unchanged.
- Benefit of A: prevents NaN conversion and out-of-range motion without silently altering the requested target, preserves a path back into the approved range, and avoids guessing vendor velocity semantics.
- Drawback of A: an expected start outside `[0.0,0.8]` can still pass if it matches actual position, and `profile_velocity_raw=0` is forwarded even though its physical meaning remains unverified. These cases must be covered by commissioning evidence/operator procedure.
- Alternative B: also require `expected_start_position` inside `[0.0,0.8]` and reject `profile_velocity_raw=0`.
- Benefit of B: every command field looks conventionally valid and zero-speed moves are blocked.
- Drawback of B: can prevent a controlled command back into range after overshoot/manual displacement and invents a vendor-specific rule for `0x6081=0` without authority.
- Question: approve A—strict finite/final-target validation, no clamping, expected start allowed outside range only when close to actual, and raw velocity passed unchanged—or choose B?
- Decision: approve A. Reject non-finite `expected_start_position`, final `position`, or actual position; reject a final target outside inclusive `[0.0,0.8] m`; do not clamp. Apply the existing `0.05 m` expected-start proximity check without independently range-limiting `expected_start_position`, so an axis slightly outside the production range can be commanded back to a valid final target. Forward the complete `uint32 profile_velocity_raw`, including zero, unchanged until authoritative vendor evidence defines bounds. A rejected command leaves the active PP target and profile velocity unchanged.
- Unblocked scope: numeric validation and rejection semantics are frozen for the updown command admission layer in T-007.

## BQ-050 — CANopen group Stop does not invalidate command interfaces in stock `Cia402System` [RESOLVED 2026-07-21]

- Evidence: BQ-006/BQ-026 require a track heartbeat timeout or EMCY to issue all-node NMT Stop and latch recovery until a full rt-control restart. The pinned master implements `stop_all_nodes: true` for mandatory-node errors, and the approved narrow callback path covers the required track EMCY case. The pinned `Cia402System` receives each node's existing NMT-state callback, but its stock `write()` continues processing NMT start/reset, init/recover/mode requests and calling `set_target()` every update regardless of the reported NMT state. Therefore NMT Stop prevents present PDO execution, but the software command-interface values and one-shot requests remain live inside the same process.
- Proposed decision A (recommended): add a process-lifetime CAN group-stop latch inside the already approved narrow `Cia402System` overlay. Arm it only after successful hardware activation. If either mandatory track reports unexpected `NmtState::STOP`, latch the whole three-node group; the master remains the owner that performs the actual NMT Stop. From the next `write()`, suppress all external NMT start/reset, init/recover, mode-switch, generic TPDO and motion-target dispatch for all three nodes; overwrite the in-process updown target cache with the latest cached actual position and both track target caches with zero on every cycle. Log the latch once. Normal lifecycle teardown may still attempt the BQ-047 upstream `handleShutdown()` cleanup, but neither deactivation nor reactivation clears the latch; only destruction/recreation in a full rt-control process restart does. Do not add a public command-result/status interface.
- Benefit of A: a controller or stale client cannot stage a target, recovery command, or NMT Start behind the stopped bus; no pre-fault or post-fault command can reappear through a same-process lifecycle cycle. It enforces the already approved restart-only recovery boundary without another watchdog or ROS process.
- Drawback of A: controllers/topics can continue accepting data while the hardware overlay silently suppresses it, because BQ-016 explicitly chose no command status/result. The cached updown hold position may itself be old, although it is not transmitted while NMT Stop is latched. A benign/manual NMT Stop of either track after activation also makes the process restart-only.
- Alternative B: do not add a latch; rely on NMT Stop to block PDO execution and on the required full process restart to destroy all cached commands before any recovery.
- Benefit of B: smaller ros2_canopen overlay and behavior closest to upstream.
- Drawback of B: stock `write()` keeps accepting and attempting targets and even NMT Start/recover operations after the group stop; correctness then depends entirely on master/drive rejection and on every recovery path truly replacing the process before operational state can return.
- Question: approve A—latch unexpected track NMT Stop for the rest of the process and suppress/overwrite all three nodes' commands—or choose B and rely only on NMT Stop plus process restart?
- Decision: choose B. Do not add a process-lifetime command-suppression latch to `Cia402System`. The approved master/callback behavior issues all-node NMT Stop for the required mandatory-track fault events, and recovery remains a complete rt-control process restart that destroys the hardware object and all cached command-interface values before operation can resume. Do not add a same-process NMT recovery path.
- Benefit of the selected policy: keeps the ros2_canopen overlay smaller and leaves ordinary target dispatch behavior aligned with upstream while NMT state provides the execution barrier.
- Drawback explicitly accepted: during the stopped interval, stock `Cia402System::write()` may continue attempting target, NMT Start, recover, or mode operations from live command interfaces. Safety therefore depends on the master/drives honoring NMT Stop and on operational recovery never occurring without full process replacement.
- Unblocked scope: no CAN command-cache latch or suppression layer is added for T-007/T-014; full process restart remains the only recovery route after the approved group stop.

## BQ-051 — Partial CANopen hardware activation failure has no rollback policy [RESOLVED 2026-07-21]

- Evidence: BQ-046 makes `Cia402System::on_activate()` initialize and enable Nodes 1/2/3 sequentially, then select modes and seed safe targets. Pinned upstream `RobotSystem::on_activate()` returns immediately on the first `init_motor()` failure and does not shut down motors already enabled earlier in the loop. A ros2_control lifecycle activation failure does not itself guarantee that those earlier drives receive CiA402 disable commands. The already approved overlay exposes upstream `Motor402::handleShutdown()` as `shutdown_motor()` for BQ-047.
- Proposed decision A (recommended): stop activation at the first failed init/mode/seed stage and run a best-effort rollback inside the same non-RT `on_activate()` call. First request the safe targets that are valid at that point (cached-current hold for Node 1 when finite; zero for both tracks), allow the already frozen 20 ms processing hold, then call `shutdown_motor()` for all three nodes, in reverse order for nodes whose activation stages completed and then for any remaining node that can respond. Attempt every node even after a rollback failure. Return activation ERROR after rollback. If any safe-target or shutdown operation fails, additionally request all-node NMT Stop and report both the original activation failure and the first rollback failure; no automatic retry or partial operation is allowed.
- Benefit of A: a later-node failure does not intentionally leave an earlier drive Operation Enabled, and the rollback reuses the selected upstream standard shutdown path before falling back to the approved group NMT Stop. It keeps all blocking work in the non-RT lifecycle callback.
- Drawback of A: a failed activation takes longer because rollback includes the 20 ms hold and bounded upstream state transitions. A drive that failed before current-position feedback became finite cannot receive a proven updown hold target, so the fallback still depends on NMT Stop/drive configuration if its shutdown also fails.
- Alternative B: on the first activation failure, skip standard rollback and immediately request all-node NMT Stop, then return activation ERROR.
- Benefit of B: fastest and simplest failure exit, with no extra target or state-transition attempts on a partially initialized bus.
- Drawback of B: NMT Stop is a communication-state action rather than confirmed CiA402 Switch On Disabled; previously enabled drives may remain electrically enabled, and their mechanical stop behavior depends on the drive-side heartbeat/NMT configuration.
- Question: approve A—best-effort standard shutdown of the whole three-node group with NMT Stop fallback—or choose B and use immediate NMT Stop only?
- Decision: approve A. Stop at the first failed CAN init/mode/seed stage, apply every valid safe target, hold for 20 ms, and then make a best-effort `shutdown_motor()` attempt for all three nodes. Process all rollback targets even after one fails. If any safe-target or shutdown step fails, additionally request all-node NMT Stop. Return lifecycle activation ERROR with the original activation failure retained as primary and the first rollback failure recorded as secondary; do not retry activation or permit partial operation.
- Unblocked scope: partial CAN hardware activation failure and rollback are frozen for the ros2_canopen overlay in T-007/T-014.

## BQ-052 — Normal EtherCAT disable-stage timeout has no bounded terminal action [RESOLVED 2026-07-21]

- Evidence: normal `/rt/disable` is frozen as the confirmed standard sequence `0x0007 -> 0x0006 -> 0x0000`, with a 4.0 s timeout per stage. The earlier AMB-010 record explicitly says not to skip a stage after timeout, but neither the specification nor later decisions say what the RT FSM commands after that timeout. Returning immediately while leaving the timed-out stage command active can leave an axis electrically Switched On; waiting forever violates the configured timeout and the synchronous final-result service contract. JTC is already strictly deactivated by BQ-044, so no trajectory remains active at this point.
- Proposed decision A (recommended safety fallback): preserve the first timed-out joint/raw statusword/stage as the `/rt/disable` failure, but switch the whole group from the failed normal path into the already approved BQ-042 emergency terminal-disable path. Operation Enabled healthy axes receive `0x0002` until Quick Stop Active using the same 4.0 s bound, then `0x0000` until `0x0040`; lower-state axes continue state-aware downward handling, and Fault axes receive `0x0000`. Attempt every axis independently. The service returns `ok=false` after this bounded cleanup attempt even if all axes eventually reach `0x0040`, because the requested normal sequence failed. If any axis still cannot confirm the terminal state, latch its first cleanup failure, reject later enable/reset in this process, and require hardware intervention/full rt-control restart.
- Benefit of A: a failed normal transition does not deliberately leave healthy peers powered, and it reuses an already approved emergency path rather than inventing another controlword sequence. The response truthfully retains the original normal-disable failure.
- Drawback of A: this explicitly overrides AMB-010's “do not skip after timeout” rule for the abnormal fallback. Quick Stop/Disable Voltage may produce a more abrupt, drive-dependent mechanical response, and the call can take additional emergency-stage timeout periods before returning.
- Alternative B: strictly obey the no-skip rule. On timeout, return `ok=false`, continue writing the timed-out stage's controlword to that axis, allow other axes to finish their normal paths, reject all later enable/reset operations, and require operator/hardware intervention; do not invoke Quick Stop or advance that failed axis to `0x0000`.
- Benefit of B: never bypasses the standard confirmed transition order and avoids an automatic abrupt fallback.
- Drawback of B: the failed axis may remain Switched On or Ready indefinitely, so software cannot guarantee terminal disable and process restart/communication loss may still leave its electrical/mechanical behavior dependent on the drive.
- Question: approve A—on normal disable timeout, run the existing emergency group terminal-disable fallback while returning failure—or choose B and freeze the failed axis at its timed-out standard stage for manual intervention?
- Decision: approve A after clarification. An enable-stage failure first uses BQ-030's standard state-aware rollback. If a normal disable/rollback stage itself times out, preserve that original failure but move the whole group through BQ-042's bounded emergency Quick-Stop-to-Disable-Voltage path and attempt to confirm every axis at `0x0040`. The service remains `ok=false` even when emergency cleanup reaches the terminal state. A subsequent launch may treat the group as cleanly awaiting enable only when all thirteen axes actually report `0x0040`; Ctrl+C/process restart must not claim or synthesize that state. A real Fault remains Fault and requires explicit `/rt/reset_fault`; lost communication or an unconfirmed terminal state remains an operator/hardware blocker.
- Benefit of the selected policy: maximizes the chance that an interrupted/failed enable or disable leaves every reachable healthy axis electrically disabled before process exit, instead of preserving a timed-out Switched On stage.
- Drawback explicitly accepted: the abnormal fallback can be mechanically more abrupt and may extend shutdown by additional stage timeouts; it still cannot guarantee a clean state when communication is unavailable or a drive is genuinely faulted.
- Unblocked scope: the bounded fallback and the next-launch terminal-state precondition are frozen for T-010 and shutdown handling.

## BQ-053 — EtherCAT launch startup behavior for residual CiA402 states is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-030/BQ-052 attempt to leave all axes at Switch On Disabled before a failed service or orderly Ctrl+C completes, but SIGKILL, communication loss, or an exhausted stage timeout can interrupt that cleanup. On the next launch, `auto_state_transitions: false` prevents ICube from normalizing states automatically, JTC starts INACTIVE, and the specification does not say whether `enable_manager` may write controlwords before the first service. A drive can therefore be observed in Ready to Switch On, Switched On, Operation Enabled, Quick Stop Active, Fault Reaction Active, or Fault rather than the intended clean `0x0040` state.
- Proposed decision A (recommended, matches the clarified clean-start goal): when `enable_manager` activates, enter a startup-sanitize FSM before accepting `/rt/enable`. Keep JTC INACTIVE. For non-faulted axes, use the already approved state-aware downward rules: Operation Enabled follows normal `0x0007 -> 0x0006 -> 0x0000`; Switched On/Ready join at their matching step; Quick Stop Active receives `0x0000`; already-disabled axes remain `0x0000`. Use the existing 4.0 s stage timeout and require all axes to confirm `statusword & 0x004F == 0x0040`. Fault/Fault-Reaction axes receive `0x0000` but are not reset automatically; expose the existing `/rt/reset_fault`, and only after explicit reset plus terminal confirmation may the manager become clean/IDLE. Reject `/rt/enable` with `stage="startup_not_clean"` until this completes. A sanitize timeout remains blocked and must not be reported as a clean launch.
- Benefit of A: a process restart cannot accidentally inherit powered residual states as if they were a valid enable session; every accepted enable begins from a confirmed all-axis disabled baseline. It also makes orderly and interrupted shutdown converge on the same condition without automatic Fault Reset.
- Drawback of A: launch activation itself writes controlwords and may remove holding torque from a residual powered axis before an operator calls a service. Startup can remain unavailable for multiple four-second stages, and a real Fault still needs explicit reset/operator action rather than becoming automatically clean.
- Alternative B: startup is read-only. Require all axes already to report `0x0040`; otherwise reject `/rt/enable` and require the operator to call `/rt/disable` or `/rt/reset_fault` explicitly to normalize the group.
- Benefit of B: launch never changes actuator power state without an explicit service request, preserving operator control over a mechanism that might need temporary holding torque.
- Drawback of B: restarting launch does not itself produce the requested clean awaiting-enable state; residual powered states remain until an operator notices and issues the correct service, and `/rt/disable` must be callable from an unclean startup state.
- Question: approve A—automatically sanitize every non-faulted residual state to confirmed `0x0040` at startup while requiring explicit reset for real Faults—or choose B and make startup read-only until an operator calls a service?
- Decision: approve A. `enable_manager` starts in a startup-sanitize state with JTC INACTIVE, automatically applies the approved state-aware downward sequence to every reachable non-faulted residual state, and requires all thirteen axes to confirm `0x0040` before entering clean/IDLE or accepting `/rt/enable`. Fault/Fault-Reaction axes stay at `0x0000` and require the explicit group reset service; no startup auto-reset is allowed. Sanitize timeout or unconfirmed status keeps startup blocked and is never reported as clean.
- Unblocked scope: clean startup normalization and enable admission precondition are frozen for T-010/bringup.

## BQ-054 — A launch `OnShutdown` service call cannot reliably finish EtherCAT disable [RESOLVED 2026-07-21]

- Evidence: the specification says launch registers a shutdown handler that calls `/rt/disable`. In pinned Humble `ros2_control_node`, the controller manager's context pre-shutdown callback removes/cancels the executor, `rclcpp::ok()` becomes false so the 250 Hz read/update/write thread exits, and only then active controllers are deactivated and hardware components shut down. `enable_manager` needs continued RT `update()` cycles to execute and confirm its disable FSM. A launch `OnShutdown` action is dispatched as the same shutdown event also signals child processes, so it cannot guarantee that the service request completes before controller_manager stops. Controller `on_deactivate()` cannot replace the FSM because it has no subsequent update cycles and blocking there would violate the control-thread constraint.
- Proposed decision A (recommended): make the supported rt-control entrypoint a signal-gating wrapper, not raw `ros2 launch`. Keep it in the existing bringup/enable-manager packages: the wrapper starts the launch child in a separately signalled process group and traps orderly SIGINT/SIGTERM first; a small one-shot C++ client installed by the existing `enable_manager` package calls `/rt/disable` and waits for its bounded final result while controller_manager and the RT loop remain alive. Only after the service returns does the wrapper forward SIGINT to the launch process group. If the service is unavailable or returns failure, record that result, still shut the process group down, and rely on BQ-053 startup sanitization rather than claiming a clean exit. No new long-running ROS node or package is added. SIGKILL, host power loss, and a second external forced kill remain outside this orderly guarantee. The wrapper/client wait and container stop-grace bound are deferred to the next timing decision rather than guessed.
- Benefit of A: an ordinary Ctrl+C or container SIGTERM gives the already approved disable FSM actual RT cycles to finish and confirm states before controller_manager teardown. It avoids a controller_manager/ICube patch and adds no resident coordination node.
- Drawback of A: operators and Compose must use the supported wrapper; running raw `ros2 launch` or killing the controller-manager PID bypasses orderly cleanup. The wrapper adds PID/signal-forwarding logic and shutdown latency, and hard power/SIGKILL still depends on hardware safety plus next-start sanitization.
- Alternative B: remove the unreliable launch service claim and rely only on controller-manager's stock shutdown plus BQ-053 normalization at the next startup.
- Benefit of B: no wrapper/helper and Ctrl+C behavior remains stock ROS 2.
- Drawback of B: Ctrl+C cannot promise standard disable or terminal confirmation before communication stops; powered residual states are intentionally left for drive reaction and next launch to handle.
- Question: approve A—use a signal-gating entrypoint plus one-shot C++ `/rt/disable` client before forwarding shutdown—or choose B and make clean state a next-launch-only responsibility?
- Decision: approve A. The supported rt-control process entrypoint is a signal-gating wrapper that starts launch in a separately signalled child process group, traps orderly SIGINT/SIGTERM, invokes a one-shot C++ `/rt/disable` client installed within the existing `enable_manager` package, and forwards shutdown to launch only after the service returns its final result. Failure/unavailability is recorded but cannot be presented as a clean exit; shutdown still proceeds and BQ-053 handles residual states on the next start. Do not add a resident coordinator node or another package. Raw `ros2 launch`, SIGKILL, and host power loss are explicitly outside the orderly-cleanup guarantee.
- Unblocked scope: orderly signal ownership and shutdown-call ordering are frozen for T-007/T-008/T-010; exact wrapper/grace timing remains BQ-055.

## BQ-055 — Orderly shutdown client and container grace periods are not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-054 requires the wrapper to wait for a final `/rt/disable` result before forwarding SIGINT. The JTC strict switch request accepts an explicit timeout, where zero means infinite. With existing 4.0 s FSM stage bounds, a conservative EtherCAT worst path is: finish the active/preempted enable batch (up to 4 s), strict JTC switch (proposed 4 s), normal three-stage disable (up to 12 s), and BQ-052 emergency cleanup (up to 8 s), totaling 28 s before scheduling/transport margin. After SIGINT is forwarded, BQ-047 sequentially invokes upstream `Motor402::handleShutdown()` for three CAN nodes; its unmodified mode/state implementation contains as many as four separate 5 s waits per node in a worst failure path. Docker Compose's ordinary 10 s stop grace can therefore SIGKILL the process before either cleanup path completes. Adding `stop_grace_period` also exceeds BQ-007's previously authorized build-wiring-only Compose additions and needs explicit approval.
- Proposed decision A (recommended): set the strict JTC lifecycle switch timeout to the already approved `4.0 s`; set the one-shot shutdown client's final `/rt/disable` wait to `30 s`; authorize Compose `stop_grace_period: 100s`. The wrapper records a 30 s client timeout as unclean, forwards SIGINT, and the remaining grace allows up to 60 s of sequential stock CAN shutdown plus approximately 10 s process/scheduling margin. A second user-forced SIGKILL still overrides this bound. Keep all component-internal timeouts unchanged; do not parallelize upstream CAN shutdown calls merely to shorten the grace.
- Benefit of A: every wait is finite, normal Ctrl+C/container stop has enough derived time for both the EtherCAT FSM and unmodified upstream CAN shutdown, and a hung process is still forcibly terminated after a known bound. It preserves the chosen upstream CAN behavior.
- Drawback of A: a pathological `docker compose stop` can take up to 100 seconds. The 30/100 s values are deployment policy derived from software worst paths rather than hardware measurements and should be tightened only after commissioning evidence.
- Alternative B: keep the 4 s JTC switch and 30 s helper wait but retain Compose's short/default stop grace, accepting forced termination of CAN shutdown after the grace expires.
- Benefit of B: containers stop faster and no new Compose runtime field is added.
- Drawback of B: directly defeats the clean-exit objective whenever EtherCAT cleanup runs long or sequential CAN shutdown needs more than the remaining grace; the next launch must repair residual states.
- Question: approve A with exact `4 s / 30 s / 100 s` bounds, or choose B and retain the short Compose shutdown grace?
- Decision: approve A. Use a 4.0 s strict JTC lifecycle-switch timeout, a 30 s one-shot `/rt/disable` client deadline, and explicitly authorize `stop_grace_period: 100s` in Compose. On the 30 s client deadline, mark the exit unclean and forward SIGINT so component shutdown still has the remaining grace; do not parallelize or shorten upstream CAN shutdown behavior. These are provisional deployment bounds that may only be reduced using commissioning timing evidence.
- Unblocked scope: orderly shutdown timing and the additional Compose runtime field are frozen for T-007/T-008/T-010.

## BQ-056 — Service behavior while automatic startup sanitization is active is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-053 starts an internal state-aware disable/sanitize FSM before the system becomes clean/IDLE. BQ-036 rejects a duplicate service request while a same-direction service operation is active, but startup sanitization has no original service caller. BQ-054's orderly shutdown wrapper can receive SIGINT during this startup interval and must call `/rt/disable`; rejecting it as busy would cause the wrapper to tear down controller_manager while the safety-direction FSM is already trying to reach `0x0040`. Conversely, allowing `/rt/enable` or Fault Reset to modify the startup FSM before non-faulted axes are clean creates mixed operation ownership.
- Proposed decision A (recommended): while `STARTUP_SANITIZING`, reject `/rt/enable` immediately with `stage="startup_not_clean"` and reject `/rt/reset_fault` with `stage="operation_in_progress"`. Allow the first `/rt/disable` call to attach as the sole service owner of the already-running startup sanitize operation without resetting any stage or timeout; it waits for and returns that operation's real final structured result. Any second disable while that attached call is pending follows BQ-036 and returns `operation_in_progress`. If startup reaches a blocked-Fault condition after every non-faulted axis is terminal, return the attached disable as `ok=false`, first Fault joint/raw statusword, `stage="fault_requires_reset"`; only then accept the explicit group reset service. If all axes reach `0x0040`, return disable success and let the wrapper continue orderly shutdown.
- Benefit of A: Ctrl+C during startup waits on the safety work already in progress instead of restarting it or cutting it off, while enable/reset cannot race the normalization. Stage deadlines are not extended by attaching the caller.
- Drawback of A: this is a narrow exception to the general duplicate-call rejection policy and needs one “attached shutdown caller” result slot. A disable call during startup may block for the remaining sanitize duration even though it did not initiate the operation.
- Alternative B: reject every service during `STARTUP_SANITIZING`; callers retry only after startup reaches clean/blocked terminal state.
- Benefit of B: one uniform busy policy and simpler request ownership.
- Drawback of B: an orderly Ctrl+C during startup cannot wait for the in-progress cleanup and will generally be marked unclean before launch teardown interrupts it.
- Question: approve A—let one `/rt/disable` attach to startup sanitization without restarting it—or choose B and reject all services until startup sanitization finishes?
- Decision: choose B. While `STARTUP_SANITIZING`, `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` all return immediately with `ok=false`, `failed_batch=-1`, empty `failed_joint`, `status_word=0`, and `stage="operation_in_progress"`; none attaches, restarts, cancels, or extends the internal sanitize FSM. After the startup operation reaches its clean or blocked terminal state, a caller may issue a fresh applicable service. If the signal-gating wrapper receives Ctrl+C during this interval, its disable call is recorded as an unclean failure and shutdown proceeds; BQ-053 must normalize residual states on the next launch.
- Benefit of the selected policy: startup has one owner and all concurrent services follow one uniform rejection rule without extra waiter state.
- Drawback explicitly accepted: orderly shutdown during startup does not wait for the sanitizer and can interrupt it before `0x0040` confirmation, despite the longer normal-exit mechanism.
- Unblocked scope: startup-service concurrency is frozen for T-010 and the shutdown wrapper.

## BQ-057 — Service behavior during automatic EtherCAT runtime-Fault shutdown is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-041/042 make `enable_manager` automatically latch a runtime drive Fault, deactivate JTC, Quick Stop healthy peers, and drive the group toward terminal disable without a service caller. A `/rt/disable` can arrive from an operator or the BQ-054 shutdown wrapper while this internal fault-stop FSM is active. BQ-036 only covers duplicate service-owned operations, and BQ-056 explicitly chose rejection for startup sanitization but did not authorize extending that choice to runtime Fault handling. Enable and reset must not race the still-active emergency sequence; the remaining choice is whether one disable caller may wait on the same final outcome.
- Proposed decision A: allow the first `/rt/disable` received during the automatic fault-stop FSM to attach without changing any command, stage, deadline, or first-fault record. It returns `ok=false` with the original first-fault joint/raw statusword and `stage="fault_group_stopped"` after emergency terminal cleanup completes; it cannot return success because the group stop arose from a Fault. Further disable calls follow BQ-036 and return `operation_in_progress`. Reject enable/reset until cleanup is terminal.
- Benefit of A: the shutdown wrapper can wait for emergency hardware cleanup before killing launch, and the caller receives the actual originating Fault rather than a generic busy result.
- Drawback of A: adds another special attached-caller path and can hold `/rt/disable` open through the remaining emergency timeouts even though its final result is necessarily failure.
- Alternative B (consistent with BQ-056): reject `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` with `operation_in_progress` while automatic fault-stop is active. After it reaches terminal disabled/failed state, require a fresh reset/disable request as applicable.
- Benefit of B: one uniform policy for every internally owned FSM and no extra waiter/result slot.
- Drawback of B: Ctrl+C during fault cleanup immediately becomes an unclean exit and may interrupt the Quick-Stop-to-terminal sequence; the next launch must sanitize whatever state remains.
- Question: choose A and allow one disable caller to wait on automatic Fault cleanup, or choose B and reject all services consistently with startup sanitization?
- Decision: choose B. While the internally owned runtime-Fault shutdown is active, `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` all return immediately with the standard `operation_in_progress` response and do not attach, cancel, restart, or extend the emergency FSM. Applicable recovery services require a fresh call only after its terminal state. A shutdown-wrapper disable during this interval is therefore recorded as unclean and launch teardown proceeds.
- Benefit of the selected policy: internally owned startup and runtime-Fault operations now share one simple service-rejection rule.
- Drawback explicitly accepted: Ctrl+C can interrupt an in-progress emergency terminal-disable sequence; residual state is deferred to next-launch sanitization and drive-local safety behavior.
- Unblocked scope: runtime-Fault service concurrency is frozen for T-010/shutdown handling.

## BQ-058 — `/rt/disable` arriving during final JTC activation is not frozen [RESOLVED 2026-07-21]

- Evidence: BQ-035 says a disable received during enable finishes the currently active hardware batch, starts no next batch, and then enters `DISABLING`. BQ-044 adds a final non-RT strict activation of `dual_arm_jtc` after all five hardware batches, with the BQ-055 4.0 s switch timeout; this is part of the synchronous enable operation but is not an enable batch. A controller-manager switch request cannot be safely canceled midway, and rejecting the disable until it returns weakens the already approved higher-priority safety-direction request.
- Proposed decision A (recommended): accept `/rt/disable` during pending JTC activation as BQ-035 preemption. Do not issue another controller-manager request concurrently. Wait only for the already bounded activation request to return. If JTC is then ACTIVE, immediately request strict deactivation; if it is confirmed INACTIVE, skip that request; if its state is ambiguous, continue hardware terminal disable but latch the existing restart-only controller-state failure. In all cases run the normal state-aware hardware disable after resolving/expiring the 4 s switch. The enable caller returns `stage="preempted_by_disable"`; the disable caller returns the actual disable result. No trajectory goal is accepted as a successful enable outcome in between—the enable service never returns success once preemption is recorded.
- Benefit of A: preserves disable priority across the entire enable transaction, avoids two concurrent switch requests, and guarantees that a late preemption cannot be reported as successful enable.
- Drawback of A: the disable may wait up to the remaining 4 s for an activation that it no longer wants, then incur another deactivation switch and the full hardware-down sequence. For a brief interval JTC can become ACTIVE before being immediately deactivated.
- Alternative B: reject disable with `operation_in_progress` while JTC activation is pending; the caller retries only after enable returns.
- Benefit of B: simpler controller-switch ownership and no immediate activate-then-deactivate sequence.
- Drawback of B: a safety-direction disable cannot take ownership for up to 4 s, and the original enable can return success just before the retry even though the disable request already arrived.
- Question: approve A—record disable preemption, wait for the bounded JTC switch, then deactivate/disable—or choose B and reject disable until JTC activation finishes?
- Decision: choose B. While the final strict JTC activation request is pending, `/rt/disable` returns immediately with the standard `operation_in_progress` response and does not preempt or queue. The original enable operation completes with its actual activation result; only a fresh disable call made afterward may begin the normal downward path. A shutdown-wrapper call in this interval is treated as unclean and does not wait/retry automatically.
- Benefit of the selected policy: only one controller-manager switch operation owns the final enable phase and no activate-then-immediate-deactivate branch is added.
- Drawback explicitly accepted: disable priority is suspended for up to the 4.0 s JTC switch bound, and a Ctrl+C in this window can tear down after a busy response rather than performing the orderly service path.
- Unblocked scope: final-enable switch concurrency is frozen for T-010/shutdown handling.

## BQ-059 — Unexpected loss of EtherCAT Operation Enabled without Fault is not covered [RESOLVED 2026-07-21]

- Evidence: BQ-041 triggers automatic group shutdown only when an axis reports Fault Reaction Active or Fault during `ENABLING`/`ENABLED`. CiA402 can also leave Operation Enabled without setting Fault—for example into Quick Stop Active, Switched On, Ready to Switch On, or Switch On Disabled because of a local safety input, watchdog reaction, or external state transition. While `enable_manager` still believes the group is `ENABLED`, JTC remains ACTIVE and the other twelve axes can continue sampling the trajectory. No standalone watchdog remains, and the frozen documents do not define whether a non-Fault state loss is a group event.
- Proposed decision A (recommended): while the manager is stably `ENABLED` and no commanded disable/fault transition is active, require every axis each RT cycle to match Operation Enabled (`statusword & 0x006F == 0x0027`). The first recognized non-Fault mismatch latches the offending joint/raw statusword as `stage="unexpected_drive_state"`, strictly deactivates JTC, and runs the BQ-042 group emergency terminal-disable path. Do not auto-reset. If no axis is in Fault after cleanup, recovery requires a fresh `/rt/enable` (which starts from BQ-053/BQ-043 clean state); if a Fault appears, explicit reset is required first. An unknown/unrecognized statusword is treated the same as a mismatch rather than ignored. Do not add debounce or a new timeout without evidence.
- Benefit of A: a single axis silently losing torque/control authority cannot leave its peers executing a coordinated 13-axis trajectory; all unexpected CiA402 exits converge on the existing group stop and stale-goal disposal path.
- Drawback of A: one transient or torn/cached statusword sample can abort the whole trajectory and force terminal disable/fresh enable. This broadens automatic shutdown beyond the original Fault-only wording and can create nuisance trips during commissioning.
- Alternative B: keep automatic group shutdown limited to Fault/Fault Reaction Active. Report other non-Operation-Enabled states through diagnostics but leave JTC and healthy axes running until an existing hardware/controller error or operator action stops them.
- Benefit of B: adheres narrowly to the currently approved Fault trigger and avoids full-group trips on transient non-Fault states.
- Drawback of B: a coordinated trajectory may continue on twelve axes after one drive has locally Quick-Stopped or disabled, causing large path/pose divergence without an immediate group reaction.
- Question: approve A—treat any unexpected loss of Operation Enabled as a group-stop event—or choose B and stop automatically only on Fault states?
- Decision: approve A. During stable `ENABLED`, every RT update compares the thirteen already-mapped statuswords against Operation Enabled. The first nonmatching or unrecognized state latches `unexpected_drive_state`, deactivates JTC, and enters the existing group emergency terminal-disable path. No debounce is added. A non-Fault terminal result requires a fresh enable; any actual Fault requires explicit reset first.
- Communication/load clarification: this adds no PDO, SDO, state interface, bus frame, or cycle-rate change. The 13 statuswords are already transferred every 4 ms and already claimed by `enable_manager`; the steady-state cost is only thirteen in-memory mask comparisons per 250 Hz update (approximately 3,250 simple comparisons per second). Event handling reuses existing command PDOs and non-RT controller switching.
- Unblocked scope: runtime non-Fault loss-of-enable detection is frozen for T-010/T-012.

## BQ-060 — `turn` is continuous in description but limited to ±179° by hardware authority [RESOLVED 2026-07-21]

- Evidence: the supplied description source declares `<joint name="turn" type="continuous">` with no position limit. The unique hardware-number authority and frozen `REQ-ECAT-007` declare `turn.limit_deg: 179.0`, `lower_position_rad: -3.12413936107`, and `upper_position_rad: 3.12413936107`. Pinned JTC 2.53.2 supports `angle_wraparound`/shortest-angular-distance behavior, but the authoritative controller config does not enable it. If turn is treated as continuous, a goal from approximately +179° to -179° can be transformed into a short path crossing the ±π boundary, outside the frozen ±179° range. If direct numeric difference is used instead, the same physically close wrapped representation fails the 1° admission check.
- Proposed decision A (recommended, follows hardware authority): apply a documented active-description overlay changing `turn` from `continuous` to a bounded `revolute` joint with the exact inclusive position range `[-3.12413936107, 3.12413936107] rad`; propagate the same limits to motion/MoveIt and rt-control. Keep JTC `angle_wraparound=false` for turn and apply the existing 1° admission test as direct absolute numeric difference, exactly like the other bounded joints. Reject any trajectory point outside the turn range rather than wrapping or clamping it. Preserve the pinned raw description as provenance and list this as an intentional hardware-authority delta.
- Benefit of A: kinematics, planning, admission, and the production hardware limit all agree; no trajectory can cross an unapproved wrap boundary merely because the mesh description called the joint continuous.
- Drawback of A: explicitly changes the supplied description's joint type and eliminates continuous shortest-path planning. Commands near opposite ±179° limits require a long in-range move or a different operational plan rather than crossing ±π.
- Alternative B: supersede the hardware mapping limit, retain `turn` as continuous, enable JTC angle wraparound, and use shortest angular distance for its 1° admission check and trajectory interpolation.
- Benefit of B: preserves the description semantics and gives natural short-path motion across ±π.
- Drawback of B: removes a frozen production position boundary without mechanical/drive evidence and can command through a region currently outside the authorized ±179° travel.
- Question: approve A—make active `turn` a bounded revolute ±179° axis with no wraparound—or choose B and explicitly authorize continuous wraparound motion?
- Decision: approve A. The active migrated description defines `turn` as a bounded revolute joint with exact inclusive limits `[-3.12413936107, 3.12413936107] rad`; motion/MoveIt and rt-control use the same range. JTC angle wraparound remains false, the 1° initial-point admission check uses direct absolute numeric difference, and out-of-range trajectory positions are rejected without wrapping or clamping. Preserve the pinned raw description and document this intentional hardware-authority overlay.
- Unblocked scope: turn joint type, planning boundary, and JTC admission/error metric are frozen for T-003/T-007/JTC overlay.

## BQ-061 — Bounded `turn` URDF requires an effort value that has no authority [RESOLVED 2026-07-21]

- Evidence: BQ-060 changes active `turn` from continuous to revolute. URDF requires a revolute `<limit>` to contain lower, upper, effort, and velocity. Hardware authority supplies exact turn position limits and `max_velocity_rad_s: 87.2664626` (explicitly a PLC upper bound, not first-test speed), but supplies no turn maximum effort/torque. The description's arm joints contain model-specific effort values, while its pitch/updown placeholders use `effort="0"`; copying an arm effort to turn would be an unsupported hardware number.
- Proposed decision A: write the active turn limit as lower/upper from BQ-060, `velocity="87.2664626"` from the hardware authority, and `effort="0"` explicitly as an “effort not modeled/not authorized” sentinel consistent with existing description placeholders. Do not expose or enforce an effort command interface, and document that `0` is not a measured zero-torque capability. Motion's initial commissioning remains limited to the already frozen ≤20% velocity scaling; the high PLC velocity limit does not authorize full-speed testing.
- Benefit of A: produces a valid bounded URDF without inventing a torque value and propagates the one available authoritative velocity bound.
- Drawback of A: generic consumers can interpret URDF `effort=0` literally as zero allowable effort, so this description is unsuitable for torque-aware simulation/planning until a real value is supplied. The very large PLC velocity upper bound is also not a safe commissioning speed by itself.
- Alternative B (strict no-placeholder): block the bounded-turn description until the drive/mechanical owner supplies an authoritative maximum effort; do not create the revolute `<limit>` yet.
- Benefit of B: every URDF limit field has real physical meaning and no consumer sees a zero sentinel.
- Drawback of B: BQ-060 cannot be implemented and T-003/T-007 remain blocked even though rt-control is position-only and does not consume effort.
- Question: choose A and authorize `effort="0"` as a documented nonphysical sentinel with authoritative velocity, or choose B and block until turn effort is supplied?
- Decision: choose A. The active turn URDF limit uses exact lower/upper `-3.12413936107`/`3.12413936107`, authoritative `velocity="87.2664626"`, and `effort="0"` as an explicitly nonphysical “not modeled/not authorized” sentinel. Do not add an effort command/state interface or infer torque capability. The velocity remains a PLC ceiling and does not change the ≤20% commissioning restriction.
- Unblocked scope: bounded turn URDF limit fields are frozen for T-003; torque-aware simulation remains outside the authorized scope until real effort evidence exists.

## BQ-062 — Movable `pitch` has no production state source, breaking the RSP TF chain [RESOLVED 2026-07-21]

- Evidence: the supplied description defines `pitch` as a revolute joint from `base_link` to the parent link of `turn`, so all turn/updown/arm transforms depend on its position. Frozen `REQ-CAN-002` prohibits Node 20 from the production CANopen bus/configuration and the implementation spec defers even its read-only diagnostic path; the hardware authority also says its feedback scale is unknown. Frozen `/joint_states` contains 16 production joints (13 EtherCAT + updown + two tracks), not pitch. `robot_state_publisher` publishes a movable joint transform only after receiving that joint's state, so the approved RSP cannot produce a complete `base_link -> pitch -> turn -> ...` TF subtree. Publishing a made-up pitch state or adding Node 20 now would violate the frozen boundary.
- Proposed decision A (only valid if the production mechanism is physically locked at zero for this stage): apply an active-description overlay changing `pitch` to a fixed joint at the source joint's zero pose. Remove it from active MoveIt variables and publish the transform through RSP as static. Preserve the raw source's revolute joint as provenance. Reverting to movable pitch later requires a separately approved read-only physical state source and scale.
- Benefit of A: produces a complete, single-source TF tree with no fabricated runtime feedback or Node 20 write path, and matches a genuinely locked mechanism.
- Drawback of A: if physical pitch is not locked exactly at zero, every downstream arm/lift transform and collision model is wrong; motion planning cannot use pitch at all in this phase.
- Alternative B (strictly preserves physical model): keep `pitch` revolute and do not publish a constant substitute. Mark the complete RSP subtree/T-007 as blocked until an authoritative Node 20 angle conversion and read-only state transport are supplied and approved; Node 20 still receives no NMT/controlword/mode/target/SDO write.
- Benefit of B: never lies about robot geometry and preserves the future movable joint correctly.
- Drawback of B: the production TF tree remains incomplete and T-007/RSP cannot be accepted in this phase without new hardware evidence.
- Question: is pitch mechanically fixed at zero for this production phase, approving A; or choose B and keep the TF/bringup work blocked until real read-only pitch feedback is available?
- Decision: approve A. For this production phase pitch is treated as mechanically fixed at the source joint's zero pose. The active migrated URDF changes it to a fixed joint, removes it from active MoveIt variables, and lets RSP publish that transform statically. Preserve the raw revolute description as provenance. Do not add Node 20 to bus configuration or create any write/read-control path for it in this phase.
- Unblocked scope: the base-to-turn TF chain and production RSP model are frozen for T-003/T-007; future movable pitch support requires a separately approved real state source.

## BQ-063 — Track position scale is unspecified but stock ros2_canopen invents `0.001` [RESOLVED 2026-07-21]

- Evidence: frozen `REQ-IF-002` requires position for all 16 production joints plus track velocity. The hardware authority supplies only the track velocity conversion `scale_vel_to_dev: -731746.8647903234` and `scale_vel_from_dev: -1.36659280431156e-6`; TPDOs also contain raw `0x6064` track position. Pinned ros2_canopen always exports a position state for Cia402 nodes and, when no position scale is configured, silently defaults `scale_pos_from_dev` to `0.001` and `scale_pos_to_dev` to `1000.0`, which have no authority. The production convention deliberately uses linear track speed as the joint velocity and `wheel_radius=1.0`, so accumulated linear displacement is numerically the matching logical joint position.
- Proposed decision A (recommended): explicitly set each track's position scales equal to its already frozen velocity/count scales: `scale_pos_to_dev: -731746.8647903234` and `scale_pos_from_dev: -1.36659280431156e-6`, with zero offsets. Treat `0x6064` as accumulated track displacement in metres; under the intentional `wheel_radius=1.0` convention the same numeric value is the continuous joint coordinate consumed by JSB/RSP. Position commands remain unclaimed/unused in PV operation. Keep the existing T-014 low-speed sign/ratio verification as a commissioning gate and record rollover behavior.
- Benefit of A: eliminates ros2_canopen's unauthorized defaults, preserves the frozen 16-position `/joint_states` contract, and uses no new numeric constant—the derivative of position has the same unit scale as the already authorized velocity.
- Drawback of A: the published track “position” is a logical accumulated linear displacement rather than physical motor/output-shaft radians; its absolute origin is arbitrary and signed. Raw int32 `0x6064` eventually rolls over, and the exact sign/ratio still depends on the pending T-014 commissioning result.
- Alternative B: do not assert a position scale. Patch/specialize the hardware interface so PV track nodes expose velocity but no position, and supersede `/joint_states` to contain 14 positions plus two track velocities until a position conversion is separately proven.
- Benefit of B: publishes no unverified accumulated position and avoids rollover being mistaken for physical angle.
- Drawback of B: changes frozen `REQ-IF-002`, broadens the ros2_canopen overlay, and leaves continuous track-joint TF unavailable unless the active description also fixes those visual joints.
- Question: approve A—reuse the exact authorized count scale for accumulated track position under `wheel_radius=1.0`—or choose B and remove track position from the production interface?
- Decision: approve A. Explicitly configure both track position scales using the same exact authorized count conversion as velocity, with zero offsets, and interpret `0x6064` as accumulated logical track displacement under the intentional `wheel_radius=1.0` convention. Position command interfaces remain unused in PV operation. The subsequently requested physical sprocket-size change can supersede the numeric scale only after BQ-064 resolves whether `0.2088 m` is a radius or diameter; position and velocity scales must then change together from the same approved geometry.
- Unblocked scope: track position-interface semantics are frozen; final numeric factors await the new physical-dimension adjudication.

## BQ-064 — New track sprocket dimension changes the meaning of the frozen conversion [RESOLVED 2026-07-21]

- New direction: replace the track driving-sprocket dimension `0.174` with `0.2088`, reduce the chassis linear-velocity limit to `0.3 m/s`, reduce the angular-velocity limit to `0.3 rad/s`, and set the corresponding acceleration limits to twice the velocity limits. These explicitly requested values supersede the older frozen limits once their units and limiter semantics are resolved.
- Inconsistency: the hardware authority labels `0.174 m` as the wheel dimension and derives the frozen raw scale as `-40 * 10000 / (pi * 0.174)`. That formula uses `0.174 m` as the **diameter**, even though the new direction says the driving-wheel **radius** changes from `0.174` to `0.2088`. Treating the new number as a radius versus a diameter changes every track position/velocity raw conversion by exactly a factor of two.
- Proposed decision A (literal new wording): freeze the physical driving-sprocket radius as `0.2088 m`, hence diameter `0.4176 m`. Configure both position and velocity scales as `scale_*_to_dev = -304894.5269959681` raw per metre (or metre/second) and `scale_*_from_dev = -3.27982273034774e-6` metre (or metre/second) per raw count. Keep the logical diff-drive `wheel_radius=1.0` convention.
- Benefit of A: follows the newly stated word “radius” literally and gives one consistent physical geometry for position and velocity conversion.
- Drawback of A: this changes the old physical diameter from `0.174 m` to `0.4176 m`, a 140% increase, rather than changing like-for-like from `0.174` to `0.2088`; an unintended interpretation would halve commanded/observed linear motion relative to decision B.
- Alternative B (same field as the authority): freeze `0.2088 m` as the new physical driving-sprocket **diameter**, hence radius `0.1044 m`. Configure both position and velocity scales as `scale_*_to_dev = -609789.0539919361` and `scale_*_from_dev = -1.63991136517387e-6`. Keep the logical diff-drive `wheel_radius=1.0` convention.
- Benefit of B: replaces the authority's existing diameter with another diameter; the physical size rises by 20%, which is a like-for-like update.
- Drawback of B: contradicts the new wording “radius `0.2088 m`” and would be wrong by a factor of two if `0.2088 m` is truly the measured radius.
- Common effect: BQ-063 remains valid semantically, but its numeric position and velocity factors change together to the selected geometry. Do **not** set `diff_drive_controller.wheel_radius` to the physical radius: its frozen `1.0` is what makes the hardware joint command/state units directly metres and metres/second; changing it too would apply the wheel-radius conversion twice.
- Question: is `0.2088 m` the measured physical **radius** (choose A), or the measured physical **diameter** replacing the old `0.174 m` diameter (choose B)?
- Decision: choose A. Freeze the physical driving-sprocket radius as exact `0.2088 m` and diameter as `0.4176 m`. Both track nodes use `scale_pos_to_dev` and `scale_vel_to_dev` equal to `-304894.5269959681`; both inverse factors equal `-3.27982273034774e-6`, with zero offsets. Keep the logical diff-drive `wheel_radius=1.0` unchanged. This explicitly supersedes the old `0.174 m` diameter and its `-731746.8647903234` conversion in the hardware authority and frozen REQ-CAN-003.
- Benefit of the selected policy: the CANopen raw conversion now matches the newly confirmed measured physical radius, and position/velocity retain identical derivative-consistent scales.
- Drawback explicitly accepted: this is a large physical-geometry change (old diameter `0.174 m` to new diameter `0.4176 m`) and deliberately invalidates the old frozen conversion; commissioning must reverify direction and measured travel before motion acceptance.
- Unblocked scope: final track position/velocity CANopen conversion is frozen for T-006/T-014.

## BQ-065 — Chassis twist limits do not by themselves limit each track to `0.3 m/s` [RESOLVED 2026-07-21]

- Evidence: the new direction states linear velocity limit `0.3 m/s` and angular velocity limit `0.3 rad/s`. With the frozen `wheel_separation=0.82 m`, standard differential-drive inverse kinematics produce `v_left/right = linear.x ∓ angular.z * 0.41`. A simultaneous corner command `(0.3 m/s, 0.3 rad/s)` therefore commands one track at `0.177 m/s` and the other at `0.423 m/s`, even though each chassis-axis limit is individually satisfied. Under BQ-064 the outer command is approximately `-128970` raw rather than the `-91468` raw corresponding to `0.3 m/s`.
- Proposed decision A (standard diff-drive parameter meaning): apply independent symmetric chassis command limits `linear.x ∈ [-0.3, 0.3] m/s` and `angular.z ∈ [-0.3, 0.3] rad/s`. Do not add a per-track `0.3 m/s` limiter; combined commands may reach `0.423 m/s` on the outside track. The old hardware limit was `0.6 m/s`, so this combined maximum remains below that prior authorized physical-track speed, although that fact does not turn `0.3` into a per-track bound.
- Benefit of A: maps the requested linear/angular limits directly onto stock `diff_drive_controller` parameters, adds no custom saturation path, and permits both requested maxima simultaneously.
- Drawback of A: an individual track can exceed `0.3 m/s` by 41% during combined translation and rotation; anyone reading `0.3 m/s` as a hard actuator/track ceiling would get the wrong behavior.
- Alternative B (hard physical-track ceiling): treat `0.3 m/s` as the maximum magnitude of either track. After inverse kinematics, uniformly scale both track velocities whenever `max(abs(v_left), abs(v_right)) > 0.3`, preserving curvature. At the simultaneous maximum corner, both chassis components reduce by factor `0.3/0.423 = 0.709219858156`, yielding approximately `linear.x=0.212765957447 m/s` and `angular.z=0.212765957447 rad/s`.
- Benefit of B: neither track ever exceeds the requested `0.3 m/s`, and uniform scaling preserves the commanded curvature/sign relationship.
- Drawback of B: the two advertised chassis maxima cannot be achieved simultaneously, and pinned stock `diff_drive_controller` does not expose this coupled wheel-speed limiter as a parameter, so it requires a narrow controller overlay/patch plus tests.
- Question: choose A and interpret `0.3 m/s`/`0.3 rad/s` as independent chassis-twist limits, or choose B and additionally enforce `0.3 m/s` as a hard limit on each physical track?
- Decision: choose A. Configure stock `diff_drive_controller` with independent symmetric chassis limits `linear.x.min_velocity=-0.3`, `linear.x.max_velocity=0.3`, `angular.z.min_velocity=-0.3`, and `angular.z.max_velocity=0.3`, with both velocity-limit enable flags true. Do not add a coupled per-track limiter. At simultaneous maxima the standard inverse kinematics may command one physical track at `0.423 m/s`; this is explicitly permitted.
- Benefit of the selected policy: the requested chassis limits map directly to supported upstream parameters with no controller patch, and either maximum remains usable while the other component is nonzero.
- Drawback explicitly accepted: the stated `0.3 m/s` is not a hard actuator/track-speed ceiling; a combined command can drive the outer track 41% faster.
- Unblocked scope: chassis velocity-limiter semantics are frozen for T-007; acceleration semantics remain in BQ-066.

## BQ-066 — “Acceleration is twice velocity” does not specify the normal deceleration bound [RESOLVED 2026-07-21]

- Evidence: the pinned stock `diff_drive_controller` exposes signed `min_acceleration` and `max_acceleration`, not separate acceleration/deceleration parameters. If only `max_acceleration` is supplied, the upstream `SpeedLimiter` automatically sets `min_acceleration=-max_acceleration`, so the normal slowdown is symmetric. The same limiter runs after a stale `cmd_vel` is replaced with zero, whereas hardware disable, NMT Stop, EMCY/heartbeat handling, and the approved emergency group-stop paths are separate and must not be delayed by this ordinary motion limit.
- Proposed decision A (recommended upstream-native interpretation): enable acceleration limiting and explicitly set linear acceleration to `[-0.6, 0.6] m/s²` and angular acceleration to `[-0.6, 0.6] rad/s²`; leave jerk limiting disabled because no jerk value was authorized. Thus normal acceleration, normal braking, reversal, and `cmd_vel`-timeout ramp-down all use the same magnitude. From maximum speed, an ordinary stop takes at least `0.5 s` and approximately `0.075 m` (or `0.075 rad`) under a constant `0.6` deceleration; reversal from `+0.3` to `-0.3` takes at least `1.0 s`.
- Benefit of A: directly implements “twice” with supported stock parameters, gives predictable bidirectional ramps, and avoids an unbounded command discontinuity that can cause track slip or mechanical shock.
- Drawback of A: an ordinary command stop, including command-timeout zeroing, is intentionally not immediate and can add up to `0.075 m` straight-line stopping distance at the limit. This is not the emergency-stop guarantee.
- Alternative B: apply `+0.6` only to speed increase but do not assume a `-0.6` normal deceleration. Because the pinned controller defaults an omitted minimum to `-max`, B requires either a separately supplied negative-deceleration value or another narrow limiter patch; implementation remains blocked until that value/behavior is frozen.
- Benefit of B: allows normal braking to be made faster than acceleration if that is the intended vehicle behavior.
- Drawback of B: it is not implementable from the current numbers alone and loses the upstream-native symmetric interpretation; unrestricted braking would also create step commands and higher slip/shock risk.
- Question: choose A and make both normal acceleration and deceleration symmetric at magnitude `0.6`, or choose B and provide a distinct normal-deceleration requirement?
- Decision: choose A. Enable the stock acceleration limiters and explicitly configure `linear.x.min_acceleration=-0.6`, `linear.x.max_acceleration=0.6`, `angular.z.min_acceleration=-0.6`, and `angular.z.max_acceleration=0.6`. Leave both jerk-limit flags false. These are ordinary chassis-command limits; emergency hardware group stop, NMT Stop, heartbeat/EMCY reaction, and disable/fault FSMs bypass this ordinary ramp as already frozen.
- Benefit of the selected policy: ordinary acceleration, braking, reversal, and stale-command zeroing have one auditable symmetric upstream-native behavior with no controller patch.
- Drawback explicitly accepted: a normal stop from the configured maximum takes at least `0.5 s` and can travel/rotate approximately `0.075 m`/`0.075 rad`; this must not be presented as emergency-stop performance.
- Unblocked scope: chassis velocity/acceleration limiter values and semantics are frozen for T-007.

## BQ-067 — T-004 `joint_limits.yaml` target path conflicts with its allowed modification scope [RESOLVED 2026-07-21]

- Evidence: implementation spec §4 defines the target ownership as `src/rt_control/rt_control_bringup/config/joint_limits.yaml`, alongside `controllers.yaml`. TASK-RT-004 nevertheless says its allowed modification scope is only `robot_hw_ethercat/`, while also requiring migration of `joint_limits.yaml`. Both instructions cannot be satisfied literally. Creating a copy in both packages would introduce two editable numeric authorities and an unspecified consumer precedence.
- Proposed decision A (recommended, follows the target architecture): place the one production `joint_limits.yaml` at `src/rt_control/rt_control_bringup/config/joint_limits.yaml` and narrowly expand T-004's allowed paths to that single bringup file. Keep slave profiles and `ecat.ros2_control.xacro` in `robot_hw_ethercat`; T-007's controller configuration consumes the bringup-owned limit file. Do not create another copy in the hardware package.
- Benefit of A: preserves the explicit target directory architecture and gives controller/MoveIt-facing runtime configuration one authoritative limit file adjacent to `controllers.yaml`.
- Drawback of A: T-004 touches one file outside the execution-plan line that says only `robot_hw_ethercat/`, so that path whitelist must be explicitly superseded.
- Alternative B (follows the task path whitelist): place the only `joint_limits.yaml` at `src/rt_control/robot_hw_ethercat/config/joint_limits.yaml` and supersede spec §4's bringup location. T-007/launch must resolve the limits from the hardware configuration package.
- Benefit of B: T-004 stays entirely inside its stated allowed package and keeps hardware numeric configuration together.
- Drawback of B: violates the documented package ownership, couples controller/MoveIt limits to the EtherCAT hardware package, and requires later launch/controller references to diverge from spec §4.
- Rejected implicit option: do not install two copies; synchronization and precedence are not specified and would undermine the exact-text migration gate.
- Question: choose A and authorize the single bringup-owned `joint_limits.yaml`, or choose B and move the authoritative limit file into `robot_hw_ethercat`?
- Decision: choose A. The single production `joint_limits.yaml` is owned by `src/rt_control/rt_control_bringup/config/joint_limits.yaml`. T-004 receives a narrow path-whitelist exception for that one file; its slave profiles and EtherCAT ros2_control xacro remain under `robot_hw_ethercat`. Do not create a second limits file in the hardware package.
- Benefit of the selected policy: controller and launch configuration retain the target package ownership documented by implementation spec §4, with one exact-text numeric authority.
- Drawback explicitly accepted: T-004 modifies one bringup-owned file outside the execution plan's original `robot_hw_ethercat/`-only line.
- Unblocked scope: T-004 and dependent T-005/T-007/T-010/T-012 work may proceed.
## BQ-068 — 250 Hz Controller Manager 无法用官方分频得到精确 100 Hz JSB [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：冻结 `REQ-RT-001` 要求 Controller Manager 为 250 Hz，冻结 `REQ-IF-002` 要求
  `/joint_states` 为 100 Hz。ros2_control Humble 基线的 per-controller `update_rate`
  采用整数分频；250/100 不是整数，因此配置 `joint_state_broadcaster.update_rate: 100`
  时，源码明确将实际频率调整为 `250 / floor(250/100) = 125 Hz`。
- 证据：`controller_manager.cpp` 的 controller update-rate 校验与 update loop（上游只读基线
  `/home/kkozia/rt_control_refs/ros2_control@e65ddd72804f3f2d9b19e533a15ed436b2f3fc42`）。
- Decision：将 `/joint_states` 的生产频率由原冻结的 100 Hz 改为 **50 Hz**，明确取代
  `REQ-IF-002` 中的 100 Hz 数值。Controller Manager 保持 250 Hz；给官方
  `joint_state_broadcaster` 配置 `update_rate: 50`，使用精确整数分频 5，不增加发布节流补丁。
- Benefit：完全沿用 Humble 上游调度逻辑，精确得到 50 Hz，并相对原 100 Hz 目标降低一半
  JointState 序列化、DDS 和下游处理负载。
- Drawback：外部关节状态更新周期从原目标的 10 ms 增加到 20 ms；依赖 100 Hz 状态流的域外
  消费者必须接受新契约。250 Hz 控制环和内部硬件反馈采样频率不变。
- Unblocked scope：BQ-068 不再阻塞 T-007；控制器配置和接口检查统一使用 50 Hz。

## BQ-069 — 同一 service 的互斥回调组无法实现“重复调用立即 busy” [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：BQ-035 已批准 `/rt/enable` 与 `/rt/disable` 分别使用独立的
  `MutuallyExclusive` callback group，以便两种服务可以并发、disable 可以打断 enable；
  BQ-036 又要求同方向的第二次调用在第一次仍执行时**立即**返回
  `stage="operation_in_progress"`。在 Humble MultiThreadedExecutor 中，同一个
  MutuallyExclusive group 内的第二个同名 service callback 不会并发进入，而是排队到第一次
  callback 返回；届时操作已经结束，无法再按 BQ-036 立即报 busy。
- 需要裁决：A）将三个 service 各自改为 `Reentrant` callback group，在代码中用原子所有权实现
  同向重复调用立即 busy，同时仍允许 enable/disable 并发；或 B）保留
  `MutuallyExclusive`，接受同向重复请求由 executor 排队，完成后按终态幂等结果返回。
- 权衡：A 精确满足 BQ-036，但放弃了 BQ-035 对 callback-group 类型的字面选择并要求所有共享
  状态均通过原子记录保护；B 保持原 callback-group 设计更简单，但可观察的重复调用语义与已批准
  结果不同。
- Decision：选择 A。`/rt/enable`、`/rt/disable`、`/rt/reset_fault` 分别使用
  `Reentrant` callback group；每个 service callback 先通过原子操作所有权判定，同方向已有操作时
  立即返回 `ok=false, stage="operation_in_progress"`，不得排队后伪装成终态幂等调用。
  enable/disable 之间已冻结的并发、抢占和不抢占窗口保持不变。
- Benefit：精确保留 BQ-036 的可观察立即-busy 语义，并允许独立 service 方向按既有状态机规则
  并发进入，而不依赖 executor 排队时序。
- Drawback：放弃 BQ-035 对 `MutuallyExclusive` 类型的字面选择；共享操作状态必须全部采用原子
  记录或明确的单所有者传递，代码审查和竞态验证面增大。
- Unblocked scope：BQ-069 不再阻塞 T-010；enable_manager 按 Reentrant+atomic 方案实现。

## BQ-070 — enable_manager 诊断如何交给 T-012 未定义 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：冻结诊断表要求 1 Hz 输出 enable_manager 状态和首个失败字段，但已批准的
  `RtEnable.srv` 只有请求的最终响应，没有状态查询；BQ-024 只冻结了 EtherCAT 快照接口，
  没有冻结 enable_manager 到 `rt_diagnostics` 的状态 topic/state interface。
- 需要裁决：A）由 enable_manager 自己的非 RT 1 Hz timer 直接发布该行标准
  `/diagnostics`（T-012 聚合其余来源，不新增状态 topic）；或 B）新增一个只读状态消息/topic，
  由 `rt_diagnostics` 订阅后统一发布。
- 权衡：A 接口最少、状态所有者直接发布，但该行不经过聚合节点；B 所有行由一个节点统一，
  代价是新增公共消息、topic、QoS 和生命周期契约。
- Decision：选择 A。enable_manager 使用自身普通优先级、非 RT 的 1 Hz timer，直接向标准
  `/diagnostics` 发布 `enable_manager` 行及已冻结的 `state`、`failed_batch`、`failed_joint`、
  `failed_status_word` 字段；不得在 controller `update()` 路径发布、记录日志或分配消息。
  `rt_diagnostics` 继续发布 EtherCAT/CANopen 等其余行，不新增 enable-manager 状态 topic/message。
- Benefit：使用 ROS 标准 diagnostics 的多发布者模型，状态所有者直接生成快照，不新增机内接口、
  QoS 和消息版本维护面。
- Drawback：`/diagnostics` 的生产者不再只有 `rt_diagnostics`，enable_manager 行不会经过该节点的
  统一组装；消费者必须按诊断 name/hardware_id 聚合，而不能假设单一 publisher。
- Unblocked scope：BQ-070 不再阻塞 T-012 的 enable_manager 行，也不新增接口包工作。

## BQ-071 — 当前 ICube 固定提交属于 Jazzy，不能在冻结的 Humble 环境编译 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 冲突：当前审计参考 `/home/kkozia/rt_control_refs/ecat_icube` 固定为官方 `jazzy`
  提交 `7a32bd7b8fc066c6668b1df7446c89aff570bb7d`，其 `EthercatDriver::on_init()`
  使用 Jazzy 的 `hardware_interface::HardwareComponentInterfaceParams`。冻结运行环境是 ROS 2
  Humble；Humble 的 `SystemInterface` 只有 `on_init(const HardwareInfo &)`，因此该官方提交在
  `ros:humble-ros-base` 中原样编译失败。官方仓库另有 Humble 分支，当前头为
  `1390be742986f4e898ca112e49bb24805be9899a`，但它的内部结构早于 Jazzy 的
  `EthercatBusManager` 重构，切换后必须针对 Humble 源重新审计并生成 preload/只读状态补丁。
- 选择 A（推荐，向开源目标发行版靠拢）：固定官方 Humble 提交
  `1390be742986f4e898ca112e49bb24805be9899a`，在该源上重做已批准的原始位置预装载、
  per-enable-session 清理和只读状态接口补丁。
- A 的好处：基础 API 与部署发行版一致，避免维护 Jazzy→Humble 的整层兼容回移，官方分支可
  直接接受 Humble 的 ros2_control ABI。A 的弊端：放弃已经审计过的 Jazzy BusManager 重构，
  需要重新检查旧版 read/write、锁和生命周期实现，补丁落点及测试都要重做。
- 选择 B：继续固定 Jazzy `7a32bd7`，授权增加 Humble API 兼容回移，并继续修复随后发现的
  Jazzy/Humble ABI 差异。
- B 的好处：保留较新的 BusManager 拆分和目前已完成的 Jazzy 源码审计。B 的弊端：形成跨发行版
  私有 backport，当前只证实了第一个 `on_init` 错误，后续可能还有更多编译或运行 ABI 差异，
  维护和验证面明显更大。
- Decision：选择 A。ICube 唯一实现基线改为官方 Humble 提交
  `1390be742986f4e898ca112e49bb24805be9899a`；在该提交上重新审计 read/write、生命周期和锁，
  并重做已批准的原始位置预装载、per-enable-session 清理及
  `ethercat_domain/process_data_age_ms` 只读状态接口补丁。禁止把 Jazzy
  `7a32bd7b8fc066c6668b1df7446c89aff570bb7d` 的 BusManager 结构或兼容回移带入生产分支。
- Benefit：ICube 基础源码与 ROS 2 Humble 的 ros2_control ABI 原生一致，部署和编译目标统一，
  避免维护不可预估的跨发行版私有 backport，并符合向官方开源实现靠拢的原则。
- Drawback：必须放弃此前针对 Jazzy BusManager 的源码审计落点，在旧版 Humble 结构中重新审计
  和实现补丁；两版内部结构不同，不能机械移植补丁。
- Process correction：该发行版不匹配本应在 T-001/T-002 基线核验时报告，实际直到 Humble
  Docker 编译才暴露，属于实现前置检查顺序错误。任何 Jazzy ICube 实验修改均未进入目标仓库或
  提交，因此不存在生产代码回退；后续必须先完成 Humble 原版编译，再开始功能补丁。
- Unblocked scope：ICube overlay 可在固定 Humble 基线上重做；完成前仍阻塞 T-007 全量加载和
  T-012 EtherCAT 数据源验收。

## BQ-072 — Leadshine 原始 EDS 不能被 Humble `dcfgen 2.4.0` 原样解析 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 证据：在恢复后的 Docker 网络环境中安装官方 Humble
  `ros-humble-lely-core-libraries`，按上游 `cogen_dcf()` 流程运行
  `cogen -> dcfgen -rS`。仓库内原始 `LD2-CAN.eds` 的
  `SupportedObjects=0x0003/0x01CA/0x0054` 被 `dcf-tools 2.4.0` 用十进制
  `int(..., 10)` 解析而失败；其 9 个 PDO COB-ID 表达式使用 `$NodeID`，而工具只向
  表达式环境注入大写 `$NODEID`。`-S/--no-strict` 只放宽 lint，不能绕过这两个解析错误。
  EDS 还含 `[607E]` 的 `DataType=INTEGER8, HighLimit=255` lint 警告，但 `-S` 可保留
  原文并继续生成。
- 已做只读验证：仅在一次性临时副本中把三个 `SupportedObjects` 改为数值等价的
  十进制 `3/458/84`，把 `$NodeID` 改成 `$NODEID`，其余内容不变；随后成功生成
  `master.dcf`、`master.bin` 和三个节点 `.bin`。verbose 输出只有每节点
  `0x1016:1=5000 ms consumer(node 100)` 与 `0x1017:0=1000 ms producer`，与已冻结值一致。
  仓库中的权威原始 EDS 未修改，SHA-256 仍为
  `e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08`。
- 选择 A（推荐，最小构建适配）：权威原始 EDS 原封不动入仓并继续用于存档校验；构建时
  复制到 build 目录，严格只执行上述 12 处等值文本规范化，再运行 `cogen` 和
  `dcfgen -rS`。安装生成物和规范化后的运行时 EDS，同时记录源/派生文件哈希及转换日志。
- A 的好处：不改任何对象值、PDO 映射或驱动器参数，已证明可由官方 Humble 工具生成，
  且保留供应商文件作为唯一审计源。A 的弊端：明确例外于“数值文本不得改格式”的一般规则；
  运行时使用一个派生 EDS，构建脚本必须长期维护并验证只发生这 12 处变换；`-S` 仍会接受
  供应商 `[607E]` 元数据不一致警告。
- 选择 B（工具兼容补丁）：不转换 EDS，给 `dcf-tools 2.4.0` 增加兼容解析，使
  `SupportedObjects` 接受基数自动识别并对变量名做大小写兼容，再以原始 EDS 生成。
- B 的好处：构建和运行始终消费字节完全相同的供应商 EDS。B 的弊端：新增第三个上游依赖
  补丁，修改通用 DCF 解析语义，验证和长期维护面明显大于 12 处确定性构建转换。
- 选择 C：不授权任何转换或工具补丁，等待驱动器厂商提供可被 `dcfgen 2.4.0` 接受且经重新
  签核的 EDS；在此之前保持 CANopen DCF/加载/实机任务阻塞。
- Decision：选择 A。供应商原始 EDS 保持字节不变并继续作为唯一硬件审计源；构建脚本先校验其
  SHA-256 必须为 `e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08`，
  在 build 目录复制后仅执行三个 `SupportedObjects` 十六进制到等值十进制转换，以及九个
  `$NodeID` 到 `$NODEID` 的大小写转换。脚本必须验证替换数量恰为 3+9，输出源/派生哈希及转换
  日志；随后用未修改的官方 Humble `cogen` 和 `dcfgen -rS` 生成并安装运行时 bus/EDS/DCF/BIN。
- Benefit：已实证可由官方 Humble 工具链生成，且不修改解析工具、对象值、PDO 映射、默认值或
  驱动器参数；供应商原始文件仍可按固定哈希独立审计。
- Drawback：明确授权一个只限于该固定哈希 EDS 的文本格式例外；运行时 EDS 与供应商文件不再
  字节相同，构建和评审必须长期保留派生哈希/转换日志，`-S` 仍会显示 `[607E]` 元数据警告。
- Unblocked scope：DCF 构建适配可实施；完成生成检查后 BQ-072 不再阻塞 T-007/T-014。

## BQ-073 — 官方 Humble ICube 的 IgH 安装前缀与现有 Dockerfile 不一致 [RESOLVED 2026-07-21]

- 状态：**RESOLVED**
- 证据：按 BQ-071 固定官方 Humble `1390be742986f4e898ca112e49bb24805be9899a` 后，先在
  已完成的 rt-control 镜像中原样编译。官方 `ethercat_interface/CMakeLists.txt` 与
  `ethercat_manager/CMakeLists.txt` 均硬编码 `ETHERLAB_DIR=/usr/local/etherlab`，导出
  `/usr/local/etherlab/include` 并从 `/usr/local/etherlab/lib` 查找 `libethercat`。现有
  `docker/rt-control/Dockerfile` 则以 `./configure --prefix=/usr/local --disable-kernel` 安装，
  实际头文件/库位于 `/usr/local/include`、`/usr/local/lib`。因此官方原版首先在不存在的导出
  include 路径上失败，尚未进入功能补丁编译。
- 选择 A（推荐，向官方 ICube 布局靠拢）：把容器内 IgH 用户库安装前缀改为
  `/usr/local/etherlab`，同步设置 `PATH=/usr/local/etherlab/bin`、
  `LD_LIBRARY_PATH=/usr/local/etherlab/lib` 并重跑 T-008 镜像/版本检查。后续 ICube 不为路径增加
  补丁；T-009 宿主脚本也明确记录其独立安装前缀，避免把容器路径误当宿主 ABI 要求。
- A 的好处：官方 Humble ICube 原版 CMake 可直接查找 IgH，减少一个非功能性下游源码补丁；
  EtherLab 文件集中在专用目录，来源和卸载边界更清楚。A 的弊端：修改已验证的 Docker 镜像布局，
  必须重建镜像并复核 `ethercat` CLI、动态库搜索路径以及宿主/容器版本一致性命令。
- 选择 B：保留容器 `/usr/local` 前缀，给官方 ICube 的两个 CMakeLists 增加可配置
  `ETHERLAB_DIR` cache 变量，并在构建时传 `/usr/local`。
- B 的好处：现有 Dockerfile、CLI 和动态库路径保持不变，补丁是常见的 CMake 可配置化改动。
  B 的弊端：新增与机器人功能无关的 ICube 下游补丁；任何遗漏的硬编码路径仍可能在后续包或导出
  配置中出现，且偏离“官方 Humble 原版先编译”的目标。
- Decision：选择 A。容器内 IgH 用户态源码仍由 Dockerfile 构建，但安装前缀改为
  `/usr/local/etherlab`；设置 `PATH=/usr/local/etherlab/bin` 与
  `LD_LIBRARY_PATH=/usr/local/etherlab/lib`。官方 Humble ICube 不携带 ETHERLAB_DIR 路径补丁。
- Benefit：部署布局与官方 ICube 的原生查找路径一致，少维护一个无关功能的下游补丁；头文件、
  库和 CLI 均位于独立 EtherLab 目录。
- Drawback：T-008 镜像需要重建，并重新核验 CLI、动态库和版本一致性；现有 `/usr/local` 布局
  不再是生产镜像布局。
- Unblocked scope：官方 Humble ICube 原版编译可在新布局镜像中重新验证。

## BQ-074 — 是否在最终运行镜像中保留 IgH 完整源码

- 状态：**RESOLVED 2026-07-21**
- 证据：当前 Dockerfile 在镜像构建过程中克隆并编译 IgH `stable-1.6`，随后执行
  `rm -rf /tmp/ethercat`。因此“在 Docker 内从主站源码构建”已经成立，但最终运行镜像只保留
  `/usr/local/etherlab` 下的用户态安装产物，不保留 Git 工作树。用户提出“肯定要把主站源码放在
  docker 里”，可能表示要求最终镜像也能现场查看源码，现有行为不满足该解释。
- 选择 A：在最终镜像 `/opt/src/igh-ethercat` 保留与安装产物完全对应的固定源码工作树和提交信息，
  不允许运行时修改或从该目录重新安装。
- A 的好处：离线现场可审计具体源码/提交，排障时可直接对照头文件与实现，也更易证明容器用户库
  的来源。A 的弊端：增加镜像体积和文件数量；源码本身不参与运行，保留编译工作树可能让现场人员
  误以为可在生产容器内临时重编覆盖。
- 选择 B：构建后删除源码，仅在镜像标签、versions.env 和构建记录中保存精确提交/版本，最终镜像
  只含运行与后续包编译需要的 bin/lib/include。
- B 的好处：镜像更小、运行内容边界更清楚，避免生产容器内临时编译的误用。B 的弊端：现场离线
  无法直接查看完整主站实现，需要从固定提交另取源码核对。
- Decision：选择 B。IgH 源码只存在于 Docker 构建阶段；安装到
  `/usr/local/etherlab` 后删除 Git 工作树，最终运行镜像只保留运行以及编译 ROS overlay 所需的
  bin/lib/include，并在版本清单和构建记录中保存可核验版本与精确提交。
- Benefit：减小最终镜像并清晰区分运行产物与源码，避免在生产容器中临时修改、重编或覆盖主站。
- Drawback：现场离线时无法直接查看完整 IgH 实现；排障审计需要按记录的固定提交另取源码。
- Unblocked scope：T-008 最终镜像内容可定稿；本裁决不阻塞 ICube Humble 原版及功能补丁编译。

## BQ-075 — IgH `stable-1.6` 是可移动分支，无法仅凭版本名复现构建

- 状态：**RESOLVED 2026-07-22**
- 证据：`versions.env` 当前只有 `IGH_VERSION=stable-1.6`，Dockerfile 使用
  `git clone --depth 1 --branch "${IGH_VERSION}"`。2026-07-21 只读查询官方远端时，该分支指向
  `2f7f884f1c7d377c02a7d627eb06512126a0e50e`；分支今后可移动，同一仓库提交不能保证得到同一主站源码。
  BQ-074 选择 B 又要求删除源码并在版本/构建记录中保存精确提交，因此必须明确是固定输入，还是仅记录
  每次实际解析到的输出提交。
- 选择 A（推荐，可复现）：增加 `IGH_COMMIT=2f7f884f1c7d377c02a7d627eb06512126a0e50e`，构建时仍可记录
  人类可读版本 `stable-1.6`，但必须 checkout 并校验这个完整提交；把版本和提交写入镜像 label/清单。
- A 的好处：今后重建消费同一源码，能准确对应已验证的 ICube/IgH ABI；即使分支移动也不改变生产输入。
  A 的弊端：不会自动获取 stable-1.6 后续修复，升级必须显式改提交并重新验证。
- 选择 B（只记录、不固定）：继续从 `stable-1.6` 分支头构建，每次把实际 `git rev-parse HEAD` 写入镜像；
  接受不同时期构建可能使用不同源码。
- B 的好处：自动得到分支后续修复，不需要人工更新提交。B 的弊端：相同配置不能保证复现相同镜像，且未经
  再审批的上游变化会直接进入下一次构建。
- Decision：选择 A。保留 `IGH_VERSION=stable-1.6` 作为人类可读版本，同时固定
  `IGH_COMMIT=2f7f884f1c7d377c02a7d627eb06512126a0e50e`。Docker 构建必须直接 fetch、checkout 并
  校验该完整提交，把两者写入镜像 label 和依赖版本清单；不得在构建时解析可移动分支头作为输入。
- Benefit：所有后续构建使用同一份已知源码，并能将 IgH 安装产物追溯到完整提交，不受官方分支移动影响。
- Drawback：不会自动接收 `stable-1.6` 后续修复；任何升级都必须显式修改提交、重新编译并完成回归验证。
- Unblocked scope：T-008 的 IgH 源码版本输入可定稿并重新验证。

## BQ-076 — 清华 ROS 2 镜像缺少 `ros-humble-lely-core-libraries`

- 状态：**RESOLVED 2026-07-22**
- 证据：按 BQ-073/BQ-075 重建时，固定 IgH 提交已成功编译并安装到 `/usr/local/etherlab`；随后
  `rosdep install` 在清华 ROS 2 apt 镜像报 `Unable to locate package
  ros-humble-lely-core-libraries`。使用相同 `ros:humble-ros-base` 但不替换 ROS 2 软件源，只读运行
  `apt-cache policy` 可得到官方源候选版本 `0.2.13-1jammy.20260304.090440`。因此是清华镜像缺包，
  不是依赖名错误。当前 Dockerfile 选择清华源是早期网络条件下的实现，冻结规格没有规定缺包时的来源优先级。
- 选择 A（推荐，代理已修复）：Ubuntu 软件包继续使用清华镜像，但 ROS 2 apt 和 rosdep 索引恢复官方
  `packages.ros.org`/`raw.githubusercontent.com`，然后通过 apt 安装发布版 Lely。
- A 的好处：使用 ROS 官方发布的 Humble 二进制包及依赖解析，不增加 Lely 私有源码构建和补丁，完整工作区
  已实证能解析到所需版本。A 的弊端：镜像构建依赖 GitHub/ROS 官方网络可用性，国内网络速度可能低于镜像。
- 选择 B：保持清华 ROS 2/rosdep 镜像，另行固定并从源码构建 `lely_core_libraries`。
- B 的好处：继续使用国内镜像完成主要 apt/rosdep 流程。B 的弊端：新增一个必须固定、编译、安装和验证的
  源码依赖；其 ABI/安装布局必须与 ros2_canopen Humble 重新核对，维护面更大。
- 不建议混用条件式 fallback：同一 Dockerfile 根据镜像当时是否缺包在清华和官方 apt 间切换，会使实际二进制
  来源随时间变化，不利于复现和审计。
- Decision：选择 A。Ubuntu apt 继续使用清华镜像；ROS 2 apt 保持基础镜像中的官方
  `packages.ros.org`，rosdep 保持官方 `raw.githubusercontent.com` 和官方 rosdistro 索引。Lely 使用
  官方 Humble 二进制发布包，不增加源码构建。
- Benefit：恢复已实证存在的官方 Lely 包并沿用 ROS Humble 发布体系，避免增加一个源码依赖及其 ABI、安装和
  补丁维护工作。
- Drawback：Docker 构建依赖 ROS/GitHub 官方网络可用性，国内下载可能比镜像更慢。
- Unblocked scope：可重新执行 T-008 镜像构建及全工作区依赖/编译检查。

## BQ-077 — 官方 ROS apt 可达，但 Docker 构建内 GitHub Raw 的 rosdep 索引连续超时

- 状态：**RESOLVED 2026-07-22**
- 证据：BQ-076 A 实施后，`packages.ros.org` 已成功下载 ROS 2 软件包，原先缺失的 Lely 二进制包
  可以解析；但 `rosdep update` 两次连续在 `raw.githubusercontent.com/ros/rosdistro` TLS 读取阶段
  超时，一次只读到两个 YAML，另一次只读到一个。Docker daemon 显示 HTTP/HTTPS proxy 为
  `127.0.0.1:10808`，但构建 `RUN` 环境没有 `HTTP_PROXY/HTTPS_PROXY`；容器内的 `127.0.0.1` 也不是
  宿主代理。因此“Docker 代理已修复”目前覆盖镜像拉取，不等于构建步骤能使用该宿主回环代理。
- 选择 A（推荐，最小来源拆分）：保持 ROS 二进制 apt 为官方 `packages.ros.org`，仅把 rosdep YAML 与
  rosdistro 索引恢复到清华镜像。rosdep 索引只把包键映射到 apt 包名，最终 Lely/ROS 二进制仍由官方 apt
  下载。
- A 的好处：沿用此前已成功的索引访问路径，不暴露或硬编码宿主代理地址；同时解决清华 apt 缺 Lely和
  GitHub Raw 超时两个问题。A 的弊端：依赖清华 rosdep/rosdistro 镜像同步时效，索引与官方 apt 可能短暂
  不同步。
- 选择 B：保持 rosdep/rosdistro 全部官方，暂停镜像构建，先由用户提供一个从 Docker build 网络可达的
  代理地址或系统级 BuildKit 代理配置，再重试。
- B 的好处：所有 ROS 元数据和二进制均来自官方端点。B 的弊端：当前任务继续阻塞；不能把 daemon 的
  `127.0.0.1:10808` 直接作为容器代理，代理地址和凭据也不应猜测或写入仓库。
- Decision：选择 A。ROS 2 apt 保持官方 `packages.ros.org`；仅将 rosdep YAML URL 重写到清华
  `github-raw` 镜像，并将 `ROSDISTRO_INDEX_URL` 指向清华 rosdistro index。不得把宿主回环代理地址或凭据
  写入仓库/Dockerfile。
- Benefit：最终 ROS/Lely 二进制仍由官方 apt 发布源提供，同时避开 Docker build 网络对 GitHub Raw 的
  连续超时，也不引入环境相关代理配置。
- Drawback：依赖清华 rosdep/rosdistro 元数据镜像的同步时效；镜像与官方 apt 短暂不同步时仍可能解析失败。
- Unblocked scope：可继续 T-008 全镜像构建及依赖、标签、IgH 安装布局检查。

## BQ-078 — ICube Humble 的原始位置预装载与发送确认边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（安全关键下游补丁）**
- 证据：官方 Humble ICube 在 `read()` 中保存换算后的 `last_position_`，但没有为外部
  `control_word` 所有者提供“本次使能会话的 0x607A 已按最新 0x6064 原始值装载并至少进入一次
  EtherCAT 发送”的证明。仅在 `write()` 内给 0x607A 赋值不能区分尚未调用
  `ecrt_master_send()` 与已提交给主站的帧，也不能防止上一使能会话的旧标志授权下一次 0x000F。
- 自主裁决：在固定 Humble 提交上保持 position-only PDO，不新增 PDO/SDO；以驱动器原始
  `0x6064` 精确复制到 `0x607A`。补丁采用两阶段确认：slave 写入本周期预装载后只登记候选，
  `EcMaster` 完成 `ecrt_master_send()` 后再通知 slave 把该候选确认为已发送。外部控制字在本会话
  首次确认前把 `0x000F` 钳制为 `0x0007`；确认至少一个更早周期的预装载帧后才允许 0x000F。
  离开 Operation Enabled、硬件 deactivate 或新 activate 时清空本会话确认；位置命令只有在与
  当前反馈相差不超过精确 `1° = 0.017453292519943295 rad` 时才完成接管。硬件 activate 等待预装载
  的上限采用已批准的 `5000 ms`，但不等待 Operation Enabled，后者仍由 enable_manager 管理。
- Benefit：每次使能都由最新原始反馈自动预装载，避免编码器比例/offset 的往返舍入，也保证 0x000F
  不会与首个预装载值同帧越过；用户无需先发单点 FJT。
- Drawback：确认边界只能证明数据已经交给 IgH 的 `ecrt_master_send()`，不能证明从站物理接收或
  内部接受；补丁横跨 ICube master/slave/驱动层，未来升级官方提交时必须重新审计。为满足 RT
  规则，周期路径不记录日志，现场定位只能依赖只读状态接口和非 RT diagnostics。
- 验证边界：补丁 SHA-256 为
  `70b18622069420185d8f94b7b1ffe546cea52795df5525884ba84b1295267d64`；已在固定 Humble 原版上
  `git apply --check`，且其最小上游安全测试为 91 tests、0 errors、13 skipped。实机接收与无跳变
  仍由 T-013 证明。

## BQ-079 — 生产源码 overlay 的固定提交与维护边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：生产构建只导入三份运行依赖源码并固定完整提交：ICube Humble
  `1390be742986f4e898ca112e49bb24805be9899a`、ros2_canopen Humble
  `fef50e54b1c94c50e908e2c5d0b8888eed907e8d`、ros2_controllers Humble
  `cbcf66218ff43353f9fb5fe7a2c33f458d578d73`。三份补丁在 Docker 构建中先执行
  `git apply --check` 再应用。`/home/kkozia/robot_driver@6bc94cd` 继续只读，只作为迁移/差异权威，
  不作为生产 runtime vendor，也不写入 `deps.repos`。
- Benefit：构建输入可复现，目标运行时不会悄然链接只读存量 fork；每个偏离上游的行为都有单独补丁
  和固定落点。
- Drawback：镜像必须从源码构建三套 overlay，构建时间、网络依赖和升级维护成本均增加；任何上游
  升级都不能仅改分支名，必须重新 apply-check、编译和审计补丁语义。

## BQ-080 — enable_manager 超时、控制器切换歧义和复位终态 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（全组使能/去使能状态机）**
- 自主裁决：RT `update()` 仅用固定数组、标量和原子标志推进状态机；service 回调以 2 ms 普通线程
  轮询最终结果，单次最多 30 s。30 s 只是调用方等待上限，不在后台取消一个仍安全推进的硬件
  状态机。常规下使能按状态感知的标准 `0x0007 -> 0x0006 -> 0x0000` 执行；任一阶段 4 s 超时后，
  全组进入有界 `0x0002` Quick Stop，再继续 terminal disable。任何轴 Fault/Fault Reaction 或
  ENABLED 中意外离开 Operation Enabled 都触发相同全组路径，并在非 RT 线程严格停用 JTC。
- 自主裁决：controller-manager switch 的明确 `ok=false` 记为失败；service 不可用、future 超时或
  同时已有 switch 无法判断服务端最终状态，记为 **ambiguous**。激活歧义立即锁存
  `restart_required`，并发起一次尽力 JTC deactivate；本进程拒绝再次 enable/reset，避免不确定的
  JTC 生命周期和缓存轨迹被复用。
- 自主裁决：`/rt/reset_fault` 为全组操作，先全组至少一个 0x0000 周期，仅对当前 Fault/Fault
  Reaction 目标轴发 0x0080；成功条件不是“原目标已清”，而是全部 13 轴都已到 0x0040。复位期间
  新出现的另一轴故障因此不能被误报为成功。
- Benefit：任何普通失败都优先完成硬件去使能；无法证明 JTC 状态时选择重启边界，不会在不确定的
  控制器状态下恢复运动；全组复位结果与“干净待使能状态”一致。
- Drawback：短暂的 controller-manager 响应延迟也可能升级为必须重启，恢复较保守；service 客户端
  30 s 超时后后台 FSM 仍可能完成，调用方需看 diagnostics 再决定重试；Quick Stop/disable 的实物
  减速度和抱闸效果仍必须由 T-013 验证，软件路径不是认证安全功能。

## BQ-081 — T-012 诊断聚合只使用现有权威来源 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：`rt_diagnostics` 只订阅 JSB 的 `/dynamic_joint_states` 与标准
  `/diagnostics`。EtherCAT 行使用 BQ-024 的只读接口；CANopen 行只接受 hardware_id 1/2/3 且含
  上游 native NMT/EMCY/CiA402 字段的状态，再规范化到 `/robot/rt_control/canopen/node_<id>`。
  本节点自己发布的 `hardware_id=robot-001` 行不会被再次采集，避免自反馈。源快照 3 s 未更新即
  STALE；WC 只在累计值相对上一秒增长时 WARN。enable_manager 按 BQ-070 自己发布其诊断行。
- Benefit：不再造 raw CAN 观察器、心跳年龄或 SDO 计数，也不把 diagnostics 变成第二套安全判断；
  EtherCAT、CANopen 和使能状态都来自实际执行者。
- Drawback：`/diagnostics` 是多发布者模型，消费者必须按 name/hardware_id 聚合；CAN 行的字段和
  可见性依赖 pinned ros2_canopen native diagnostics，JSB 未运行时 EtherCAT 全部 STALE。

## BQ-082 — 生产/Mock 启动链与干净停止入口 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：`use_mock_hardware` 默认 `false`，生产 xacro 只装载真实 ICube 和 Cia402System；显式
  `true` 时才装载接口镜像用 GenericSystem。启动时 JTC 必须先成功配置为 INACTIVE，随后才加载
  enable_manager；JTC spawner 失败则关闭整次 launch，不留下可使能的半启动系统。JSB、updown、
  diff-drive 默认 ACTIVE，JTC 默认 INACTIVE。容器唯一支持的入口是 `rt_control_start`：launch 位于
  独立进程组，收到 INT/TERM 先调用 `/rt/disable` 的一次性客户端，再把 SIGINT 转发给 launch。
- Benefit：Mock 可执行冻结的控制器加载检查而不伪装生产总线；生产启动不会在 JTC 配置失败时开放
  使能；容器停止有明确的标准下使能机会。
- Drawback：Mock 只验证接口和生命周期加载，不模拟真实 CiA402 状态跳转、CANopen 心跳或机械响应；
  直接绕过包装脚本运行 `ros2 launch` 不享受 orderly disable，必须作为不受支持的调试方式标注。

## BQ-083 — Docker build 的宿主网络与代理暴露边界 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（构建网络边界）**
- 证据：直接 VCS 拉取在当前网络多次卡住，而用户确认本机代理和 Docker 已恢复；宿主代理监听
  `127.0.0.1:10808`，普通 Docker build 网络中的回环并不指向宿主。
- 自主裁决：compose 的 build 使用 `network: host`；可选环境变量
  `RT_CONTROL_BUILD_PROXY` 只映射到 Docker/BuildKit 预定义的 `HTTP_PROXY/HTTPS_PROXY` build args，
  Dockerfile 不声明同名 `ARG`，避免代理值进入镜像历史。仅 `vcs import` 那个 RUN 继承代理；apt、
  IgH fetch/build、rosdep 和 colcon 的 RUN 均先显式清除大小写 HTTP(S)/ALL_PROXY。仓库、镜像 ENV、
  runtime compose 和生成文档中均不写代理地址或凭据。
- Benefit：当前 GitHub 源码拉取可稳定完成，同时代理配置保持为构建机临时输入，不进入生产镜像或
  远程仓库。
- Drawback：BuildKit 构建阶段获得宿主网络可见性，隔离弱于默认 bridge；源码拉取仍依赖操作者
  正确提供本机代理，且代理服务能观察该阶段访问的三个公开 Git 仓库。

## BQ-084 — 生产镜像排除测试依赖与测试目标 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 自主裁决：Docker 中先用 `colcon list --packages-up-to` 得到生产包闭包，只对这些路径执行
  `rosdep install`，dependency types 限于 build/buildtool/export/exec；colcon 统一传
  `-DBUILD_TESTING=OFF`。规则 16 允许的 ICube 安全补丁最小上游测试已在独立固定源码工作区运行，
  不把 test/doc/lint 工具链带入最终镜像。
- Benefit：生产镜像依赖面和构建时间下降，严格遵守“不建设测试体系”的冻结范围；实际 runtime
  包闭包仍由 rosdep 解析而非人工漏列。
- Drawback：生产 Docker build 本身不会发现仅在上游测试目标中发生的编译回归；补丁升级时必须在
  镜像外重新执行相应的最小安全测试并留证。

## BQ-085 — JTC admission 快照竞争采用有界拒绝 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：JTC 的 FJT goal callback 在非 RT executor 中读取 RT `update()` 写入的 13 轴位置和反馈
  age。seqlock 风格快照若使用无限循环，在 update 恰好写到奇数序列后停止或极端争用时可能永久
  阻塞 action goal callback。
- 自主裁决：goal callback 最多尝试 1000 次取得序列号前后相同且为偶数的快照；未取得就以
  `FJT_REJECT reason=feedback_snapshot_unavailable` 拒绝。成功快照仍必须满足 age <= 500 ms 且
  每轴第一点误差 <= 1°。不在 RT 路径增加锁、日志、等待或分配。
- Benefit：任何控制环停顿都不会把 FJT 准入线程永久挂死；失败是明确的拒绝，旧轨迹不会被接受。
- Drawback：在极端调度争用下，即使下一瞬间就能获得一致快照，也可能产生一次保守的假拒绝；上层
  必须基于新状态重新发送目标，而不是假设 action server 会排队等待。

## BQ-086 — Humble launch 的 xacro 输出必须显式按字符串参数传递 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：生产 xacro 单独展开并通过 `check_urdf`，但首次 mock 启动在 launch 参数装配阶段失败：
  `launch_ros` 尝试把完整 URDF 当作 YAML 解析，并提示字符串参数应显式声明类型。
- 自主裁决：只在 `rt_control.launch.py` 中用 Humble 提供的 `ParameterValue(..., value_type=str)` 包装
  `Command(xacro ...)` 的结果，不修改 URDF、控制器参数或硬件选择逻辑。
- Benefit：消除不同 XML 内容触发 YAML 隐式解析的歧义，真实和 mock 启动共用同一条确定的字符串参数路径。
- Drawback：launch 文件增加一个 Humble API 依赖；升级 ROS 发行版时需确认该 API 的兼容性，但没有
  控制周期运行时开销。

## BQ-087 — 生产容器 PID 1 必须是失能包装脚本 [RESOLVED 2026-07-22]

- 状态：**RESOLVED — HIGH-RISK（生产停止信号链）**
- 证据：首次 mock 停止验证中，镜像默认命令是 `ros2 run rt_control_bringup rt_control_start`；容器
  PID 1 因而是 ROS CLI 启动器，不是安装后的 Bash 包装脚本。Docker SIGTERM 未触发包装脚本的 trap，
  40 s 后容器被 SIGKILL，退出码为 137，日志中也没有 `/rt/disable` 结果。
- 自主裁决：Docker CMD 直接执行 merge-install 下的
  `/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start`。entrypoint 最终 `exec` 该文件，
  使包装脚本成为 PID 1；调试时也必须直接运行该已安装文件，不再用 `ros2 run` 包一层。
- Benefit：INT/TERM 直接进入包装脚本，先完成有界 `/rt/disable`，再向 launch 进程组转发 SIGINT；Docker
  的 `stop_grace_period` 因此覆盖真正的硬件失能窗口。
- Drawback：CMD 与当前 `/opt/rt_control_ws/install` merge-install 路径绑定；若未来改变工作区安装前缀或
  包可执行文件布局，镜像会在启动时明确失败并需要同步修改。本路径不是硬件断电保证，实机终态仍由
  T-013 验证。

## BQ-088 — 上游补丁载荷的空白行不做机械改写 [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：`git diff --cached --check` 会把 ICube 补丁中表示上游源文件空白行的 `+ ` 报为 trailing
  whitespace；这些字符属于已在冻结提交上生成、应用、编译和测试过的补丁载荷，补丁 SHA-256 也已写入审计记录。
- 自主裁决：仓库自身源代码继续执行 `git diff --check`；补丁目录从该机械空白检查中排除，改用冻结上游
  HEAD、`git apply --check` 和已记录 SHA-256 三项联合校验。不为满足样式检查重写补丁载荷。
- Benefit：保留已验证补丁的逐字身份和可追溯哈希，同时仍对实际仓库源代码执行严格空白检查。
- Drawback：对整个暂存区直接运行无排除项的 `git diff --check` 会返回非零；审阅者必须使用文档中的分层
  命令，且未来重新生成补丁时仍需人工区分载荷空白与仓库源代码空白。

## BQ-089 — Compose 必须以仓库根目录作为 project directory [RESOLVED 2026-07-22]

- 状态：**RESOLVED**
- 证据：从仓库根直接执行 `docker compose -f docker/compose.yaml ...` 时，当前 Compose 将首个 compose
  文件所在的 `docker/` 作为默认 project directory，因而把 `dockerfile: docker/rt-control/Dockerfile`
  错解为 `docker/docker/rt-control/Dockerfile`；失败发生在构建上下文解析阶段，没有执行镜像步骤。
- 自主裁决：生产构建、config 和 up/down 统一使用 `tools/rt_control_compose.sh`；该冻结包装器显式传入
  `--project-directory .`、`--env-file versions.env`，并从 Git HEAD 生成镜像标签。不再把裸 `-f` 命令
  作为受支持入口。
- Benefit：无论调用者当前目录和 Compose 默认规则如何，context、Dockerfile、版本文件和镜像标签都绑定
  到同一仓库根目录。
- Drawback：操作者必须经过包装器并预先提供已在目标机验证的 `RT_CONTROL_CPUSET`；绕过包装器的临时调试
  命令需要自行完整复现 project-directory、env-file 和标签参数。

## BQ-090 — T-009 实机内核与 EtherCAT MAC 覆盖冻结初值 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 用户明确批准的实机覆盖**。
- 证据：`alfa-two` 实机运行 Ubuntu Pro `6.8.1-1056-realtime`，`/sys/kernel/realtime=1`；
  I210 `enp3s0` 的固化 MAC 为 `8c:59:3c:15:01:f8`。原 REQ-RT-002/REQ-ECAT-008 写的是
  5.15-rt 与旧 MAC `8c:59:3c:14:ff:d3`，在本机均不成立。用户已明确批准 HWE 6.8 RT 与新 MAC。
- Decision：只对 T-009 主机实装值应用上述覆盖；master ID 0、`enp3s0` 专用且无 IP/DHCP/DDS、
  IgH stable-1.6+ec_igb 与其余总线约束不变。旧值保留在原始证据文档中，不静默改写。
- Benefit：部署绑定实际硬件和已成功启动的受支持 Ubuntu Pro RT 包，不会把主站指向不存在的 MAC。
- Drawback：偏离原冻结主机版本，所有 out-of-tree 模块必须针对 6.8 RT 单独编译验证；换机不得复用该 MAC。

## BQ-091 — `alfa-two` 隔离 CPU 与 SMT 策略 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — HIGH-RISK（启动参数/调度拓扑）**。
- 证据：i7-14700 的 CPU 0-15 是八个双线程 P 核，CPU 16-27 是十二个单线程 E 核；
  CPU 14/15 同属 P-core 7。当前 cmdline 无 isolcpus/nohz_full/rcu_nocbs/irqaffinity。
- Decision：控制容器 cpuset 固定 CPU 14；启动参数采用
  `nosmt isolcpus=domain,managed_irq,14 nohz_full=14 rcu_nocbs=14 irqaffinity=0-13,15-27`；
  Controller Manager 使用已批准的 PROVISIONAL `SCHED_FIFO 80`。CPU 15 及其他 SMT 次线程由
  `nosmt` 下线，避免同物理核争用。
- Benefit：控制环独占完整 P 核，普通调度、IRQ、RCU callback 和 SMT sibling 均不与其竞争。
- Drawback：全机少八个 SMT 逻辑线程且 CPU 14 不供普通工作负载使用；需重启生效，换 CPU/BIOS 后必须重探测。

## BQ-092 — NVIDIA 595-open 强制 RT 构建与 PCIe 错误隔离 [RESOLVED/REVISED 2026-07-24]

- 状态：**RESOLVED — HIGH-RISK（官方不支持的 RT GPU + 实机 PCIe 异常）**。
- 证据：595 闭源和开源 DKMS 均以“does not support real-time kernels”拒绝 PREEMPT_RT；
  版本限定覆盖与 GCC 12 可成功构建 595-open，`nvidia-smi` 正常识别 RTX A2000。但加载后 AER
  correctable 总数约 80486174，主要为 RxErr，并含 BadTLP/BadDLLP。首次卸载模块后 5 秒增量为 0，
  但重启后在 NVIDIA 模块未加载时仍复现：GPU 位于 D0/Gen4 x8，3 秒新增 10414 个 correctable，
  HDMI 音频 function 仍绑定 `snd_hda_intel`。因此“仅卸载/屏蔽 NVIDIA 驱动即可止错”的早期结论
  已被实测推翻，驱动不是根因。
- Decision：保留 `/etc/dkms/nvidia-595.71.05.conf` 供可审计重建，但以
  `/etc/modprobe.d/nvidia-rt-quarantine.conf` 阻止 NVIDIA 自动/直接加载；GDM 使用 Intel i915。
  另外用 `gpu-pcie-quarantine.service` 在启动时严格校验 `01:00.1=10de:2291`、
  `01:00.0=10de:25b8`；全 PCI sysfs 预扫发现任一目标 ID 移到其他 BDF 时 fail closed，并校验独占上游
  根端口 `00:01.0=8086:a70d`，bus 01 出现任何非预期 child 同样拒绝执行。物理重新插拔后的复测推翻了
  “只移除两个 endpoint 足以止错”的早期观测：两个 endpoint 已从 sysfs 消失时，根端口仍持续收到 AER；
  改为移除经过完整拓扑校验的独占根端口后，AER 与日志增长才同时停止。EtherCAT、SSH、GDM/i915 均保持正常。
  不用 `pci=noaer` 或关闭 AER 隐藏问题。
  物理链路/供电检查和受控 Gen3 复测通过前不得解除两层隔离。
- 启动时序修正：首次持久服务验证虽然最终摘除设备，但因错误地排在 `systemd-udev-settle` 之后，启动早期
  仍产生 389349 条 AER 日志，属于不合格证据。unit 改为 `DefaultDependencies=no`、由 `sysinit.target`
  拉起、仅等待根文件系统 remount，并排在 `systemd-udev-trigger` 之前；这样在任何 udev 驱动绑定前先完成
  全 PCI ID/BDF 校验和移除。第二次重启中 unit 在 userspace 约 0.47 s 开始、107 ms 完成，启动时间由
  约 86 s 降至 52 s，移除后根端口 15 s 增量为 0；但该 boot 仍产生 127072 条 AER 日志，证明错误已在
  PCI 枚举/内核早期大量形成。服务保留为部分 containment，**不得据此宣称 GPU/RT 主机生产就绪**；
  必须断电检查插接/载板/供电，或 BIOS 强制 Gen3 后重测。用户态继续提前已无可信收益，且仍禁止
  `pci=noaer` 掩盖。重新插拔后的受控 PCI rescan 仍以 Gen4 x8 建链，2 秒产生 2360 条 AER 匹配日志，
  明确为 correctable Physical Layer `RxErr`，因此插拔动作未修复链路；下一硬件试验必须是 BIOS 固定 Gen3
  后冷启动复测，或交叉更换载板/插槽/显卡。
- 后续用户报告已更换接口/Gen3 后的冷启动，实机仍枚举为同一 `00:01.0 -> 01:00.x` 拓扑；启动隔离前产生
  2462 条 AER 匹配日志。受控 rescan 显示 GPU `LnkCap=16GT/s x16`、`LnkSta=16GT/s x8`，根端口
  `LnkCap=32GT/s x16`、`LnkSta=16GT/s x8`，因此实际仍是 **Gen4 x8**，不是 Gen3。短窗口继续产生
  2295 条 correctable Physical Layer `RxErr`，VGA 与 HDMI-audio function 均有报告，随后根端口重新隔离。
  本次证据只说明更换动作未改变可见 BDF/协商代际，**不得误写成“Gen3 也失败”**；只有现场读到
  `LnkSta: Speed 8GT/s` 后，才开始正式 Gen3 AER 验收。
- 2026-07-24 用户最终裁决：**本台 rt-control 工控机不再使用 NVIDIA GPU**。因此取消 Gen3、驱动加载、
  CUDA 与 GPU-load 验收，不再以“修复后可用 GPU”作为 T-009 前提；但这不等于允许带故障卡参加最终 RT
  验收。断电物理拆除前继续保留 modprobe 与根端口两层隔离，只允许在确认 `00:01.0` 已摘除后进行受控
  EtherCAT/CAN bring-up；最终 30 分钟 250 Hz/cyclictest 必须在故障卡物理移除、冷启动 NVIDIA 链路 AER
  为零后执行。Benefit：彻底消除已知 AER 风暴和官方不支持的 NVIDIA+PREEMPT_RT 组合对实时/日志的影响。
  Drawback：本 IPC 永久不提供 CUDA、NVIDIA 显示或 HDMI 音频；需要 GPU 的感知负载必须迁移到其他硬件。
- Benefit：真正停止会污染日志和实时性的错误风暴，同时保留已安装驱动及明确恢复路径；设备 ID 校验可避免
  PCI 地址变化时误删其他设备。
- Drawback：隔离期间无 NVIDIA 计算、HDMI 音频和独显输出；服务依赖本机固定 BDF，目标 ID 出现在其他 BDF
  时会失败而不猜测；
  即使 PCIe 修复，595+PREEMPT_RT 仍不受 NVIDIA 官方支持，必须以 idle/GPU-load cyclictest 对照决定
  是否可用于生产。重新插拔后的 endpoint-only 隔离失效使当前 boot 出现 8112545 条 AER 匹配记录，
  `syslog`/`kern.log` 各增至约 82 GB、`/var/log` 共 169 GiB。用户明确要求删除测试产生的大日志后，已在
  根端口止错条件下清空活动文件、删除八个超过 100 MB 的 AER 膨胀轮转文件，并执行 journal vacuum。
  随后用真实 PCI rescan 验证新版完整拓扑校验/根端口移除脚本：service exit 0、根端口与 NVIDIA endpoint
  均消失，短暂枚举产生的 1655 条 AER 匹配日志也已清理。最终 journal 占 40 MB、`/var/log` 占 602 MB，
  根分区可用 371 GB（17% used），活动日志 5 秒无增长。代价是混入这些文件的旧日志也被删除，故关键诊断
  数字保留在本裁决记录中。

## BQ-093 — `alfa-two` Docker 来源、权限与受限网络部署 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 自主部署裁决（安全边界已记录）**。
- 证据：实机为 Docker 官方支持的 Ubuntu 22.04 amd64，安装前无 `docker.io`、`containerd`、`runc`
  等冲突包，UFW 未启用。工控机直连 `download.docker.com` 被重置，Docker Hub 请求超时；WSL
  `127.0.0.1:10808` 可访问官方源。官方文档要求生产机使用签名 apt repository，而 convenience script
  只建议测试/开发使用。
- Decision：使用 Docker 官方签名 apt repository，安装精确实测版本 Docker CE 29.6.2、containerd 2.2.6、
  Buildx 0.35.0、Compose 5.3.1；不使用 convenience script，也不执行系统整体升级/自动清理。下载时仅建立
  工控机 `127.0.0.1:18080` 到 WSL 代理的临时 SSH reverse forward，完成后关闭，不写入 apt、daemon、
  镜像或仓库。保持 rootful Docker，但不把 `alfa` 加入等同 root 权限的 `docker` 组，生产操作显式使用
  `sudo`。通过 FIFO+SSH 流式导入已验证镜像，不落大 tar 文件。对 Docker CE/CLI、containerd、Buildx、
  Compose 和 rootless extras 设置可逆 apt hold；维护升级必须先 unhold，完成复测后重新 hold。
- Benefit：软件来源、签名、确切版本和回滚包边界可审计；没有持久代理或扩大的本地用户权限；离线导入的
  image ID 与 WSL 完全一致。
- Drawback：`alfa` 不能直接操作 Docker socket，命令需 `sudo`；工控机尚不能直接从 Docker Hub 拉取，
  后续升级/新镜像仍需临时受控传输。Docker 发布端口会改变防火墙路径，未来若新增端口必须在
  `DOCKER-USER` 链单独审查；当前 Compose 使用 host network 但未声明发布端口。apt hold 同时阻止自动获取
  Docker 安全更新，需纳入显式维护窗口，不能无限期不审查。

## BQ-094 — IgH 安装脚本在 `ec_igb` 已接管 I210 后的幂等校验 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 实机复查发现的脚本缺陷修正**。
- 证据：首次启动成功后 `ec_igb` 接管 I210，普通 netdev `enp3s0` 不再出现在 `/sys/class/net`；旧脚本
  仅从该路径读 MAC，因此合法复跑会在构建前误报接口缺失。当前主站仍能给出精确
  `Main: 8c:59:3c:15:01:f8 (attached)`，且 `/etc/ethercat.conf` 保存同一 MAC。
- Decision：首次安装继续以 netdev MAC 精确匹配且无 IPv4/IPv6 为门禁；接口已消失时，只有
  `ethercat.service` active、既有 CLI 可执行、配置行逐字匹配、主站报告同一 MAC attached 四项同时成立，
  才认定是 `ec_igb` 合法接管并允许复建。脚本不为幂等校验自动停止主站；显式 `--start` 才在安装末尾
  重启服务，已接管路径跳过无意义的 `nmcli device set`。
- Benefit：内核更新后的重复构建不再误失败，同时不会为了检查而中断可能在线的 EtherCAT 主站，也不会把
  “接口消失”宽松解释成任意硬件状态。
- Drawback：已接管路径依赖既有 IgH CLI 与配置作为交叉证据；若旧安装损坏到无法提供四项证据，脚本会
  fail closed，需要维护窗口中先恢复 netdev/服务，而不会自行卸载模块猜测处理。

## BQ-095 — CPU governor 在预备延迟通过后的保留策略 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 自主实时调优裁决**。
- 证据：保持实机默认 `intel_pstate`、`powersave` governor、`balance_performance` EPP，不修改 C-state 或
  RT runtime；CPU14 上以 FIFO90、1 ms interval、mlockall 运行 30 分钟宿主 preliminary cyclictest，
  1,800,000 周期结果为 Min 1 / Avg 4 / Max 18 µs，低于 100 µs 门槛；期间无调度节流、RCU stall、
  lockup、AER 或内核 warning。
- Decision：保留默认 governor/EPP，不为进一步压低预备数字强制全机/单核 performance，也不改 C-state
  和 `sched_rt_runtime_us`。该结果只证明空闲宿主基线，不能替代实总线 250 Hz 控制空跑的最终 30 分钟验收。
- Benefit：已经满足基线门槛，同时减少常驻功耗、热量和因温度降频带来的长期不确定性，避免引入规格外启动参数。
- Drawback：真实感知/控制负载下的最坏延迟仍未知；若最终联合验收超过 100 µs，必须基于 trace/IRQ/热数据
  重新裁决，不能把本次 18 µs 当作生产保证。

## BQ-096 — 最终“容器内 cyclictest”工具注入方式 [RESOLVED 2026-07-23]

- 状态：**RESOLVED — 不修改冻结生产镜像/Compose 的验收工具方案**。
- 证据：生产镜像未内置 `cyclictest`，但已有兼容 `libnuma`；宿主提供 rt-tests 2.2-1 / cyclictest 2.20。
  以同一生产 image ID 启动一次性诊断容器，只读绑定 `/usr/bin/cyclictest`、映射
  `/dev/cpu_dma_latency`、CPU14、FIFO90、`SYS_NICE`/`IPC_LOCK` 与相同 ulimit，5 秒 5,000 周期结果
  Min 1 / Avg 3 / Max 12 µs，并确认 PM QoS 设为 0 µs。
- Decision：最终联合验收使用上述独立诊断容器，与 250 Hz 生产容器同时运行；不向生产镜像安装 rt-tests，
  不给冻结 Compose 增加诊断 device/volume。证据必须同时记录生产 image ID、宿主 rt-tests 包版本和完整命令。
- Benefit：验证发生在容器调度/namespace 路径中，又不污染生产镜像或永久扩大其设备权限；工具可独立移除。
- Drawback：工具二进制来自宿主而非镜像供应链，且 FIFO90 的诊断线程会短暂抢占优先级80的控制环；这正是
  联合延迟压力的一部分，但最终结果必须和 250 Hz overrun/总线状态一起解释。

## BQ-097 — CANable 2.0 的 SLCAN commissioning 与生产 `gs_usb` 路径 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — HIGH-RISK（需现场刷写适配器固件，尚未执行）**。
- 证据：实机 USB `16d0:117e`、序列号 `208031C05230` 运行 CANable 2.0 原厂 SLCAN 固件；现场手工进程为
  `slcand -o -c -s6 /dev/ttyACM0 can0`。官方 CANable 命令表确认 `S6=500 kbit/s`；被动监听收到
  `0x701/0x702/0x703/0x714`，数据均为 `0x7F`，SocketCAN 错误计数和 TX 均为 0。其中 `0x714` 是权威映射
  已登记的 Node 20 Pitch 只读诊断心跳，不进入生产配置。SLCAN 在适配器内设置 bit timing，Linux 虚拟
  interface 因而报告 `bitrate 0`，且手工 daemon 随桌面会话存在；这不满足 REQ-CAN-001/REQ-DEP-003 的
  宿主 systemd 和 `bitrate 500000` 可核验证据。
- Decision：SLCAN 仅允许继续做不发帧的 commissioning 观察，不改冻结 `hostsetup/can0.service` 去迁就它。
  生产路径采用厂家官方支持的 CANable 2.0 candleLight 固件，使设备由内核原生 `gs_usb` 驱动；RT 内核
  `6.8.1-1056-realtime` 已验证存在 signed in-tree `gs_usb.ko` 且 `CONFIG_CAN_GS_USB=m`。刷写后继续使用
  冻结 unit 通过 netlink 设置 500000，并重新验证设备固定命名、`ip -details`、心跳和零错误。刷写需要现场
  BOOT 按键/拔插配合，当前尚未执行；不得把本次 SLCAN 观察写成 T-009 完成。
- Benefit：回到原生 SocketCAN/systemd/可观测 bitrate 的冻结架构，绕过 `slcand` userspace 串行转发，
  官方说明在高负载下性能更好。
- Drawback：固件刷写有中断和恢复风险，需要现场物理操作；CANable 2.0 candleLight 不支持 CAN-FD，但本项目
  仅使用经典 CAN。依据：`https://canable.io/getting-started.html`、
  `https://github.com/normaldotcom/canable2-fw`。

## BQ-098 — low-latency 迁移后的 NVIDIA PCIe 隔离解除门禁 [BLOCKED 2026-07-24]

- 状态：**BLOCKED — HIGH-RISK（需要用户明确批准一次无隔离启动）**。
- 新裁决与旧裁决关系：用户随后明确选择 Ubuntu HWE low-latency 迁移以恢复 NVIDIA 能力，这覆盖 BQ-092
  中“本 IPC 永久不使用 GPU”的软件部署范围；但它没有证明此前的 Gen4 x8 Physical Layer `RxErr` 已修复，
  也没有自动授权解除为防止 169 GiB 日志膨胀而建立的硬件隔离。
- 已完成证据：移除损坏的 `nvidia-dkms-535`/`nvidia-driver-535` 后，安装并一次性启动
  `6.8.0-136-lowlatency`；`CONFIG_PREEMPT=y` 且无 `CONFIG_PREEMPT_RT`，CPU14 隔离参数原样继承，
  realtime `6.8.1-1056` 与 generic `6.8.0-134` 回退内核均保留。IgH stable-1.6 冻结提交
  `2f7f884f1c7d377c02a7d627eb06512126a0e50e` 已针对新 ABI 重建，`ec_igb` 报告主设备
  `8c:59:3c:15:01:f8 (attached)`；当时现场链路为 DOWN、从站 0 个。安装了 Ubuntu 仓库的
  `nvidia-driver-595-open=595.84-0ubuntu0.22.04.1` 与精确 ABI 的
  `linux-modules-nvidia-595-open-6.8.0-136-lowlatency`，APT 模拟和实装均未引入 DKMS；模块 vermagic 与
  当前内核一致。NVIDIA 官方说明 Turing 及更新架构应使用 open 模块，CUDA 13.2 Update 1 要求驱动至少
  595.58.03，因此 595.84 满足 RTX A2000/CUDA 13.2 软件条件。
- 接口名现场修正：移除旧 ABI 的 `ec_igb` 占用后，权威 MAC `8c:59:3c:15:01:f8` 在 sysfs 唯一对应
  `enp2s0`；`enp3s0` 实际 MAC 为 `8c:59:3c:15:01:f9`。旧记录把 f8 与 `enp3s0` 并列是误判。
  `hostsetup/igh-install.sh` 改为按权威 MAC 唯一解析当前 netdev 名称，发现零个或多个匹配均 fail closed；
  `/etc/ethercat.conf` 仍只以冻结的 `MASTER0_DEVICE` MAC 绑定，不依赖可能随枚举变化的接口名。
- 未通过门禁：`gpu-pcie-quarantine.service` 与 `/etc/modprobe.d/nvidia-rt-quarantine.conf` 仍启用。
  当前内核的受控全局 PCI rescan 没有重新枚举已摘除的 `00:01.0` 根端口，因此 AER 无新增但也未见 GPU，
  该结果只能判定为“未枚举”，不能判定硬件恢复。必须通过一次禁用根端口隔离的重启才能验证
  `lspci`、协商代际、AER 增量和 `nvidia-smi`。
- 用户批准与收紧执行：用户明确批准一次无隔离测试启动。为避免 SSH 恢复前重现持续风暴，实际采用更窄的
  一次性 sysinit 测试窗口：保持 NVIDIA modprobe 禁载，在原隔离服务时序中先保留设备 2 秒、记录 BDF/
  link/AER，再自动调用原脚本摘除根端口并删除测试 drop-in。Benefit：测试窗口有硬上限且自动恢复；
  Drawback：只能采集早期物理层证据，不能在同一次启动直接运行 `nvidia-smi`。
- 测试结果：`6.8.0-136-lowlatency` 于 21:01 启动，但测试包装器第一次读取时
  `00:01.0/01:00.0/01:00.1` 就全部不存在，2 秒后仍不存在；AER 匹配数保持 `4 -> 4`、增量 0。
  包装器成功报告 `root_port_removed=yes` 并自删除 drop-in，日志未失控（`/var/log` 约 855 MiB、journal
  约 288 MiB），系统与 IgH 无 failed unit。该结果是“设备未枚举”，不是“链路通过”；软重启没有恢复此前
  已从 sysfs 摘除的根端口，故不能解除 modprobe 禁载或测试 `nvidia-smi`。
- 剩余问题：RTX A2000 当前是否仍物理安装？若已拆除，本轮 GPU 迁移在硬件门禁处终止；若仍安装，需要现场
  完整断电冷启动（不是 `systemctl reboot`）后重跑同一有界窗口，才能判断物理链路/AER。
- 用户纠正与冷启动证据：用户说明 21:01 测试时显卡实际未安装，随后断电插入并冷启动。默认 GRUB 回到
  `6.8.1-1056-realtime`；隔离服务执行前本次 boot 记录 2,850 条 AER 匹配信息，仍为
  `01:00.0` correctable Physical Layer `RxErr`，随后根端口被成功摘除。插卡还触发 BIOS 自动图形策略：
  冷启动后的 PCI 树完全没有原先的 Intel `00:02.0`，`i915` 无设备可加载；NVIDIA 又被隔离，导致系统最终
  没有任何 DRM GPU。`/dev/dri/card0` 是无 sysfs 后端的 `root:root 0600` 残留节点，GDM/Xorg 因此报
  `Permission denied`、`no primary bus or device found` 与 `no screens found`。这解释了图形界面失败，
  不是解除 modprobe 禁载的理由。
- 新物理门禁：先在 AMI BIOS `RXE26005` 中把集成显卡从 Auto 改为强制 Enabled（常见名称为 Internal
  Graphics/IGD/iGPU Multi-Monitor），并把主显示设为 IGD/IGFX、显示器接主板接口；确切菜单名称由该 OEM
  BIOS 决定，若现场页面不同必须提供照片，不能猜测。只有冷启动后重新出现 `00:02.0`、`i915` 和有效
  DRM 节点，才允许恢复 GDM。NVIDIA 隔离保持不变，因为本次冷启动已经复现硬件错误。
- 暂行决定：确认硬件状态并完成必要冷启动前，不解除 modprobe 隔离、不把 low-latency 写成永久 GRUB 默认，
  也不宣称 GPU 可用。
- Benefit：内核、IgH 与 NVIDIA 软件栈已经可回退地准备完成，同时避免未经授权重现已知 PCIe 风暴。
- Drawback：当前启动是一次性 low-latency；下一次普通重启仍回到 realtime，且隔离期间 `nvidia-smi` 必然
  不可用。完整迁移和 GPU/负载延迟验收仍未完成。

## BQ-099 — T-009 更换到原冻结硬件主机后的覆盖边界 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 用户授权配置新主机后的实施裁决**。
- 证据：新目标 `ar-Default-string`（`192.168.0.40`）实际运行
  `5.15.0-1032-realtime`，EtherCAT I210 MAC 为冻结值
  `8c:59:3c:14:ff:d3`，15 个从站全部可见；CPU 同为 i7-14700，CPU 14/15
  是一个完整 P-core。RTX 2000 Ada 正常工作且无 AER/Xid。BQ-090、BQ-092、
  BQ-098 的 6.8/new-MAC/GPU-quarantine 证据均明确属于已放弃的 `alfa-two`。
- Decision：新主机恢复原 REQ-RT-002/REQ-ECAT-008 的 5.15 PREEMPT_RT 和
  `8c:59:3c:14:ff:d3`，仍用固定 IgH stable-1.6 提交与 `ec_igb`。同拓扑下
  生产 cpuset 选 CPU 14，并采用已批准的 `nosmt` whole-core 参数；原主机已有的
  C-state=1 参数只保留、不由本任务新增。`alfa-two` 的 GPU 隔离脚本不安装到新机，
  但专有 NVIDIA 模块下的最终 GPU/MoveIt 联合延迟仍必须实测。
- Benefit：主机重新与原冻结内核和硬件身份对齐，避免把 `alfa-two` 的异常与覆盖
  污染到健康主机；CPU14 具有完整物理核隔离。
- Drawback：换机使先前 `alfa-two` 的 Docker/IgH/延迟证据不能直接充当本机验收；
  `nosmt` 会移除八个逻辑线程，NVIDIA 专有模块仍会 taint 实时内核。

## BQ-100 — 两只同型号 CANable 的生产接口身份 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 实机枚举事实下的 fail-closed 绑定**。
- 证据：两只 `1d50:606f` CANable 2.0 均由 `gs_usb` 驱动。当前 `can0` 序列号
  `004D00675230500720333159` 正在收发 rt-control Node 1/2/3 数据；当前 `can1`
  序列号 `003000265230500720333159` 由现有 BMS 节点使用。仅靠内核枚举顺序会在
  换 USB 口或启动时序变化后交换名称。
- Decision：用一次性 systemd 命名服务按两个精确 USB 序列号事务式重命名：前者
  固定 `can0`，后者固定 `can1`，再由冻结 `can0.service` 设置 500 kbit/s。任一
  设备缺失、重复或序列号变化均拒绝启动，不猜测另一只适配器。命名过程保留执行前
  的 UP/DOWN 状态，避免实装检查额外改变 BMS 链路状态。
- Benefit：彻底消除同型号适配器枚举交换导致向错误物理总线发控制帧的风险。
- Drawback：两个指定适配器成为启动依赖；维护更换 CANable 后必须人工更新并重新
  审核序列号，不能自动接受新硬件。

## BQ-101 — Jammy AppArmor 不识别 Cursor 的 `userns` 规则 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 宿主安全服务修复**。
- 证据：`apparmor.service` 唯一失败原因是
  `/etc/apparmor.d/cursor-sandbox:11` 的 `userns,`；Ubuntu 22.04 自带解析器
  报 `unexpected TOK_END_OF_RULE`，导致整次 profile reload 非零。该 profile 自
  2026-01-23 起未成功加载，而不是 rt-control/Docker 新引入的问题。
- Decision：备份原 profile，只注释 Jammy 不支持的 `userns,` 一行并保留其余规则，
  以 `apparmor_parser` 解析成功和 `apparmor.service` active 为门禁。不禁用整个
  AppArmor 服务，也不为 rt-control 添加宽松 profile。
- Benefit：恢复全机 AppArmor profile 的正常加载，优于继续容忍失败服务或整体禁用。
- Drawback：Cursor profile 在本机不包含较新 AppArmor 的 user-namespace 专用规则；
  升级到支持该语法的系统后需要重新评估，而不能永久沿用注释。

## BQ-102 — IgH 1.6 默认允许实时上下文 syslog [RESOLVED 2026-07-24]

- 状态：**RESOLVED — REQ-RT-004 的直接实施约束**。
- 证据：固定 IgH 1.6.10 提交的 `configure` 默认报告
  `whether to syslog in realtime context... yes`，并在 `master/domain.c`、
  `master/master.c` 和 datagram 路径通过 `EC_RT_SYSLOG` 编译实时日志。冻结规则 9
  明确禁止 RT read/update/write 路径日志；宿主模块无需实时日志才能提供已有的非 RT
  启动/状态变化证据。
- Decision：宿主 IgH 构建显式使用 `--disable-rt-syslog`，容器用户态库继续保持同一
  1.6.10 ABI/提交。首次发现默认值时尚未进行任何运动或激活应用；随后立即用该开关
  重建并重启空闲主站，以最终模块配置为验收对象。
- Benefit：消除内核实时数据路径调用 syslog 的延迟和分配风险，直接满足 REQ-RT-004。
- Drawback：少数只在 `EC_RT_SYSLOG` 下产生的运行期详细日志不可用；故障定位依赖已冻结
  的主站状态、WC/diagnostics 与 commissioning CLI，而不能临时在生产模块中打开实时日志。

## BQ-103 — Docker 安装脚本复跑的网络与包升级边界 [RESOLVED 2026-07-24]

- 状态：**RESOLVED — 实机幂等性复跑发现并收紧**。
- 证据：Docker 已按固定指纹和版本成功安装后，第一版脚本复跑仍重新下载官方 GPG key；本机到
  `download.docker.com` 的 TLS 连接瞬时 reset，使脚本退出。此前置步骤还因普通
  `apt-get install` 语义把已安装的 `curl`、`libcurl4`、`libcurl4-openssl-dev` 从 Ubuntu
  `.23` 升到 `.25`。Docker、EtherCAT、CAN 和启动配置均未改变，但这证明原脚本不满足“复跑不扩大变更”。
- Decision：若 `/etc/apt/keyrings/docker.asc` 已存在，则离线读取并逐字核对冻结指纹后复用，绝不为复跑
  再下载；仅在文件缺失时通过 TLS 获取。前置包安装增加 `--no-upgrade`，Docker 六包继续精确版本安装和
  `apt-mark hold`。修正版实机复跑退出 0，零升级、零新装、零卸载。已有 RealSense 源缺 key
  `FB0B24895113F120` 的 apt 警告不属于 rt-control，不越权修复。
- Benefit：断续网络下已配置主机仍可验证并复用可信 key，维护复跑不会顺带升级宿主依赖；固定版本与 hold
  继续阻止未验漂移。
- Drawback：Docker 官方未来轮换签名 key 或发布安全更新时脚本会 fail closed，必须人工审核并更新冻结指纹/
  版本；本次意外发生的三个 Ubuntu curl 包升级不可无损回滚，已作为部署事实保留。
