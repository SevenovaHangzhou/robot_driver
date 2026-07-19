# Blocked questions

Only tasks listed under each question are blocked. Unrelated tasks continue in unattended mode.

## BQ-001 — T-005 sanctioned legacy deltas

- Evidence: `/home/kkozia/robot_driver@6bc94cd` has Ti5 RxPDO/TxPDO `0x1600/0x1A00`, extra velocity/effort/mode/gap channels, and no `0x60C2` startup SDOs. Frozen REQ-ECAT-002/004 and the hardware mapping require position-only `0x1601/0x1A01` plus `0x60C2:01=4`, `0x60C2:02=-3`.
- Conflict: REQ-MIG-001 requires `diff_legacy.py` to report 100% against that named legacy baseline, but faithfully implementing the higher-priority frozen mapping necessarily produces semantic differences.
- Minimal question: Should T-005 (a) apply an explicit frozen-requirement allowlist/overlay to the `6bc94cd` baseline before enforcing 100%, or (b) compare against a separately approved normalized baseline snapshot? No policy is selected here.
- Blocked scope: T-005 completion and any claim that `diff_legacy.py` is a 100% gate. T-004 file generation may proceed from frozen requirements, but it cannot satisfy its final diff gate yet.

## BQ-002 — ZeroErr preload with the zero-patch gate design

- Evidence: local upstream `EcCiA402Drive` writes the target-position default only when `mode_of_operation_display_ != MODE_NO_MODE` and does not gate `0x000F` on a completed write. Frozen ZeroErr TPDO has no `0x6061`; the authoritative fork solved this with a source patch, while AMB-007 requires a zero-patch target and an `enable_manager` that claims only `control_word`/`status_word`.
- Conflict: the inspected upstream code does not prove REQ-ECAT-005 (`0x607A=0x6064` before `0x000F`) for ZeroErr, and the specified controller interfaces cannot themselves write/read the raw target/actual pair.
- Minimal question: What approved mechanism closes the ZeroErr preload gate: authorize the already-reviewed minimal upstream patch, add explicitly authorized raw preload interfaces to `enable_manager`, or provide another specified mechanism? No option is selected here.
- Blocked scope: T-010 safety completion and T-013 enable authorization. Configuration and non-enable tasks continue.

## BQ-003 — CANopen operation-mode command ownership

- Evidence: the exact design skeleton requires `operation_mode: 1/3/3` in `bus.yml`, but pinned ros2_canopen Humble `Cia402System` does not consume that key. It changes mode only through edge-triggered `position_mode_cmd`/`velocity_mode_cmd` hardware command interfaces.
- Conflict: REQ-CAN-002 freezes PP/PV modes, but no specified controller owns the three mode-command edges or confirms them before target commands.
- Minimal question: Which approved component owns and sequences the ros2_canopen mode command interfaces for nodes 1/2/3? No owner is selected here.
- Blocked scope: T-007 CANopen controller activation wiring and T-014 hardware activation. The T-006 bus template retains the mandated keys and capability report records that they are not runtime-active.

## BQ-004 — Updown 0x6081 command ownership and ordering

- Evidence: the EDS maps 0x6081 in RPDO2, and ros2_canopen exposes it only through generic `tpdo/index`, `tpdo/subindex`, `tpdo/data`, `tpdo/ons` one-shot interfaces. Its typed velocity interface means a velocity-mode target, not PP profile velocity.
- Conflict: AMB-015 requires each updown move to carry target position plus 0x6081 maximum velocity, but the specification does not assign ownership or ordering of the generic PDO write relative to the position command.
- Minimal question: Which approved component must claim and atomically sequence the generic 0x6081 one-shot with each updown position move? No controller/adapter is selected here.
- Blocked scope: T-007 final updown controller wiring and T-014 updown motion. Other controller configuration continues.

## BQ-005 — CANopen frozen activation/PP semantics versus stock Motor402

- Evidence: stock `Motor402::handleInit()` enables before PP/PV preload, uses an unconditional fault-reset arm, and does not implement the frozen retry/deadline sequence. Stock `ProfiledPositionMode` waits on statusword bit 12 and does not guarantee the required one-cycle `0x003F` then `0x000F` pulse.
- Conflict: these behaviors are not equivalent to REQ-CAN-004/005, and bus configuration alone cannot change them.
- Minimal question: What implementation vehicle is authorized for the frozen CANopen activation and PP semantics: a downstream custom SystemInterface/controller, a minimal pinned ros2_canopen fork patch, or another specified mechanism? No option is selected here.
- Blocked scope: T-014 hardware activation and all CANopen motion. Configuration, archive tooling, and source capability reporting continue.

## BQ-006 — CANopen freshness and EMCY exposure to watchdog/diagnostics

- Evidence: ros2_canopen internally detects heartbeat loss and queues EMCY/RPDO events, but pinned `Cia402System` exports only latest NMT state and latest RPDO value; it registers no EMCY callback and exposes no monotonic heartbeat/PDO receive timestamp.
- Conflict: AMB-009 assigns 4000 ms heartbeat and 3000 ms PDO age decisions to `rt_watchdog`, while REQ-CAN-006 requires those ages and no-EMCY admission. The specified public interfaces cannot supply the inputs.
- Minimal question: What approved boundary exposes heartbeat receipt time, PDO receipt time, and EMCY state to `rt_watchdog`/`rt_diagnostics` (for example an authorized downstream SystemInterface extension, a pinned fork interface, or another specified source)? No mechanism is selected here.
- Blocked scope: T-011 CANopen freshness branches, T-012 CANopen diagnostics, and T-014 hardware activation. Domain-heartbeat watchdog work and non-CAN diagnostics may continue.

## BQ-007 — Docker Compose frozen fields versus build/version wiring

- Evidence: REQ-DEP-001 requires the design §7 Compose fields verbatim and forbids adding/removing fields. That fragment is stored at `docker/compose.yaml` but uses `build.context: .`, Dockerfile `docker/rt-control/Dockerfile`, and volume `./docker/cyclonedds.xml`, which resolve incorrectly unless the project directory is forced to the repository root. The same spec requires `versions.env` to be the single IgH version source and the image tag to equal the git SHA; normal Compose wiring needs additional `build.args` and `image` fields that the frozen list does not contain.
- Conflict: either editing the Compose field set or relying on an external wrapper/build command is a deployment-contract choice not covered by the specification. The unresolved T-009 CPU set also cannot be committed as a literal.
- Minimal question: Should T-008 authorize adding `build.args`, `image`, and an external cpuset substitution to `compose.yaml`, or must the Compose fragment remain byte-for-field equivalent and use a separately specified wrapper command/project-directory convention? No option is selected here.
- Blocked scope: T-008 completion and image-build claim. Other source/configuration tasks continue; no privileged configuration was created.

## BQ-008 — T-003 named URDF feature source is unavailable

- Evidence: the supplied GitHub path for `feature/joint-naming-unify-underscore-20260718` returns 404, and that branch/ref is absent from every local `alfa_robot` clone inspected. The available local package is on commit `948689c59fb` and still contains mixed legacy joint spellings such as `left_v5_joint1` and `leftjoint1`, so it is not equivalent to the explicitly named joint-normalization source.
- Conflict: T-003 requires the named upstream URDF as its migration input. Selecting the older local package or inventing the intended rename would be an uncovered source-of-truth decision.
- Minimal question: Please provide the exact feature-branch commit/archive (or restore access to it), or explicitly authorize a particular local commit/package as the T-003 source.
- Blocked scope: T-003 and description-dependent final launch validation. No substitute URDF was generated.

## BQ-009 — T-004 slave-profile count and filename mapping conflict

- Evidence: T-004 requires exactly 10 `slaves/*.yaml`, while spec §4 names `{zeroerr_j1..j5,left_j6,right_j6,ti5_j2,ti5_j3,turn}.yaml`. Expanding that set labels J2/J3 as both ZeroErr and Ti5, contradicting the hardware authority (J2/J3 are Ti5 only). The authoritative xacro actually uses eight profiles: ZeroErr J1/J4/J5/left-J6/right-J6/turn and Ti5 J2/J3; its ninth source file is an unused generic `joint6.yaml`. Its Ti5 J2 and J3 files are distinct but each is shared unchanged by left and right axes.
- Conflict: retaining shared profiles yields eight active files; splitting both Ti5 profiles per physical axis yields ten, but that changes the specified filename set and duplicates identical stored configuration. Retaining or repurposing the unused generic J6 profile would create an unreferenced or hardware-inapplicable artifact. The specification does not choose among these structures.
- Minimal question: Please freeze the exact ten target filenames and the 13-axis-to-filename mapping, or authorize the eight-profile mapping already proven by the authoritative xacro.
- Blocked scope: T-004 and its dependent T-005/T-007/T-010/T-013 tasks. No EtherCAT numeric file was generated from an inferred mapping.
