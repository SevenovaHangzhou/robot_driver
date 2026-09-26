---
id: motion-20260925-01
area: motion
title: ELECTRI-133 Tier A relative chassis execution core and no-device mock
date: 2026-09-25
type: feature
trigger: ELECTRI-133 user-authorized Tier A; component integrations ELECTRI-144/ELECTRI-146
commits: [feature/electri-133-tier-a]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-144, BQ-145, BQ-150, contract-20260925-01]
---

## 背景

ELECTRI-133 authorizes one RT-private relative move in the execution-start base frame,
with translation <=0.700 m, yaw <=15 degrees and translation speed <=0.2 m/s.
The existing Kinco backend is draft. This delivery exercises the execution contract
offline without opening production runtime or creating a device writer.

## 改动

Integrated the ELECTRI-146 fixed-size SE(2) planner into `swerve_math`; added
`RelativeMoveSession` and a separate `chassis_relative_move_mock` executable in
`swerve_driver`. Session state covers confirmed stationary mode handoff, admission,
bounded steering alignment, common-scalar CSP travel, deceleration, encoder settling,
first-fault latch and explicit healthy stationary reset. Goal/cancel slots have capacity
one; ROS work stays outside the fixed-size core call in a serialized mock callback group.
The existing `SwerveController` CSV implementation and production launch were unchanged
by the original offline delivery. The mock discards all Twist messages and does not
implement NAVIGATION velocity tracking. NAVIGATION collapses TRANSIT and DOCKING solely
in this offline mock; their separate production behavior is neither implemented nor
validated, and full production mode coverage is not claimed.

The ELECTRI-145 follow-up now adds an opt-in real `SwerveController` adapter around that
same session. It claims loaned wheel position/velocity/mode plus sequence/mask command
interfaces and corresponding actual/sent-sequence/readback state from the
`KincoCyclicModeSlave`; it does not claim controlword. The local velocity subscription is
solely `/cmd_vel`. The default `relative.enabled=false`, draft machine profile and absent
production launch wiring keep runtime admission closed.

The locally cataloged Kinco FD low-voltage servo manual
`V3-DOC-2026-014` (SHA-256 `029f858f7e7a4ec1f1299ae47bb6c9d1dcee8de3ef035517e1479d7166d0125e`)
provides the protocol basis: printed page 150 lists variable PDO mappings that include
`0x607A`, `0x60FF`, `0x6060`, `0x6064`, `0x606C` and `0x6061`; printed page 154 defines
CSP=8, CSV=9, `0x6060` as the requested mode and `0x6061` as the effective mode. User
confirmation establishes that the installed drive supports runtime CSV/CSP switching.
The manual does not freeze this machine's PDO assignment, transition timing or calibrated
thresholds, so the implementation still requires sent-cycle and all-four readback gates
and keeps the machine profile draft.

The connected sequence freezes new navigation commands, ramps the last CSV references to
zero, requires fresh measured wheel/steering stationarity, seeds actual wheel positions,
and waits for an exact all-four post-send sequence acknowledgement before requesting CSP.
Relative output opens only after newer all-four PDO observations confirm mode 8, configured
status predicates and adapter mode acknowledgement. Reverse handoff preserves the stopped
CSP target, sends zero CSV first, confirms mode 9 on all four drives, advances the command
generation and accepts only a command received after completion. A send acknowledgement
means the process image passed the EtherCAT send hook; only later mode/status readback can
represent device acceptance.

Wheel travel/external steering determine translation and IMU determines yaw. Invalid
required feedback freezes the pose snapshot; discrepancy and terminal residual quality
are independent of encoder success. No active yaw correction or corrective motion is
added. Drive/bus loss inhibits output and aborts without a controlled-stop claim.
All physical and timing values are explicit; the installed YAML is a synthetic fixture
selected only with `explicit_no_device_mock=true`. Immutable configuration cannot change
frame/limits during a goal. No controlword ownership moves from `enable_manager`.

## 验证

Native Humble, no hardware. The pinned public-interface archive
`robot_interfaces@9aa2693d7d3235958369272b7ce8c48592dd7e83` was reused unmodified
from an isolated temporary prefix. Build artifacts and logs stayed outside the repository.

- `colcon build --packages-up-to swerve_driver enable_manager damiao_head_controller rolling_trajectory_controller control_api_adapter plc_node plc_io_modbus --cmake-args -DBUILD_TESTING=ON` with explicit source/base/install paths: 14 packages PASS, including all current private-interface consumers.
- `colcon test --packages-select swerve_driver enable_manager damiao_head_controller rolling_trajectory_controller control_api_adapter plc_node plc_io_modbus rt_control_interfaces`: PASS. `plc_io_modbus` and the definition-only interfaces package register no tests; their build is not runtime coverage.
- Final `swerve_driver` suite after the corridor-token/precedence follow-up: 122 cases in ten CTest targets, including 12 planner, 17 session, 20 ROS action/service/serialization mock cases, 2 executable process cases and 71 existing cases. Coverage includes busy/rejection, every cancellation phase, stationary/mixed readback, stale/lost/jumping IMU, report-only mismatch, slip/steering/drive/bus faults, timeout, stop failure, reset, stale-command discard and active-goal restart.
- `g++ -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -fno-omit-frame-pointer -fsanitize=address,undefined ... -lgtest_main -lgtest -pthread`: planner/session 29 cases PASS; no sanitizer diagnostics.
- `tools/quality_gate.sh`: 207 passed, 12 skipped, 83% gated policy coverage; architecture/file hygiene and EtherCAT shutdown policy PASS. ShellCheck unavailable locally; CI remains responsible for it.
- `git diff --check`, `git diff --cached --check`, index review and scoped baseline-preservation comparison: PASS; no staged files. The existing retirement changes remain intact.

Recovery integration verification used a fresh `/tmp/e145-recovery-ws` because the
interrupted worker's temporary overlays no longer existed. Exact local pinned sources were
used for `robot_interfaces@9aa2693d...`, `ecat_icube@1390be74...` plus repository patches
0001..0013, and `ros2_controllers@cbcf6621...`; no vendor/pin edit was made.

- Dependency/interface build with `BUILD_TESTING=OFF`: 9 packages PASS.
- `robot_hw_ethercat` and `swerve_driver` build with `BUILD_TESTING=ON`: PASS.
- Isolated localhost ROS domains: connected five-case controller/four-PDO suite repeated
  three times (15/15), controller-manager lifecycle repeated ten times (10/10), and Kinco
  adapter nine-case suite repeated five times (45/45), all PASS.
- `colcon test --packages-select robot_hw_ethercat swerve_driver` and
  `colcon test-result --verbose`: 270 tests, zero errors/failures/skips.
- Parent reproduction after async-run state loss imported every dependency from the exact
  `deps.repos` SHA, applied repository patches, built 12 dependencies with tests disabled,
  then rebuilt `rt_control_interfaces`, `robot_hw_ethercat` and `swerve_driver` with tests
  enabled using an EtherLab userspace copy extracted from the already validated no-device
  image. The three changed packages built and 270 records passed before the final reset
  correction; final two-package test results are 271 records, zero failures. Quality gate,
  diff checks and 21-case handoff ASan/UBSan test passed. Logs:
  `/tmp/e145-deps-build.log`, `/tmp/e145-current-build.log`,
  `/tmp/e145-current-test.log`, `/tmp/e145-current-quality.log`.
- The connected five-case cyclic suite repeated 3/3 and controller-manager lifecycle
  repeated 10/10 under the fully sourced overlay. A direct CTest attempt without sourcing
  the rebuilt `rt_control_interfaces` overlay failed only because its generated type-support
  library was not on `LD_LIBRARY_PATH`; the same binaries passed after using the documented
  overlay, so that attempt is environment evidence rather than a product failure.
- Parent source review found reset was evaluated before the current update consumed its
  new feedback. Reset is now deferred until the cycle's handoff/session checks have run;
  a new bus/drive fault cannot be cleared using the previous healthy sample. The connected
  `ResetUsesCurrentCycleFaultInsteadOfPreviousHealthyFeedback` regression queues a reset,
  injects lost PDO completeness before the next update and checks non-OK response and
  revoked write sequence/mask. Scoped build and final tests PASS:
  `/tmp/e145-reset-build.log`, `/tmp/e145-final-test.log` (271 records).
- Original async integration/recovery runs lost their runtime state. User explicitly
  authorized direct parent takeover; this final follow-up was self-reviewed by the parent,
  not independently accepted by the earlier offline-only reviewer. No independent review
  result for the new cyclic integration is claimed.
- Connected cases cover the full NAVIGATION stop -> sent seed -> all-four CSP confirmation
  -> relative success/hold -> sent-zero CSV return -> fresh-command path, partial readback,
  missing send hook, IMU loss latch/token echo, cancellation and lifecycle abort.

The first ROS compile exposed Humble's missing `ServerGoalHandle::SharedPtr` alias;
using `std::shared_ptr<Handle>` fixed it. An early process SIGINT initially exited 1
because ROS context shutdown interrupted service creation; the executable now exits 0
for requested shutdown while invalid configuration still exits 1. Persistent process
tests cover both startup interruption and later shutdown/restart. A numeric-overflow
fixture was corrected to actually overflow the wheel-unit conversion before final tests.

Independent review `99622560-6658-4bac-882e-7ffe32fb15ce` found P1: mode/reset services
used cached freshness and stationary dwell, and an overdue timer applied simulated
mode/output before detecting the cycle fault. The correction centralizes receipt-time
encoder/IMU age calculation for goal and both service admissions. Mode rejects stale
encoder/cycle evidence; reset additionally rejects stale IMU evidence without clearing
the latch or uninhibiting output. Receipt checks never advance dwell. The timer skips
plant simulation when its cycle expired and sends aged observations to the fault path.
Four regression cases use actual 200 ms monotonic delays with timer dispatch paused,
while real ROS service callbacks still run; they independently expire encoder, IMU and
cycle thresholds, and check that overdue dispatch changes neither mode nor positions.
The pre-fix run reproduced all four defects; an initial test's repeated single-use
future read was also corrected. Focused rerun: 4/4 PASS; full package: 117/117 PASS;
aggregate `colcon test-result`: 399 records, zero errors/failures/skips. Logs:
`/tmp/e133-p1-red.log`, `/tmp/e133-p1-focused.log`, `/tmp/e133-p1-test.log`,
`/tmp/e133-p1-results.log`. Planner/session code is unchanged in this correction;
the sanitizer evidence above belongs to the preceding implementation run.
The subsequent recheck confirmed this original freshness finding resolved.

Review `585d40aa-c659-4232-94bc-514888ac300e` confirmed the freshness P1 fixed and
identified missing corridor correlation, invalid-request precedence and implicit
mode collapse. The follow-up adds opaque `string corridor_token` to private action
Goal/Feedback/Result, echoes the accepted handle's value unchanged for every feedback
and terminal path (also admission-to-start failure), and keeps strings outside the
fixed-size core. No safety lease or validation meaning is assigned. Both services
retain explicit BUSY precedence, then validate confirmation/legal mode before stale
feedback. Invalid requests therefore remain INVALID_REQUEST even with stale samples.

The unmodified pinned public dependency plus changed rosidl and all existing private
consumers rebuilt successfully (14 packages). Tests sourced the rebuilt overlay:
eight selected packages PASS; aggregate 404 records with zero errors/failures/skips.
The package's five added cases cover three-payload C++ serialization, rejected/next-goal
token isolation, accepted-before-start failure echo, stale-invalid services and stale
busy precedence. Existing success/cancel/fault/timeout tests now verify token echo;
cancel phases use distinct tokens and fault/timeout recovery issues different/empty
tokens. Every received feedback is compared to its own submitted goal token.
Focused token/precedence/terminal tests: 8/8 PASS. Full package: 122 cases PASS.
After a provider stream interruption, the preserved source was inspected and the
14-package build, eight focused cases and eight-package test selection were rerun
successfully; this is recovery-run evidence rather than a completion claim for the
interrupted process. Final evidence: `/tmp/e133-token-recovery-build.log`,
`/tmp/e133-token-recovery-focused.log`, `/tmp/e133-token-recovery-test.log`,
`/tmp/e133-token-recovery-results.log`. The rebuilt generated Python types also pass
nine Goal/Feedback/Result serialization round trips (empty, Unicode/whitespace and
4096-byte tokens); the imported module is verified under the rebuilt install prefix.
No planner/session source changed, so previous sanitizer evidence is retained rather
than represented as rerun. Final targeted independent recheck
`045bafbb-2a98-40e6-b853-45316917494a` found no remaining issues and returned OK;
its reviewer inspected source and logs without shell execution. The parent independently
reran the final swerve package tests and repository quality gate, both PASS;
aggregate results remain 404 records with zero errors/failures/skips (including retained
unaffected consumer results). Parent logs: `/tmp/e133-parent-final-test.log` and
`/tmp/e133-parent-final-quality.log`. Baseline-preservation audit and diff checks PASS.

本记录不授权使能或运动。

## 结论与冻结事实

- F1: Tier A includes the offline core/mock and an opt-in loaned-interface controller plus
  synthetic-PDO adapter integration; physical commissioning and production runtime admission remain closed.
- F2: OPERATION semantics retain steering CSP and require drives CSP; NAVIGATION semantics require drives CSV. This extends the BQ-144 design for Tier A only, without changing its steering-observation or controlword ownership decisions.
- F3: Cancel/fault stopping preserves reference continuity and selected steering; success requires measured encoder position/velocity settling. IMU-primary pose quality is reported independently and never drives corrective motion.
- F4: All production calibration, thresholds, timestamps and drive authority must come from verified hardware owners. Synthetic fixture values are not engineering facts.

## 遗留

PR #48 independent review found a reverse-handoff residual bug: after requesting CSV,
the adapter replaced the held CSP command with actual feedback, so an allowed nonzero
settling residual prevented exact sent-sequence acknowledgement. The correction latches
the commanded raw hold through the reverse request/in-flight phase and preserves the
masked position during CSV confirmation. Connected regression
`ReverseHandoffPreservesHoldDespiteEncoderSettlingResidual` injects a three-count residual
within explicit synthetic tolerance and checks unchanged hold plus successful CSV return.
No-device container `e133-pr48-regression` rebuilt the three changed packages and tested
the two runtime packages: 272 records, zero failures; repository quality gate PASS.
Targeted independent recheck is pending for this correction.

BQ-145/BQ-150: real identity/PDO assignment/ring positions, per-module
conversion/calibration/limits, fresh steering velocities, physical mode/status acceptance,
stop authority/behavior and shared-bus IMU readiness remain unproven. Runtime CSV/CSP
support itself is no longer described as unsupported: it is user-confirmed and matches the
cataloged Kinco 0x6060/0x6061 CSP/CSV contract. No production launch admission, real
250 Hz timing, switching, braking distance, obstacle clearance, Docker packaging or hardware
validation is claimed. Local source consumes `/cmd_vel`; frozen public `robot_interfaces`
still names the previous endpoint and must be updated and atomically adopted before
cross-domain deployment. Final cyclic integration was parent-reviewed after explicit
user authorization to take over the failed async workflow. Separate independent review
of this new integration remains outstanding; the earlier offline-only review does not
cover it. Commit/push/PR/deployment and hardware operations were not performed.
