---
id: contract-20260925-01
area: contract
title: ELECTRI-133 additive RT-private chassis execution protocol
date: 2026-09-25
type: feature
trigger: ELECTRI-133; ELECTRI-144 component integration
commits: [feature/electri-133-tier-a]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-145, BQ-150, motion-20260925-01]
---

## 背景

Tier A needs an internal relative-motion contract without adding any public endpoint
or cross-domain consumer. Public types and the pre-existing non-chassis private
wire types are unchanged.

## 改动

Integrated the six ELECTRI-144 definitions into `rt_control_interfaces`:
`ChassisRelativeMove`, `ChassisMoveLimits`, `ChassisMoveState`, `ChassisState`,
`ChassisSetMode`, `ChassisResetFault`. The package README defines admission, units,
execution-start frame, single-goal reservation, terminal status, timeout, cancel,
quality bits, explicit reset and restart/no-replay semantics. The private-type inventory
test now includes these six types while preserving the retirement baseline.

The reviewed follow-up adds `string corridor_token` to ChassisRelativeMove Goal,
Feedback and Result. The accepted goal's token is echoed unchanged in every feedback
and success/cancel/fault/timeout result, including failure before execution starts.
It is opaque correlation only, with no safety lease/validation meaning; empty strings
are preserved. Tokens stay on the ROS side and cannot leak from rejected or previous
goals. This changes the chassis action wire layout, so its producer and clients must
rebuild/source matching generated types. Other private wire types remain unchanged.
NAVIGATION explicitly collapses TRANSIT/DOCKING solely in the offline mock, without
claiming their distinct production behavior or full production mode coverage.

The producer set now includes the explicit RT-local no-device
`swerve_driver/chassis_relative_move_mock` and the opt-in `SwerveController` loaned-interface
adapter. The latter binds the real command/state interface contract but remains disabled by
default and absent from production launch because the machine profile is not verified.
There are no `robot_interfaces`, pin or Robot Model changes. Existing private consumers
build against the additive types without migration. New private clients and producers
must use the same reviewed protocol revision.

The user-selected local velocity endpoint is `/cmd_vel`; the old local endpoint is not
subscribed as an alias. The pinned public `robot_interfaces` contract still names the
previous endpoint. Its owner will submit the public rename separately; RT and Motion must
adopt that SHA atomically before deployment, so this source result is not evidence of
current cross-domain compatibility.

## 验证

- Generated rosidl C++/Python interfaces compile against the unmodified public pin
  `9aa2693d7d3235958369272b7ce8c48592dd7e83` in an isolated native build.
- The 14-package closure includes `swerve_driver`, `enable_manager`,
  `damiao_head_controller`, `rolling_trajectory_controller`, `control_api_adapter`,
  `plc_node`, `plc_io_modbus` and interface dependencies. Build PASS.
- Twenty persistent ROS action/service/serialization cases exercise generated payloads,
  rejection, busy slot, goal UUID, cancel, fault/timeout abort, encoder success with quality
  bits, mode readback, reset/restart, elapsed freshness, invalid-request precedence, and
  corridor-token echo/isolation. The full swerve package has 122 passing cases; eight
  selected packages report 404 aggregate colcon records with no errors/failures/skips.
  See motion-20260925-01 and `/tmp/e133-token-recovery-test.log` for the checks rerun
  after preserving source through the provider stream interruption.
- `tools/quality_gate.sh`: 207 passed/12 skipped, architecture and interface ownership PASS.
  The definition-only interface package has no runtime tests; runtime coverage belongs
  to `swerve_driver`, not generated schema checks.

本记录不授权使能或运动。

## 结论与冻结事实

- F1: The six chassis types are additive RT-Control-private schemas; no other domain is a consumer.
- F2: Mode OK means completed backend confirmation; action success is encoder completion and is independent of pose quality flags.
- F3: Producers are the explicit no-device mock and an opt-in real-interface adapter whose production profile/gate remains closed; DDS and synthetic-PDO success do not validate physical switching.
- F4: Local source accepts velocity only on `/cmd_vel`; cross-domain use is blocked until the public contract rename is merged and atomically pinned by all consumers.

## 遗留

Independent review of the final cyclic handoff follow-up is pending. Real Kinco PDO
assignment/identity/calibration and IMU shared-bus readiness remain blocked by
BQ-145/BQ-150; runtime switching capability itself is user-confirmed and supported by the
cataloged 0x6060/0x6061 contract. Public `/cmd_vel` migration is also pending the separate
`robot_interfaces` change. No hardware access, commit, push, PR or production launch was
performed.
