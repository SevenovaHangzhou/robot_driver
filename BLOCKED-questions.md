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
