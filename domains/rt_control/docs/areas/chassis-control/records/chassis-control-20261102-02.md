---
id: chassis-control-20261102-02
area: chassis-control
title: Restore the previous steering software baseline
date: 2026-11-02
type: corrective
trigger: User explicitly requested restoring the previous remote code and removing the commissioning additions.
commits: []
env: native
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence:
  - validation/steering-baseline-restore-20261102/device-config-before-restore.json
supersedes: []
related: []
---

## Background

The user requested rollback of the current session's commissioning additions. Existing unrelated worktree changes must remain intact.

## Changes

Restore the pre-session snapshots of the steering controller, Kinco PDO adapter, renderer, profile, validation and affected tests. Remove the added commissioning tools, interfaces, tests and documentation. Restore the previous restart script. Rebuild the three affected packages and regenerate the canonical deployment artifact using the composer and CANopen generator; no generated hash or YAML was hand-edited. Restore the four steering TxPDO mappings to their original four entries and DIN1 function to zero, with PREOP, stopped-runtime, identity and disabled-state checks. No controlword, calibration, motor parameter or flash writes were performed.

## Validation

- Native build: robot_hw_ethercat, swerve_driver and rt_control_composer PASS.
- Tests: robot_hw_ethercat 151, swerve_driver 198, rt_control_composer 225; zero errors, failures or skips. Obsolete deleted-test results were excluded.
- quality_gate.sh, git diff --check, staged whitespace check and restart-script bash syntax: PASS.
- Source and deployed configuration searches: removed interfaces and tools absent.
- All four steering mappings read back with four entries; DIN1 function read back as zero.
- Runtime stopped; no enable or motion executed during rollback.
- Evidence paths are relative to the deployment workspace, outside Git.

本记录不授权使能或运动。

## Conclusions

- F1: The deployed software and cyclic input configuration match the pre-session baseline; unrelated worktree edits are preserved.
- F2: Rollback does not resolve or clear a drive's existing motor-configuration alarm.

## Remaining

Steering drive 4 retained 0x603F=0x6321, 0x2601=0x0001 and 0x2602=0x0010 before restoration. Its motor configuration requires separate diagnosis. Motion was not retested. The host restarted externally during final checks; final runtime and EtherCAT service are stopped. Live device readback after that restart was unavailable; mapping readback above occurred before the restart. No commit or push.
