# Implementation progress

| Task | Commit | Self-check result | Remaining issues |
| --- | --- | --- | --- |
| T-002 | `7d9a6b259de79902b65d2445eb793a29a3fb3dac` | PASS — `colcon build --symlink-install` (8 packages); workflow/deps YAML parse; shell syntax; `diff_legacy.py --help` | CI has not run remotely; migration gate remains an explicit T-005 placeholder. |
| T-001 | `a18ae1fc5583d2e9b93f6ebfe805a8121aabe004` | PASS — verified all five HEAD/status values, authoritative worktree clean at `6bc94cd`, config `diff -rq`, full-depth CANopen search, SHA-256 inventory, and report required sections | BQ-001 blocks T-005 gate policy; BQ-002 blocks T-010 preload safety closure. |
| T-006 | `b551dcf3bb6770390fcbbf1ef9e423336caed6df` | PASS — 8-package `colcon build`; bus YAML schema/exact-scale/no-write-key assertions; `bash -n`; archive API grep is CORead-only; EDS byte SHA-256 matches source | BQ-003/004/005/006 block CANopen activation and some T-007/T-011/T-012 wiring; live SDO archive and generated DCF inspection remain T-014 commissioning work. |
