Single-arm bench unit: one seven-axis ZeroErr arm on the main coupler X2, no gripper.
Host ar-Default-string (192.168.100.40): RT CPU 14, housekeeping CPUs 0,2,4,6,8,10,12,16-27
(deployment.yaml `host`). The scope runtime pins the update thread before hardware activation
and sets the IgH EtherCAT-OP thread to FIFO 79, so the stack must start with CAP_SYS_NICE.

On 2026-10-17 the user confirmed the bus order J1..J7 and set every drive zero
(2^18 counts) at the mechanical zero, which is also the Robot Model zero.

The right-arm-only control selection is `physical_profile=bench_right_arm` with
`control_scope=bench_right_arm` (motion) or `bench_right_arm_stationary_enable`.
The live scan on 2026-10-26 showed both arms and both wrist force sensors still
connected: 18 responders. The main coupler is position 0, left J1..J7 are 1..7,
the left force sensor is 8, right J1..J7 on X3 (main ESC port 1) are 9..15,
the right force sensor is 16, and the internal sub-coupler is 17. Only the right
seven drives have command interfaces; the remaining responders are identity-only.
`bench_right_arm_zero.yaml` retains the right-arm zero counts confirmed on
2026-10-22 and their original dual-arm ring indices. The earlier nine-responder
configuration matched an intended standalone wiring, not the live bench, and
incorrectly labeled left-arm feedback as right-arm feedback.

The motion scope manages only `right_joint1..7` and publishes
`/right_arm_jtc/follow_joint_trajectory` through `arm_joint_trajectory`.
On the bench host, `arm start right` starts this scope and the existing console
at `http://192.168.100.40:18121/`, using the independent
`console_poses_right.json` pose store. Stop the previous stack before changing
wiring or selecting another scope. Startup does not enable the arm. Right J3/J4
retain their confirmed single-turn policy; this selection does not add automatic
fault resets. Before hardware nodes are created, this scope requires an inactive
master and checks responder count, identities, PREOP states and X3 branch wiring
using cached master/slave information. This motion preflight performs no SDO
access. A different wiring is rejected and requires an explicit configuration.
