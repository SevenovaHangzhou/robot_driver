Single-arm bench unit: one seven-axis ZeroErr arm on the main coupler X2, no gripper.
Host ar-Default-string (192.168.100.40): RT CPU 14, housekeeping CPUs 0,2,4,6,8,10,12,16-27
(deployment.yaml `host`). The scope runtime pins the update thread before hardware activation
and sets the IgH EtherCAT-OP thread to FIFO 79, so the stack must start with CAP_SYS_NICE.

On 2026-10-17 the user confirmed the bus order J1..J7 and set every drive zero
(2^18 counts) at the mechanical zero, which is also the Robot Model zero.
