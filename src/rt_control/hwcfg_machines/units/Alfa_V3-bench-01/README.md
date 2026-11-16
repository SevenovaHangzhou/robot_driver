Single-arm bench unit: one seven-axis ZeroErr arm on the main coupler X2, no gripper.
Host ar-Default-string (192.168.100.40): RT CPU 14, housekeeping CPUs 0,2,4,6,8,10,12,16-27
(deployment.yaml `host`). The scope runtime pins the update thread before hardware activation
and sets the IgH EtherCAT-OP thread to FIFO 79, so the stack must start with CAP_SYS_NICE.

On 2026-10-17 the user confirmed the bus order J1..J7 and set every drive zero
(2^18 counts) at the mechanical zero, which is also the Robot Model zero.

## Independent head and arms

`head_instance.yaml` records the head namespace and exact ROS routes used on this
bench. Generate separate `head_only` and `bench_dual_arm` artifacts; pass this file
only to the head composition:

```bash
ros2 run integ_composer compose_artifact --output /abs/head-artifact \
  --robot-variant Alfa_V3 --machine-unit Alfa_V3-bench-01 \
  --physical-profile head_only --control-scope head_only --mock \
  --instance-file "$(ros2 pkg prefix --share hwcfg_machines)/units/Alfa_V3-bench-01/head_instance.yaml"
ros2 run integ_composer compose_artifact --output /abs/arms-artifact \
  --robot-variant Alfa_V3 --machine-unit Alfa_V3-bench-01 \
  --physical-profile bench_dual_arm --control-scope bench_dual_arm --mock
```

Start each with `rt_control_start --scope <scope>` and matching
`generated_dir`, `robot_variant`, `machine_unit`, and `use_mock_hardware:=true`.
The web option `--head-instance-file /abs/head-artifact/instance.yaml` selects the
independent head. Startup leaves motion controllers inactive. Stopping head uses
its artifact routes; it does not stop the arms. Real startup requires separate
hardware authorization; these mock commands do not qualify real enable or motion.
