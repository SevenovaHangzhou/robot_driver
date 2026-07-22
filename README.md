# robot

ROS 2 Humble monorepo for the robot real-time control domain.

The source layout follows the frozen rt_control implementation specification:

- `src/interfaces/robot_interfaces`: robot-internal ROS interfaces.
- `src/description/robot_description`: kinematics-only robot description.
- `src/rt_control`: EtherCAT/CANopen configuration, bringup, enable/fault handling, and diagnostics packages.
- `docker`: rt-control image and Compose deployment.
- `tools`: migration and commissioning tools.
- `hostsetup`: target-host setup assets.

The legacy baseline `/home/kkozia/robot_driver@6bc94cd` is read-only and is not vendored in this repository. External source checkouts are declared in `deps.repos`.

Build the in-repository packages with:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Build or run the rt-control image only after exporting the target host's
validated isolated CPU set:

```bash
export RT_CONTROL_CPUSET=<validated-isolated-cpu-list>
tools/rt_control_compose.sh build rt-control
tools/rt_control_compose.sh up -d rt-control
```

The wrapper reads the pinned IgH version and full commit from `versions.env`
and tags the image with the current repository commit. The dependency identity
is also stored in the image labels and
`/usr/local/share/rt-control/dependency-versions.env`. The wrapper intentionally
has no default CPU set.

The supported container command is the installed `rt_control_start` signal
gate. It waits up to 30 seconds for `/rt/disable` before forwarding an orderly
shutdown to ROS. The image executes the installed file directly so it remains
PID 1; do not wrap it in `ros2 run` when overriding the container command.
Starting `ros2 launch rt_control_bringup rt_control.launch.py` directly also
bypasses that clean-shutdown guarantee.

For hardware-free controller loading checks, pass
`use_mock_hardware:=true` to the launch file. The production default is
`false`. If GitHub access alone needs the host proxy during an image build, set
`RT_CONTROL_BUILD_PROXY` to a host-reachable proxy URL for that invocation;
the value is scoped to the source-import layer and is not stored as a runtime
environment variable.
