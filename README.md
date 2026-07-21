# robot

ROS 2 Humble monorepo for the robot real-time control domain.

The source layout follows the frozen rt_control implementation specification:

- `src/interfaces/robot_interfaces`: robot-internal ROS interfaces.
- `src/description/robot_description`: kinematics-only robot description.
- `src/rt_control`: EtherCAT/CANopen configuration, bringup, enable, watchdog, and diagnostics packages.
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

The wrapper reads the IgH branch from `versions.env` and tags the image with
the current Git commit. It intentionally has no default CPU set.
