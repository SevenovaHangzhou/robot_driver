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
