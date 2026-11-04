# Upstream sources

`ros2_control_node.cpp` is byte-for-byte `controller_manager/src/ros2_control_node.cpp` of
ros2_control tag `2.53.1` (https://github.com/ros-controls/ros2_control), the version of the
installed `ros-humble-controller-manager` package. CMake accepts only the reviewed
`controller_manager` versions 2.53.1, 2.54.2 and 2.54.3. Do not edit it; all local changes live in
`../patches/ros2_control_node-overruns.patch`, which the build applies.

Why a patched copy instead of the binary: Humble 2.53.x runs missed update cycles back to back
after a blocked read()/write(); each catch-up cycle sends an EtherCAT frame. The patch adds
Jazzy's `overruns.manage` rule (`include/integ_bringup/control_loop_timing.hpp`, unit-tested in
`test/test_control_loop_timing.cpp`), a steady-clock deadline and a 1 s overrun report. When the
controller manager moves to a release with that rule, delete this directory and the patch and use
the upstream executable.

## Compatibility with 2.54.2 and 2.54.3

The upstream [2.54.2 source](https://github.com/ros-controls/ros2_control/blob/2.54.2/controller_manager/src/ros2_control_node.cpp)
and [2.54.3 source](https://github.com/ros-controls/ros2_control/blob/2.54.3/controller_manager/src/ros2_control_node.cpp)
are byte-identical to the frozen 2.53.1 file. All have SHA-256
`ca6192e765b4ad433e9cd9053ca8209982f6367e301afd7f99620baa3c4eed92`.
The existing patch applies unchanged; the native build and controller/mock tests validate
linking against the installed 2.54.2 library. GitHub full CI validates 2.54.3, which the
ROS apt repository currently installs. Other versions still require source review.
