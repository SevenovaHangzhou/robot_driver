# Upstream sources

`ros2_control_node.cpp` is byte-for-byte `controller_manager/src/ros2_control_node.cpp` of
ros2_control tag `2.53.1` (https://github.com/ros-controls/ros2_control). Tags `2.54.2` and
[`2.54.3`](https://github.com/ros-controls/ros2_control/blob/65d389596953cb4656c4216a3f0bc08a73230739/controller_manager/src/ros2_control_node.cpp)
contain the same bytes (verified 2026-11-11, SHA-256
`ca6192e765b4ad433e9cd9053ca8209982f6367e301afd7f99620baa3c4eed92`).
CMake accepts exactly these three installed versions. Do not edit this source; local changes live in
`../patches/ros2_control_node-overruns.patch`, which the build applies.

Why a patched copy instead of the binary: Humble 2.53.x runs missed update cycles back to back
after a blocked read()/write(); each catch-up cycle sends an EtherCAT frame. The patch adds
Jazzy's `overruns.manage` rule (`include/integ_bringup/control_loop_timing.hpp`, unit-tested in
`test/test_control_loop_timing.cpp`), a steady-clock deadline and a 1 s overrun report. When the
controller manager moves to a release with that rule, delete this directory and the patch and use
the upstream executable.
