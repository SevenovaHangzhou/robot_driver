# RT-Control interface boundary

`rt_control_interfaces` is private to this domain. It contains only RT-Control
internal messages and services such as `PlcIoState` and `RtEnable`; other
domains must not depend on it.

Cross-domain packages are not mirrored here. `deps.repos` vendors the
authoritative `robot_interfaces` repository at `src/vendor/robot_interfaces`,
including `robot_rt_control_interfaces`, `robot_system_interfaces`, and
`robot_interfaces_qos`. `source-lock.yaml` records the same immutable identity
for repository gates and release evidence. Change the upstream contract first,
then update the pin and all producers/consumers in one coordinated release.

The current 0.7.0 pin is the immutable upstream commit
`9aa2693d7d3235958369272b7ce8c48592dd7e83`. It preserves the previous V3
authority commit and adds the public rolling-control messages, services, and
QoS profiles. The V3 arm runtime freezes the 14-axis order as
`right_joint1..7,left_joint1..7`; consumers must verify the published
`axis_set_hash` before opening a rolling session. Every producer and consumer
must use this same SHA and V3 axis contract in one coordinated release.
