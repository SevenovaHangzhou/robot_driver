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

The current 0.7.0 pin is the immutable head of upstream PR #6 for migration
validation. It is not a release identity: after that PR merges, replace both
pins with its final main commit before merging or releasing this branch.
