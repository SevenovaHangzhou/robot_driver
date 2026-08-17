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

The current 0.7.0 pin is the immutable upstream `main` commit
`f699f45972ad15bbbbbb3da1a4894faf209144c9`. It contains the merged BQ-137
wire-schema correction and the Perception contract-completeness change. Every
producer and consumer must use this same SHA in one coordinated release.
