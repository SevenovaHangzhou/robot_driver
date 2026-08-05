# RT-Control interface packages

Cross-domain wire contracts are owned by the `robot_interfaces` contract
repository. The checked-in `robot_control_interfaces` and
`robot_system_interfaces` packages are the RT-Control build mirror of contract
version `0.5.0` at commit
`698d520c6eebaa1437f2a03ca7ff95d95ad3a600`; they have no independent schema
authority. `source-lock.yaml` is the machine-readable source identity. Change
the contract first, then synchronize this mirror and every producer/consumer in
one release.

`rt_control_interfaces` is different: it is private to this domain and contains
only RT-Control internal messages and services such as `PlcIoState` and
`RtEnable`. Other domains must not depend on it.
