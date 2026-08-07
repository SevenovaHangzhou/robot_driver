# RT-Control interface packages

Cross-domain wire contracts are owned by the `robot_interfaces` contract
repository. The checked-in `robot_rt_control_interfaces` and
`robot_system_interfaces` packages are the RT-Control build mirror of contract
version `0.6.0` at commit
`e19d1450339d6bce598f664eb18fb093e02097ff`; they have no independent schema
authority. `source-lock.yaml` is the machine-readable source identity. Change
the contract first, then synchronize this mirror and every producer/consumer in
one release.

`rt_control_interfaces` is different: it is private to this domain and contains
only RT-Control internal messages and services such as `PlcIoState` and
`RtEnable`. Other domains must not depend on it.
