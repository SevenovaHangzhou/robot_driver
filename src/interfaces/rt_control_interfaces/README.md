# RT-Control private interfaces

This package is owned and consumed only by RT-Control: `PlcIoState` and `StopJointTrajectory`. Other domains use the public
`robot_interfaces` packages instead.

`StopJointTrajectory` is an optional operator-to-controller stop transaction. It carries the
current FJT UUID and gear limits; the controller chooses the actual takeover time. Its deferred
response reports reference completion or failure, not physical standstill. It is not a cross-domain
command or a replacement for FJT's public cancel contract.
