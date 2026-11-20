# ctrl_wheel_observation_broadcaster

Read-only ros2_control broadcaster that publishes R-OUT-29 `/rt/base/wheel_observations`
(`robot_rt_control_interfaces/msg/WheelObservationArray`, `fast_state` QoS) for navigation's
per-wheel fusion. It claims state interfaces only, so it runs while the chassis is disabled
and whichever motion controller (`swerve_controller` or `base_wheel_jtc`) owns the wheels.
`/rt/base/odom` stays unchanged; it shares these sources and must not be fused alongside them.

## Data

The composer binds the same wiring and calibration exports as `swerve_controller`. Wheels are
placed FL/FR/RL/RR by the sign of their calibrated `module_x/module_y` in `base_footprint`,
not by wiring index; configuration is rejected unless each quadrant holds one module.

| Field | State interface | Channel time | `sample_sequence` |
| --- | --- | --- | --- |
| `drive_position`, `drive_velocity` | drive `position`, `velocity` (one PDO) | `READ_COMPLETE`: controller cycle time − drive `feedback_age_ms` | new drive `feedback_sequence` (complete PDO cycles) |
| `steering_motor_angle` | steering `position` × `steering_to_yaw_sign` | `READ_COMPLETE` as above | new steering `feedback_sequence` |
| `steering_encoder_angle` | encoder `position` | `HOST_RECEIVE`: cycle time − encoder `feedback_age_ms` (user-space CANopen callback) | a fall of the encoder receive age |

Units and conventions are the hardware adapters': wheel-axis rad and rad/s, steering heading
from +x, counter-clockwise positive, in calibrated mechanical-travel coordinates. Drive
velocity is the drive's own `0x606C` measurement; its internal filter setting is not read.

The encoder exposes no frame counter. The broadcaster runs every controller_manager cycle
(1 kHz) and counts a new frame when the receive age falls; frames closer than one cycle
(SYNC is 4 ms) count once. Its stamp overestimates the receive time by the read-to-update
delay of the same cycle.

## Validity and continuity

A channel is `NOT_READY` before its first sample, `STALE` when older than `feedback_timeout`
(the swerve controller's value), `NONFINITE`, or `DEVICE_FAULT` when the CiA402 status word
reports a fault. The value stays the last one read. `epoch` changes on the first valid sample
after any invalid period, on a feedback counter regression and on a drive position step above
1000 rad (an int32 count wrap or reset). `session_id` is `<startup_id>:<n>`; `n` changes on
every activation and when ROS time steps back, and all counters restart with it.

`calibration_id` joins the chassis and encoder renderer identities: a digest of every wiring,
conversion, zero and geometry fact they rendered. Contact-patch offsets, noise and timing
bounds are not published; they belong to the calibration record.

## Verification

```bash
colcon build --packages-up-to ctrl_wheel_observation_broadcaster --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ctrl_wheel_observation_broadcaster
```

Tests use in-memory state interfaces. With `use_mock_hardware` the feedback counters never
advance, so every channel stays `NOT_READY`; timing and rate need target-host measurement.
