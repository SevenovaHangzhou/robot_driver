# modbus_tcp_rtu485

ROS 2 C++ drivers for RT-Control-local Modbus RTU devices connected through an
RS485-ETH-M04 Modbus TCP gateway. The package currently contains independent
LED and ultrasonic nodes:

```text
led_strip_node   -> TCP 502 -> WE-10x LED controllers
ultrasonic_node  -> TCP 504 -> two DYP-E084F-V2.0 adapters -> eight A22 sensors
```

Socket IO runs in normal ROS callbacks, outside the ros2_control real-time loop.
The gateway permits one TCP client per channel, so another tool must not hold
the same port while a node is running.

## LED node

`led_strip_node` provides `/led/set_rgbw` using
`robot_rt_control_interfaces/srv/SetLedRgbw`. The request selects strip 0..5
and supplies complete normalized red, green, blue and white values. Four zeros
mean off. The service rejects non-finite or out-of-range values instead of
clamping them. A successful result proves only that the Modbus write was
acknowledged; the hardware has no light-output feedback.

The supplied `led_strip.yaml` defines six controllers sharing TCP 502 and using
RTU addresses 1 through 6. Change it only after checking the physical bus.
```bash
mbpoll -v -m tcp -a 1 -0 -r 4 -c 1 -t 4:hex -1 -o 2 \
  -p 502 192.168.1.12
```
```bash
ros2 run modbus_tcp_rtu485 led_strip_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485)/share/modbus_tcp_rtu485/config/led_strip.yaml
```

Example command, which writes real hardware:

```bash
ros2 service call /led/set_rgbw robot_rt_control_interfaces/srv/SetLedRgbw \
  '{strip_id: 0, red: 1.0, green: 0.0, blue: 0.0, white: 0.0}'
```

Failed service writes are logged and are not retried. With the default
`exit_color_enabled:=true`, a graceful SIGINT/SIGTERM writes RGBW zero to all
six controllers before exit. An uncaught exception escaping the node executor
makes a best-effort write of red (`255,0,0,0`) to all six controllers before
returning failure. Each controller is attempted independently. Exit writes use the
separate `exit_response_timeout_ms` deadline (800 ms by default), bounding six
sequential attempts when one or more RTU stations are offline.

The abnormal-exit color cannot cover SIGKILL, process memory corruption, host
power loss or loss of the gateway/RS485 path because the process cannot send a
Modbus request in those cases. A separately supervised hardware or process
watchdog is required if red indication must be guaranteed for those failures.
Set `exit_color_enabled:=false` only for hardware-free tests or when an external
owner provides the exit indication.

### Six-controller field verification

On 2026-10-05, all six WE-10x controllers on `192.168.1.12:502` were verified
at RTU addresses 1 through 6. `strip_id` values 0 through 5 map to those
addresses in the same order. Service writes were acknowledged and the
corresponding outputs were observed on all six controllers.

A graceful Ctrl+C (SIGINT) exit switched off all six strips, and a controlled
caught-exception exit switched all six strips to red. The successful exit
tests used a 800 ms per-controller deadline:

```bash
ros2 run modbus_tcp_rtu485 led_strip_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485)/share/modbus_tcp_rtu485/config/led_strip.yaml \
  -p exit_response_timeout_ms:=800
```

This verifies the six-controller service-write, normal-exit and caught-exception
paths on the installed bus. It does not cover SIGKILL, host power loss, process
memory corruption, gateway failure or RS485 failure.

## Ultrasonic node

`ultrasonic_node` polls two E08 adapters sequentially through the same TCP 504
gateway channel. It reads `0x0106..0x0109` from RTU unit 1 for channels 1..4,
then `0x0106..0x0109` from RTU unit 6 for channels 5..8. E08 addresses 2..5 are
reserved for sensor interfaces and are rejected by configuration validation.
The supplied configuration uses `192.168.1.12:504`, a 300 ms poll interval and
a 500 ms deadline per transaction. A22 metadata is fixed and validated as a
3.5 m maximum range and a 40-degree (`0.6981317008 rad`) field of view. The
A22 angle-level register must be commissioned separately to level 2; this
read-only driver does not write device configuration.

Published topics:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/ultrasonic/channel1/range` .. `channel8/range` | `sensor_msgs/msg/Range` | Per-channel range; each E08 uses its own read-completion time |
| `/ultrasonic/unit1/raw`, `/ultrasonic/unit6/raw` | `std_msgs/msg/UInt16MultiArray` | Four unmodified registers from one successful E08 read |
| `/rt_control/sensors/status` | `robot_rt_control_interfaces/msg/SensorStatusArray` | Per-channel validity, protocol state, communication failure and sample time |
| `/ultrasonic/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Engineering diagnostics |

Protocol values are mapped as follows:

| Raw | Range message | Diagnostic |
| --- | --- | --- |
| normal millimetres | metres | `OK / ok` |
| `0xFFFD` | positive infinity | `OK / no_target` |
| `0xFFFE` | NaN | `WARN / interference` |
| `0xFFFF` | NaN | `ERROR / sensor_timeout` |
| `0xEEEE` | NaN | `ERROR / checksum_error` |

Values outside configured `min_range_m..max_range_m` publish NaN with
`WARN / out_of_range`. Each E08 is attempted independently. A TCP/Modbus failure
stops only that unit's four Range topics, publishes immediate structured failure
state and does not republish stale measurements. Consumers treat a Range older
than 2.5 seconds as stale.

```bash
ros2 run modbus_tcp_rtu485 ultrasonic_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485)/share/modbus_tcp_rtu485/config/ultrasonic.yaml
```

Inspect data with:

```bash
ros2 topic echo /ultrasonic/unit1/raw
ros2 topic echo /ultrasonic/channel1/range
ros2 topic echo /rt_control/sensors/status
```

The four messages from one E08 read share that request's response-completion
timestamp. The two E08 reads are not presented as one synchronized sample.
Default frames are channel identifiers only. Replace them with the installed
sensor frame names and publish measured transforms to `base_link` before another
domain uses the readings geometrically. Protocol and gateway failures are
reported on `ultrasonic/diagnostics`.

Each four-register FC03 request triggers the four sensors on that E08 together.
This driver serializes the two E08 requests but does not implement acoustic
crosstalk mitigation within either four-sensor group.

Direct communication check without ROS:

```bash
mbpoll -v -m tcp -a 1 -0 -r 262 -c 4 -t 4:hex -1 -o 2 \
  -p 504 192.168.1.12
mbpoll -v -m tcp -a 6 -0 -r 262 -c 4 -t 4:hex -1 -o 2 \
  -p 504 192.168.1.12
```

## Bringup and offline verification

V3 installs an independent acquisition entry point:

```bash
ros2 launch rt_control_bringup rt_control_ultrasonic.launch.py
```

It starts only `ultrasonic_node`. Setting `use_mock_hardware:=true` creates no
hardware acquisition process. `ultrasonic_config` accepts a commissioned YAML
file. The V3 arm runtime does not implicitly start ultrasonic acquisition.

The legacy `rt_control.launch.py` entry and its track-drive options were retired from
this source tree. V3 installs only the independent acquisition entry above; it
still defaults to no hardware acquisition in mock mode.

The transport validates the MBAP transaction, protocol, length, unit, function,
exception response, FC16 acknowledgement and FC03 byte count. A single bounded
deadline covers connect, partial send and fragmented receive.

Build and test with external artifact directories:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/modbus-rtu485-log build \
  --base-paths src/rt_control/modbus_tcp_rtu485 src/rt_control/rt_control_bringup \
  --build-base /tmp/modbus-rtu485-build \
  --install-base /tmp/modbus-rtu485-install \
  --packages-select modbus_tcp_rtu485 rt_control_bringup
colcon --log-base /tmp/modbus-rtu485-log test \
  --build-base /tmp/modbus-rtu485-build \
  --install-base /tmp/modbus-rtu485-install \
  --packages-select modbus_tcp_rtu485 rt_control_bringup
colcon test-result --test-result-base /tmp/modbus-rtu485-build --verbose
```

The transport test uses local Unix socketpairs. The lifecycle test overrides
`poll_enabled:=false`, so it never opens a connection to real hardware.
