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

`led_strip_node` subscribes to `led0/color` through `led5/color` using
`std_msgs/msg/ColorRGBA`. RGB is normalized to `0.0..1.0`; alpha is the W
channel. Each message writes WE-10x holding registers 40001..40004 with FC16.

The supplied `led_strip.yaml` defines six controllers sharing TCP 502 and using
RTU addresses 1 through 6. Change it
only after checking the physical bus.

```bash
ros2 run modbus_tcp_rtu485 led_strip_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485)/share/modbus_tcp_rtu485/config/led_strip.yaml
```

Example command, which writes real hardware:

```bash
ros2 topic pub --once /led0/color std_msgs/msg/ColorRGBA \
  '{r: 1.0, g: 0.0, b: 0.0, a: 0.0}'
```

Failed writes are logged and are not retried. Exit does not send an implicit
off/reset command, so the hardware can retain its last color.

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
| `ultrasonic/channel1/range` .. `channel8/range` | `sensor_msgs/msg/Range` | Per-channel range, timestamp, frame, radiation type, field of view and range limits |
| `ultrasonic/raw` | `std_msgs/msg/UInt16MultiArray` | Eight unmodified E08 registers, ordered by channel |
| `ultrasonic/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Per-channel protocol state or gateway error |

Protocol values are mapped as follows:

| Raw | Range message | Diagnostic |
| --- | --- | --- |
| normal millimetres | metres | `OK / ok` |
| `0xFFFD` | positive infinity | `OK / no_target` |
| `0xFFFE` | NaN | `WARN / interference` |
| `0xFFFF` | NaN | `ERROR / sensor_timeout` |
| `0xEEEE` | NaN | `ERROR / checksum_error` |

Values outside configured `min_range_m..max_range_m` publish NaN with
`WARN / out_of_range`. A TCP/Modbus failure publishes only an ERROR diagnostic;
the node does not republish stale measurements as current data.

```bash
ros2 run modbus_tcp_rtu485 ultrasonic_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485)/share/modbus_tcp_rtu485/config/ultrasonic.yaml
```

Inspect data with:

```bash
ros2 topic echo /ultrasonic/raw
ros2 topic echo /ultrasonic/channel1/range
ros2 topic echo /ultrasonic/diagnostics
```

The eight messages from one two-E08 poll cycle share the response-completion timestamp.
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

`rt_control.launch.py` provides `start_led`, `led_config`, `start_ultrasonic`
and `ultrasonic_config` in the retained legacy source entry, which V3 does not
install. Both hardware nodes default off under mock hardware and
on for a real-hardware launch. Override either start argument when a device is
absent or another client owns its gateway channel.

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
