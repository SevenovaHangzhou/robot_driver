# bms_node

ROS 2 C++ driver for the Golden Phoenix BMS CAN protocol V1.1. The node publishes only
the RT-Control contract fields `voltage`, `percentage`, and `present` on
`/battery_state`; unsupported `sensor_msgs/msg/BatteryState` numeric fields remain
`NaN`.

## CAN contract

- Physical bus: CAN, 250 kbit/s, configured by the host before the node starts.
- Interface: `can1`, selected in `config/bms_node.yaml`.
- Frame type: 29-bit extended data frame, 8-byte payload.
- Query: `0x18900140`, eight zero bytes, every 200 ms.
- Response: `0x18904001`.
- Response byte 0-1: accumulated voltage, scale 0.1 V.
- Response byte 6-7: SOC, scale 0.1 percent.

The PDF does not define multi-byte byte order. `multi_byte_order: auto` accepts a frame
only when the SOC range identifies one unique order, then locks that order for the
process lifetime. Set `big_endian` or `little_endian` explicitly after a captured frame
or vendor confirmation. Ambiguous auto-detection never updates the published sample.

## Configuration

The installed configuration is `share/bms_node/config/bms_node.yaml`. The only supported
protocol and bitrate are `golden_phoenix_v1_1` and `250000`; incompatible values make the
node fail closed. The node does not rename, bring up, or change the bitrate of `can1`.

`frame_timeout_s` controls sample freshness. When no valid response arrives within that
window, voltage and percentage are published as `NaN` and `present=false`.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bms_node
colcon test --packages-select bms_node
colcon test-result --verbose
```

The tests cover both byte orders, conservative auto-detection, exact CAN IDs, extended
data-frame validation, stale samples, and reader-thread shutdown without a CAN device.
