# bms_node

Golden Phoenix BMS CAN protocol V1.1 dual-battery monitor and opt-in discharge
coordinator. The node uses SocketCAN for a CAN card exposing a Linux `canX`
interface. It never configures the interface. Automatic control is disabled by default.

## CAN contract

- Physical bus: CAN, 250 kbit/s, configured by the host before the node starts.
- Transport: SocketCAN, with the target CAN card's `canX` selected by configuration.
- Frame type: 29-bit extended data frame, 8-byte payload.
- Poll plan: `0x90/0x91/0x92/0x93/0x94/0x95/0x98` for addresses `0x01` and `0x02`.
- Each address and data ID is queried once per 200 ms, staggered across the cycle.
- Response byte 0-1: accumulated voltage, scale 0.1 V.
- Response byte 6-7: SOC, scale 0.1 percent.

The PDF does not define multi-byte byte order. `multi_byte_order: auto` accepts a frame
only when the SOC range identifies one unique order, then locks that order for the
process lifetime. Set `big_endian` or `little_endian` explicitly after a captured frame
or vendor confirmation. Ambiguous auto-detection never updates the published sample.

## Configuration

The installed configuration is `share/bms_node/config/bms_node.yaml`. The host must
first verify the CAN card is wired to the battery bus, identify its actual `canX`, and
configure that interface at 250 kbit/s. Set `can_interface` in both the package and
bringup configurations to that verified name. `expected_adapter_serial` is optional:
when nonempty it is checked before opening the CAN socket; leave it empty for a
non-USB CAN card. The node reconnects when the interface is absent or goes down.
`automatic_discharge_control=false` keeps the node read-only. Enabling control requires
valid join/trip voltage limits, current limit, cell voltage and temperature limits,
stable/observation and command/status timeouts, and the measured `0x93` MOS on/off
values in YAML. Invalid or missing values reject node startup. The shipped YAML uses
invalid placeholders and does not enable K2 or D9 writes.

The configured domain-private Bool topics are `/bms/k2/command` (true closes,
false opens), `/bms/k2/closed` (physical auxiliary-contact feedback),
`/bms/loads/stop_request` (true requests load stop), and `/bms/loads/stopped`
(actual load-stop feedback). Topic publication alone does not drive a physical relay.
The target CAN card is reported as `can0`; this is not the earlier USB-CAN adapter.
The user reports that both battery addresses answer `0x90` queries and that manual
parallel discharge works. This does not validate automatic K2 switching or fault
isolation. The K2 hardware adapter is not in this package or the current electrical drawing;
it must use a DC-rated contactor, provide actual contact feedback, default open,
and de-energize on command heartbeat loss or process death. The controller will not
join without fresh contact/load feedback and command subscribers. It sends `D9`
only in enabled mode, checks the D9 reply and subsequent `0x93`, then closes K2
after the stable voltage window. A fault requests load stop before low-current MOS
switching; a secondary fault also opens K2. This software path has no real-hardware
commissioning evidence and does not replace the independent protection chain.
If the other pack faults after the first isolation is latched, the controller
re-enters load-stop handling and requests isolation of that second pack as well.
There is currently no producer for the K2 auxiliary-contact or load-stop feedback
topics in this repository; a ROS subscriber by itself is not proof of a wired output.
S1 on `0x01` is a manual switch. On a primary fault the controller requests `D9=0`
for `0x01`; it cannot operate S1, and the IPC may immediately lose power if the
secondary pack is not already connected. On a secondary fault it requests `D9=0`
for `0x02` and opens K2 after load-stop/low-current checks. A command or topic
publication alone does not prove that either pack was isolated.

The four control topics must be pairwise distinct after ROS name expansion and
remapping. Subscriber counts remain a connectivity check, not a new execution-end
readiness handshake. CAN reconnect waits do not suspend control ticks or relay-hold
publication. D9 write failures do not directly reset the connection. Every D9 OFF
retry rechecks fresh load-stop and low-current feedback.

Before D9 and while awaiting its ACK/MOS response, fresh K2-open feedback and an
ON primary MOS remain required. An unexpected primary MOS OFF during joining or
running enters `awaiting_primary_mos_isolation_policy`: request load stop and latch;
do not choose a K2 isolation or supply-transfer policy. Phase changes emit WARN/ERROR
logs; unchanged phases do not log every control tick.

Automatic control remains prohibited until the system owners settle controlled
shutdown, K2 level/edge/debounce/heartbeat semantics, isolation after unexpected
primary MOS loss, and load-stop persistence after fault latching. K2 OFF retries
are not changed by this review fix. See ELECTRI-131 and BQ-152.

The primary `0x01` pack publishes `/battery_state`; secondary `0x02` publishes
`/battery_state/secondary`. They are not averaged or summed because diode ORing
does not guarantee current sharing. A missing, stale, or faulted pack publishes
`present=false` with invalid numeric values; `0x98` faults set unspecified failure
health. The published status is not a replacement for the hardware safety chain.

See the dual Golden Phoenix power-flow record for the physical connection and charging
restrictions. Charging MOS is not controlled by this package.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bms_node
colcon test --packages-select bms_node
colcon test-result --verbose
```

The tests cover protocol decoding, both CAN addresses, the discharge state machine,
CAN frame filtering, fail-closed configuration, and node shutdown without a CAN device.
