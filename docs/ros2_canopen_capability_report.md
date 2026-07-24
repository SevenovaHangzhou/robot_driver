# ros2_canopen capability report (T-006)

## Scope and pinned inputs

This report checks the CANopen configuration boundary required by
REQ-CAN-001/002/003/004/007/008. It does not authorize hardware enablement,
PDO remapping, or writes to unresolved safety objects.

- Hardware numeric authority: `ecat_canopen_hardware_mapping_export.md`.
- EDS input: `LD2-CAN系列设备描述文件.eds`, copied byte-for-byte as
  `src/rt_control/robot_hw_canopen/config/eds/ld2_drive.eds`.
- EDS SHA-256: `e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08`.
- ros2_canopen source: `/home/kkozia/rt_control_refs/ros2_canopen` at
  `fef50e54b1c94c50e908e2c5d0b8888eed907e8d` (Humble reference checkout).

## EDS audit

| Item | Evidence in `ld2_drive.eds` | Result |
| --- | --- | --- |
| Identity | 0x1018:01=`0x00000331`; :02=`0x5`; :03=`0x00000100` | Matches the supplied Leadshine family identity; real-device 0x1018 readback remains a commissioning gate. |
| Heartbeat producer | 0x1017 is RW, EDS default `0x07D0` (2000 ms) | The approved production value is 1000 ms. `bus.yml` deliberately requests the generated startup write. |
| 0x6081 | RW UNSIGNED32, PDO-mappable, default 20000 | Present. The EDS default RPDO2 mapping 0x1601 contains 0x6081:00 followed by 0x6083:00, matching AMB-015. |
| 0x5010 | No object section | Absent; no value or write is generated. TBD-004 remains open for manufacturer confirmation. |
| 0x605E | RW INTEGER16, range 0..2, default 0 | Present, but the approved production value/semantics remain unresolved. No value or write is generated. |
| PDO defaults | 0x1600/01/02/03 and 0x1A00/01/02/03 are present | Sufficient for dcfgen to describe remote PDOs, but the deployed mapping must still be uploaded and archived before use. EDS presence is not proof of the live mapping. |

## Configuration write audit

`bus.yml` contains no explicit `sdo`, `rpdo`, `tpdo`, or `sync_period` key.
Consequently it requests no PDO remap and no SYNC production. The approved
`heartbeat_producer: 1000`, `heartbeat_consumer: true`, and
`heartbeat_multiplier: 5.0` keys intentionally make dcfgen configure 0x1017
and 0x1016 for symmetric 1000 ms production and 5000 ms consumption. Those
two communication objects are the only newly authorized generated startup
writes. `sdo_timeout_ms: 500` sets the host-side request timeout and does not
write an OD entry.

The dcfgen behavior was checked against the Lely implementation used by
ros2_canopen. In dcfgen terminology, the node-side consumer makes each drive
monitor the master's heartbeat; the master-side consumer monitors node
heartbeats. The generated DCF/bin and effective drive values must still be
inspected/read back during T-014 before powered motion. No separate
`rt_watchdog` heartbeat-age path remains.

`tools/canopen_sdo_archive.sh` calls only the per-node `CORead` service. It
uploads every subindex slot in the frozen ranges 0x1400..0x1403,
0x1600..0x1603, 0x1800..0x1802, and 0x1A00..0x1A02 while a candump capture is
active. Unsupported subindices are retained in the log as failed reads rather
than silently omitted. The script refuses to overwrite an existing archive.

## ros2_canopen capability findings

### Control-loop behavior

`Cia402System::on_configure()` creates a separate executor/spin thread and a
separate initialization thread. Its control-manager `read()` retrieves cached
position/speed values and performs no SDO request or blocking wait
(`canopen_ros2_control/src/cia402_system.cpp:82-112,239-258`). CAN handling is
therefore outside the 250 Hz control-manager loop, consistent with the planned
50 Hz `period: 20` sub-sampling. The upstream source nevertheless uses mutexes
inside motor accessors; this is a capability observation, not a claim that the
upstream implementation is lock-free.

### Heartbeat and NMT

Lely/dcfgen creates the approved bidirectional heartbeat consumers and reports
heartbeat loss as a boot/error-control error. The production configuration uses
a 1000 ms producer and multiplier 5 in both directions, hence a 5000 ms
communication-fault detection bound. The ROS 2 driver does not expose a
continuously increasing heartbeat age, and no custom age interface is added.

### Host bitrate and bus-off recovery

The EDS advertises 500 kbit/s support, while the actual SocketCAN bitrate stays
owned by T-009's `can0.service` (`bitrate 500000`). Per the implementation spec,
no `restart-ms` value is set. Bus-off recovery therefore follows the CAN driver
default and remains an observed commissioning property rather than an invented
configuration value.

### EMCY

`LelyDriverBridge::OnEmcy()` preserves the emergency error code, error register,
and five manufacturer bytes in an internal queue
(`canopen_base_driver/src/lely_driver_bridge.cpp:183-192`). The base driver can
invoke an EMCY callback and its diagnostic updater marks an emergency as ERROR.
However, `Cia402System::initDeviceContainer()` registers only NMT and RPDO
callbacks, not the available EMCY callback. The approved narrow upstream overlay
adds track-EMCY-to-group-NMT-Stop behavior; diagnostics retains the upstream
native EMCY fields instead of creating a watchdog input.

### PDO feedback freshness

The RPDO callback publishes index/subindex/data into a latest-value cache, and
the generic state interfaces export those values, but no monotonic receipt time
or age is stored (`canopen_system.cpp:157-185`; `lely_driver_bridge.cpp:129-180`).
The former custom updown PDO-age gate was explicitly withdrawn; PP admission
uses the approved 0.05 m expected-start check against the exported position.

### 0x6081 command channel (AMB-015)

The EDS proves 0x6081 is PDO-mappable in the preconfigured RPDO2. ros2_canopen's
generic CANopen system exports `tpdo/index`, `tpdo/subindex`, `tpdo/data`, and
`tpdo/ons` command interfaces and transmits the selected mapped object on the
one-shot edge (`canopen_system.cpp:188-224,255-281`). In master terminology this
TPDO is the drive's RPDO, so 0x6081 can be carried without an SDO and without a
mapping write if the live mapping matches the archive.

There is no typed 0x6081/profile-velocity command in `Cia402System`; its typed
velocity command targets the selected velocity operation mode, not PP profile
velocity. The approved updown controller therefore owns the generic one-shot
and the position interface together. In one `write()` pass, ros2_canopen
processes the 0x6081 one-shot before the typed PP target. No SDO readback,
additional delay, or acknowledgement gate is added.

### Mode selection and activation sequence

The design-mandated `operation_mode` keys are retained verbatim in `bus.yml`.
The pinned Humble `Cia402System` does not consume them by itself, so the
approved narrow lifecycle overlay reads them during non-RT hardware activation,
calls the existing upstream init/mode APIs, confirms 0x6061, and seeds current
position for updown plus zero for both tracks.

The stock `Motor402::handleInit()` reads state, arms its standard fault-reset
path, transitions to Operation Enabled, and automatically executes Homing when
0x6502 advertises it (`motor.cpp:351-408`). Stock PP also follows the standard
statusword-bit-12 set-point acknowledgement handshake
(`profiled_position_mode.hpp:61-88`). These behaviors supersede the former
hardware-specific REQ-CAN-004/005 ordering/pulse rules by explicit user
decision. The drive-adaptation checklist identifies the resulting drive-side
requirements, especially removal of an unintended Homing advertisement.

Quick stop itself is available: `halt_cmd` reaches `Motor402::handleHalt()`,
which requests the Quick Stop Active transition from Operation Enabled
(`cia402_system.cpp:385-390`; `motor.cpp:415-436`). Normal lifecycle teardown
instead uses upstream `handleShutdown()` after seeding safe targets; Quick Stop
remains an emergency path.

### T-014 live correction: mapped mode cache and cleanup ownership

The first live activation disproved the assumption that calling only the typed
mode API was sufficient. Because 0x6060 is in the configured RPDO image, its
Lely-side cached default zero was sent again by the periodic PDO path after the
API selected PP/PV. The approved overlay now validates the exact node mapping
and primes that existing cache with Node 1=`1`, Nodes 2/3=`3` before
`init_motor()`. This is neither a new PDO nor an SDO gate and adds no periodic
bus load. A no-command cold run confirmed 0x6061 reports of 1/3/3, periodic
cached commands of 1/3/3, and zero track targets throughout.

Live 0x6502 is read-only `0x0003002D`, so drive-side removal of its Homing bit
is not available. The overlay therefore suppresses only automatic Homing mode
allocation for these fixed axes. Normal shutdown also retains the configured
mode while using the upstream controlword state machine to reach Switch On
Disabled; selecting No Mode first is incompatible with LD2 and caused Er870.

Finally, `Motor402` owns mode helpers that retain the Lely fiber driver. Driver
cleanup releases this graph on the still-running Lely executor before master
shutdown. The corrected hardware-only stop completed in 1.903 seconds with
container exit 0, all nodes repeatedly reporting NMT Stopped, no nonzero EMCY,
no new CAN error/drop counters, and no fiber-executor assertion. BQ-112 records
the evidence, benefits, drawbacks, and the exact portions of earlier decisions
that this live result supersedes. Powered PP/PV and fault-reaction acceptance
remain open T-014 work.

## Requirement result

| Requirement | T-006 result |
| --- | --- |
| REQ-CAN-001 | 500 kbit/s remains a host `can0.service` responsibility in T-009; no conflicting bus value is introduced. |
| REQ-CAN-002 | Exactly nodes 1/2/3 and modes 1/3/3 are represented; the commissioning-only node is absent from all CANopen configuration files. Non-RT hardware activation owns mode initialization. |
| REQ-CAN-003 | Updown scaling remains exact. The user-approved 0.2088 m sprocket radius supersedes the old track scale: position and velocity both use `-304894.5269959681` to-device and `-3.27982273034774e-6` from-device with zero offsets. |
| REQ-CAN-004 | Host SDO timeout is 500 ms. The former custom activation order is explicitly superseded by upstream Motor402 plus the approved non-RT lifecycle orchestration; drive adaptation/readback remains a powered-motion gate. |
| REQ-CAN-007 | Read-only full-range archive script delivered; no PDO configuration or SYNC key is present. Hardware execution belongs to T-014. |
| REQ-CAN-008 | No unresolved motion/safety value is filled. 0x5010 absence and 0x605E presence remain documented; only the separately approved heartbeat objects 0x1016/0x1017 are generated at startup. |

T-006 delivers the configuration template, exact EDS asset, read-only archive
tool, capability evidence, and the drive-adaptation checklist. It does not
unlock powered CANopen motion: generated-DCF inspection, live OD readback,
drive-local stop-reaction proof, and T-009/T-014 commissioning gates remain in
force.
