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
| Heartbeat producer | 0x1017 is RW, EDS default `0x07D0` (2000 ms) | This differs from the recorded deployed value 1000 ms. `bus.yml` deliberately omits `heartbeat_producer`, so it does not request a 0x1017 write. |
| 0x6081 | RW UNSIGNED32, PDO-mappable, default 20000 | Present. The EDS default RPDO2 mapping 0x1601 contains 0x6081:00 followed by 0x6083:00, matching AMB-015. |
| 0x5010 | No object section | Absent; no value or write is generated. TBD-004 remains open for manufacturer confirmation. |
| 0x605E | RW INTEGER16, range 0..2, default 0 | Present, but the approved production value/semantics remain unresolved. No value or write is generated. |
| PDO defaults | 0x1600/01/02/03 and 0x1A00/01/02/03 are present | Sufficient for dcfgen to describe remote PDOs, but the deployed mapping must still be uploaded and archived before use. EDS presence is not proof of the live mapping. |

## Configuration write audit

`bus.yml` contains no `sdo`, `rpdo`, `tpdo`, `heartbeat_producer`, or
`sync_period` key. Consequently it requests no additional SDO writes, no PDO
remap, and no SYNC production. The only explicit SDO-related value is
`sdo_timeout_ms: 500`, which sets the host-side request timeout and does not
write an OD entry.

The dcfgen behavior was checked against the Lely implementation used by
ros2_canopen: a slave heartbeat-producer SDO is appended only when the YAML
explicitly contains `heartbeat_producer` and it differs from the EDS value.
When the key is absent, dcfgen reads the EDS value for master-side heartbeat
configuration and does not append a 0x1017 download. The generated DCF/bin must
still be inspected during T-014 before connecting hardware.

The `heartbeat_consumer: true` inherited by each slave section is retained from
the mandated design skeleton. In dcfgen terminology that field means the slave
monitors the master's heartbeat; master monitoring of slave heartbeats is a
separate master option whose default is true. Because the master producer time
is zero and all EDS 0x1016 entries are zero, the retained slave option does not
create an OD write in this configuration. It must not be treated as the source
of the frozen 4000 ms `rt_watchdog` age.

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

Lely/dcfgen can create master-side heartbeat consumers and reports heartbeat
loss as a boot/error-control error. The ROS 2 driver exposes NMT state callbacks,
but `Cia402System` stores only the latest NMT state; it does not expose the last
heartbeat receive timestamp or a heartbeat-age state interface
(`lely_driver_bridge.cpp:31-114`; `cia402_system.cpp:49-77`). Therefore the
frozen 4000 ms watchdog age cannot be implemented solely from the exported
`Cia402System` interfaces.

The EDS default is 2000 ms while the live recorded producer is 1000 ms. Because
0x1017 must not be written, the effective master consumer deadline produced by
dcfgen must be inspected in the generated master DCF. No multiplier is invented
in this task.

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
callbacks, not the available EMCY callback. No typed EMCY state interface is
exported to `rt_watchdog`/`rt_diagnostics`. A downstream exposure mechanism
requires an approved implementation choice (BQ-006).

### PDO feedback freshness

The RPDO callback publishes index/subindex/data into a latest-value cache, and
the generic state interfaces export those values, but no monotonic receipt time
or age is stored (`canopen_system.cpp:157-185`; `lely_driver_bridge.cpp:129-180`).
The frozen 3000 ms PDO-freshness rule therefore needs an approved timestamp
exposure path; it is not a bus.yml setting (BQ-006).

### 0x6081 command channel (AMB-015)

The EDS proves 0x6081 is PDO-mappable in the preconfigured RPDO2. ros2_canopen's
generic CANopen system exports `tpdo/index`, `tpdo/subindex`, `tpdo/data`, and
`tpdo/ons` command interfaces and transmits the selected mapped object on the
one-shot edge (`canopen_system.cpp:188-224,255-281`). In master terminology this
TPDO is the drive's RPDO, so 0x6081 can be carried without an SDO and without a
mapping write if the live mapping matches the archive.

There is no typed 0x6081/profile-velocity command in `Cia402System`; its typed
velocity command targets the selected velocity operation mode, not PP profile
velocity. Ownership and atomic ordering between the updown position command and
the generic 0x6081 one-shot are not specified. BQ-004 records the required
decision for T-007/T-014.

### Mode selection and activation sequence

The design-mandated `operation_mode` keys are retained verbatim in `bus.yml`,
but the pinned Humble `Cia402System` does not parse them. It changes mode only
when its `position_mode_cmd` or `velocity_mode_cmd` command interface receives
an edge (`cia402_system.cpp:329-365`). BQ-003 records the missing ownership of
those mode commands.

The stock `Motor402::handleInit()` reads state, unconditionally arms a fault
reset, and immediately transitions to Operation Enabled. It does not preload
0x607A from 0x6064 for PP or preload 0x60FF=0 for PV
(`canopen_402_driver/src/motor.cpp:351-408`). Mode switching uses a fixed
five-second wait and 20 ms polling fallback rather than the frozen bounded
500 ms / 3 attempts / 50 ms / 1 s / 4.5 s / 16 s sequence
(`motor.cpp:78-173`). Stock PP logic also gates the new-point bit on statusword
bit 12 and does not enforce an unconditional one-cycle 0x003F pulse
(`profiled_position_mode.hpp:61-88`). These are not equivalent to
REQ-CAN-004/005. BQ-005 blocks hardware activation until an implementation
vehicle is authorized and verified.

Quick stop itself is available: `halt_cmd` reaches `Motor402::handleHalt()`,
which requests the Quick Stop Active transition from Operation Enabled
(`cia402_system.cpp:385-390`; `motor.cpp:415-436`). This proves a standard
quick-stop trigger exists, but it does not close the missing heartbeat/PDO-age
inputs described above.

## Requirement result

| Requirement | T-006 result |
| --- | --- |
| REQ-CAN-001 | 500 kbit/s remains a host `can0.service` responsibility in T-009; no conflicting bus value is introduced. |
| REQ-CAN-002 | Exactly nodes 1/2/3 and modes 1/3/3 are represented; the commissioning-only node is absent from all CANopen configuration files. Runtime mode-command ownership remains BQ-003. |
| REQ-CAN-003 | All position/velocity scale strings are copied exactly from the hardware mapping authority. |
| REQ-CAN-004 | Host SDO timeout is 500 ms; stock activation is not equivalent and remains blocked by BQ-005. |
| REQ-CAN-007 | Read-only full-range archive script delivered; no PDO configuration or SYNC key is present. Hardware execution belongs to T-014. |
| REQ-CAN-008 | No unresolved raw value is filled. 0x5010 absence and 0x605E presence are documented; 0x1017 is not written. |

T-006 delivers the configuration template, exact EDS asset, read-only archive
tool, and capability evidence. It does not unlock CANopen hardware activation:
BQ-003/004/005/006 and T-009/T-014 commissioning gates remain in force.
