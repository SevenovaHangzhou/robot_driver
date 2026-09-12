# Inovance GR10-EC-6SW ESI

The user supplied `INOVANCE-GR10-EC-6SW-1.4.2.3.xml` on 2026-09-12 for the Gen3
branch coupler. The earlier typed designation `GR10-EC-65W` is corrected by the
ESI Type text to **GR10-EC-6SW**. Read-only identity checks on 2026-09-12 matched
both coupler entries; the observed device-name suffix is `1.4.2.0`, while this
ESI's name/file suffix is `1.4.2.3`. Matching identities do not establish firmware
or full configuration equivalence.

Original file SHA-256:
`cd47a205dc70f0ee0f91b95c2c8ca72b711e11ce55602f2566cf94d1297b98e3`.

The repository copy normalizes CRLF/trailing whitespace and adds the final newline;
XML elements, attributes and values are unchanged. Repository copy SHA-256:
`ea49828fc0ea72c6fc2d8ed4378cc078a790e0e24d3fbd5f85e056c5c91358a4`.

## Declared identities

Vendor ID: `0x00100000` (Inovance).

| Component | Product code | Revision | ESI port entries in order |
| --- | --- | --- | --- |
| GR10-EC-6SW | `0x10F40931` | `0x00010000` | IN, X3, Internal Port, X2 |
| GR10-EC-6SW Sub-device | `0x10F40932` | `0x00010000` | Internal connection, X5, X6, X4 |

The main entry declares the second component as a SubDevice. The second entry
refers back using `PreviousDevice="0" PreviousPortNo="2"`. The main internal
port and the child incoming port are EBUS; the external ports are MII. ESI port
entry order is not the same as ascending physical connector labels, and does
not by itself determine the slave enumeration once motor chains are attached.

Both Device entries belong to `JunctionSlave`. This ESI declares no RxPDO,
TxPDO, mailbox or SM elements. It advertises two operation modes: DC with
AssignActivate `0x0300`, and Synchron with `0x0000`. This is a capability
declaration, not a decision to enable DC or a measured OP/PREOP requirement.
The `1.4.2.3` display/file version is separate from the identity revision above.

## Gen3 integration boundary

Treat the coupler components as non-actuator responders, not CiA402 axes or
members of JTC, gripper controllers or the enable_manager motion-axis list.
The 2026-09-12 arm-bench scan confirmed **18 bus-visible responders and 16
actuators** in PREOP. Counts and identities do not establish cyclic WKC or
runtime state requirements.

| Observed component | Connector / ESC port | Absolute ring positions |
| --- | --- | --- |
| Main coupler | IN | 0 |
| Left arm (user-confirmed side) | X2 / main port 3 | 1..8 |
| Right arm (user-confirmed side) | X3 / main port 1 | 9..16 |
| Coupler sub-device | main internal port 2 | 17 |

Main port 3 reports NextSlave 1, port 1 reports NextSlave 9, and port 2 reports
NextSlave 17. Each arm is an eight-drive chain. All sixteen drives report
Vendor `0x5A65726F`, Product `0x00029252`, Revision `0x00000001`, Serial `0`.
These identical identities cannot distinguish J1..J7 from the gripper. The user
subsequently confirmed both chains as J1..J7 followed by the gripper: the physical
mapping and requested CSP/PP modes are recorded in
[`alfa_v3_arms_only.draft.yaml`](../machines/alfa_v3_arms_only.draft.yaml).
Formal Robot Model names and runtime drive profiles remain TBD. Main/sub-device stored
aliases read 2/7; these are distinct from the absolute positions 0/17 above.

Do not copy the old robot's Hub 0/13 positions, identity or OP/PREOP exceptions.
State/DC handling, joint bindings and PDO/SDO qualification are still pending.
The mapping above applies only to the observed arms-only bench; other physical
profiles require their own scans. This asset is installed for review; no runtime
descriptor selects or enables it yet. See BQ-146 and ecat-axes-20260912-03 for
the read-only evidence and remaining commissioning requirements.
