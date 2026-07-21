# CANopen drive adaptation checklist

This table is the operator-facing delta between the supplied LD2 EDS/defaults
and the approved rt-control configuration. It does not authorize powered motion
until every effective value and physical reaction has been archived in T-014.

| Scope/object | Current evidence | Target standard behavior | Drive-side change/persistence | Verification gate |
| --- | --- | --- | --- | --- |
| Master Node 100 heartbeat | `bus.yml` is the generated-DCF input | Produce every 1000 ms; consume Nodes 1/2/3 at 5000 ms | dcfgen owns startup configuration | Inspect master DCF/bin. |
| Drives Nodes 1/2/3, 0x1017 | EDS default 2000 ms; recorded live value 1000 ms | Produce every 1000 ms | dcfgen startup write to 0x1017; persistence is not assumed | SDO read back 1000 before motion. |
| Drives Nodes 1/2/3, 0x1016 | EDS entries default disabled | Consume master Node 100 at 5000 ms | dcfgen startup write to 0x1016; persistence is not assumed | Read back the generated entry before motion. |
| 0x6040/0x6041 state machine | EDS declares mapped RW/RO objects | Accept upstream Motor402's standard CiA402 transitions and masks | Any vendor setting needed to accept standard transitions is TBD; do not patch Motor402 | Cold activation/deactivation trace must reach the expected standard states. |
| PP set-point bits in 0x6040/0x6041 | Upstream uses New Set-point/Immediate and statusword bit 12 acknowledgement | Node 1 must acknowledge each new target through bit 12 so upstream can release/re-arm the set-point edge | Configure the drive for standard immediate PP handshake; exact vendor parameter/persistence TBD | Repeated-target trace must show no missed or permanently asserted handshake. |
| 0x6060/0x6061 | EDS supports and maps both; EDS mode default is PP | Node 1 confirms PP=1; Nodes 2/3 confirm PV=3 | Non-RT hardware activation writes/selects mode through upstream API; no controller mode edge | Read 0x6061 and record confirmation for all nodes. |
| 0x607A/0x6064 | EDS declares mapped target/actual position | Updown holds cached 0x6064 when activation/no new command, then accepts 0x607A PP targets | Ensure the drive does not move on enable before the first safe target; exact vendor option TBD | Supported, unloaded activation test with raw target/actual/status capture. |
| 0x6081 | EDS default 20000; EDS RPDO2 default contains 0x6081 then 0x6083 | Every accepted updown command sends its supplied raw 0x6081 before 0x607A | Preserve deployed RPDO mapping; no runtime remap or SDO confirmation | Upload/archive 0x140x/0x160x and prove velocity changes at supported low speed. |
| 0x6502 supported modes | EDS default `0x0000002D` advertises Homing (bit 5) | `Motor402::handleInit()` must not enter automatic Homing for these absolute/current-position axes | Candidate target is Homing bit cleared (`0x0000000D`), but the writable/persistent vendor method is **TBD and not authorized for automatic startup write** | Read live 0x6502; vendor-confirm the change; cold-init trace must contain no Homing transition. |
| Track Nodes 2/3 master-heartbeat loss | No authoritative reaction setting yet | Controlled stop/Quick Stop, no automatic motion resume | Vendor setting and persistence TBD; no OD value is guessed | Supported low-speed loss test and stopping trace. |
| Track Nodes 2/3 NMT Stop | Mechanical reaction not proven | Stop that track, no automatic motion resume | Vendor setting and persistence TBD | Either track heartbeat/EMCY must make master stop all nodes; verify both tracks physically stop. |
| Updown Node 1 heartbeat/NMT Stop | Mechanical/brake reaction not proven | Safe stop/hold/brake with the axis supported | Vendor setting and persistence TBD | Supported-axis interruption test; never infer brake behavior from NMT state alone. |
| EMCY | Upstream preserves code/register/manufacturer bytes; narrow overlay routes track EMCY to all-node NMT Stop | Track EMCY stops both tracks and updown; updown-only EMCY does not become a mandatory-node group trigger | No raw-CAN parser and no drive EMCY semantic rewrite | Inject/observe supported drive fault and archive diagnostics/NMT states. |
| Unresolved 0x6083/0x6084/0x6085/0x605A/0x605E/0x5010 | EDS has defaults for most; 0x5010 is absent | Use only vendor-approved production behavior | **TBD; no startup write** | Freeze from vendor evidence plus live readback before relying on the reaction. |

The master and all three drives use the same 1000/5000 ms heartbeat contract.
Its main benefit is a single upstream-native communication monitor with low bus
load and fewer nuisance trips. Its accepted drawback is that fault detection
may take up to five seconds before the drive's own stopping behavior begins.

The track position and velocity conversion is also intentionally changed for
the approved 0.2088 m active-sprocket radius: both directions use the exact
`bus.yml` factors. `diff_drive_controller.wheel_radius` remains 1.0 because the
hardware interface is expressed directly in metres and metres/second; applying
0.2088 there again would double-convert the command.
