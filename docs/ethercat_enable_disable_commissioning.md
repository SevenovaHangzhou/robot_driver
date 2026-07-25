# EtherCAT enable/disable commissioning record — 2026-07-25

## Status and scope

This is a **partial T-013 record**, not T-013 completion. It covers the EtherCAT
portion of specification section 16 item 5: bus admission, explicit group fault
reset, five-batch enable, current-position hold, normal disable, and orderly
container shutdown. No trajectory or point-motion command was sent. The required
12-arm-joint low-speed trajectory and `turn` jog in section 16 item 7 remain
unexecuted.

The powered evidence is retained on the IPC at:

- `/var/lib/rt-control/commissioning/ecat-enable-disable-20260725/run-6`
- `/var/lib/rt-control/commissioning/ecat-enable-disable-20260725/run-7-diag-clear`

Run 6 proved the hardware state-machine correction. Run 7 repeated only the
minimum path needed to verify that successful recovery also clears stale
diagnostic failure fields.

## Deployed implementation under test

- ICube overlay library SHA-256:
  `0369900b1048e2a5bd4e7d7143df50ebe773fd8ebf33153978523bdf069b8f46`.
- Run-6 enable-manager library SHA-256:
  `196b296a9e33d38b2703ce79b471440d3b8b891e55784f90a74590b9abe3333b`.
- Run-7 enable-manager library SHA-256 after the diagnostic cleanup:
  `5d6036dc82265e392f86ef0ebb53af78ec261f85874a4c18bb709b4a5e3db196`.
- Run-7 41-file evidence manifest SHA-256:
  `a71c48871d14de6bdc738c44e564101787a885449ae5c6f47c1185c3e44e777e`.
- The test used an incremental library injection into the stopped production
  container. The committed Dockerfile remains the reproducible build path; this
  injection is commissioning evidence, not a release-image construction method.

The EtherCAT hardware activation now has two separate bounded gates. It first
waits up to 70 s for every configured motion slave to report AL=OP and for a
complete working counter to produce a finite process-data age. Only after that
condition is true does the existing 5 s raw `0x6064 -> 0x607A` preload deadline
begin. Non-motion hub position 13 is not invented as a configured slave.

## Powered results

| Check | Result |
| --- | --- |
| Stable topology | 15 responding positions; positions 1..12 and 14 OP; position 0 hub OP; position 13 hub PREOP |
| Process domain | `WorkingCounter 39/39` before reset, after enable, and after disable |
| Group fault reset | `ok=true`, `stage=success` |
| Five-batch enable | `ok=true`, `stage=success`; run 6 duration 2.353 s; run 7 duration 2.508 s |
| JTC lifecycle | `dual_arm_jtc` ACTIVE only after hardware enable; INACTIVE after disable |
| Normal disable | `ok=true`, `stage=success`; run 6 duration 1.493 s; run 7 duration 1.574 s |
| Enable hold excursion | run 6 maximum 0.204620 degrees; run 7 maximum 0.016479 degrees; both below the 1 degree admission limit |
| Disabled hold excursion | run 6 maximum 0.009613 degrees; run 7 maximum 0.004807 degrees |
| Success diagnostics | `failed_batch=-1`, empty `failed_joint`, `failed_status_word=0`, `stage=success` in both ENABLED and IDLE |
| Orderly stop | container exit 0; master `Idle / Active:no`; all 15 positions PREOP |
| CAN side-effect check | `can0` remained 500 kbit/s, queue length 128, ERROR-ACTIVE, with zero bus errors/warnings/passive/bus-off increments |

No commanded movement occurred. The recorded position changes are feedback
excursions while the activated JTC held the measured startup position.

## Confirmed hardware-specific terminal behavior

With controlword `0x0000`, the nine ZeroErr axes reach Switch On Disabled. The
four Ti5 axes at ring positions 2, 3, 8, and 9 accept the same controlword but
remain in Ready To Switch On. Read-only SDO uploads confirmed `0x6040=0x0000`
and `0x603F=0` before enable; a separate trial of `0x0002` did not move those
drives to Switch On Disabled. The supplied Ti5 protocol manual defines Ready To
Switch On as motor not energized. The implementation therefore accepts this
state as a non-energized disable terminal only for logical axes
`right_joint2`, `right_joint3`, `left_joint2`, and `left_joint3`; every other
axis still requires Switch On Disabled.

This is a highlighted compatibility exception, not proof that frozen
REQ-SAFE-002 is met literally. On master release, all four Ti5 axes still enter
Fault with `0x603F=0x7500` despite controlword `0x0000`, so the next supported
launch requires an explicit `/rt/reset_fault`. Drive-side communication-loss
configuration must be reviewed before final acceptance; host software must not
guess or silently rewrite that safety reaction.

## Remaining high-risk observations

1. After `EthercatDriver::on_activate()` hands cyclic I/O to the normal
   controller-manager read/write loop, the ring has a one-time synchronization
   cascade. IgH reports AL status `0x001A`, and WC temporarily drops below
   `39/39`; it recovered to `39/39` before reset/enable in both successful runs.
   A tested five-second “stable before return” delay did not prevent the
   cascade and was removed. The accumulated WC error count then plateaued, but
   the root cause is not closed.
2. Ti5 PDO assignment attempts return SDO abort `0x06010002` because the mapping
   objects are read-only. IgH reports that the currently mapped entries exactly
   equal the requested frozen `0x1601/0x1A01` entries. This explains the warning
   but does not justify suppressing it or changing PDOs.
3. Master release produces Ti5 `0x7500` as described above. This prevents a
   clean no-reset restart claim even though normal `/rt/disable` itself succeeds
   and leaves all axes non-energized.

Benefits of the retained startup gate are fail-closed admission and a fresh raw
preload. Its drawbacks are a possible 70 s failed-start delay and no correction
of the later one-time DC handoff cascade. Benefits of the Ti5 exception are a
truthful non-energized terminal and bounded service completion. Its drawbacks
are the explicit deviation from literal `0x0040` confirmation and continued
dependence on a group reset after each master release.

## Safe final state

After run 7, `robot-rt-control-1` is stopped with exit code 0, EtherCAT master 0
is idle and inactive, all slaves are PREOP, and `can0` is ERROR-ACTIVE with zero
error counters. The temporary build container and the two transferred source
scratch files were removed; the built library and all evidence remain in the
run-7 evidence directory.
