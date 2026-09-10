# IgH Experiments

These patches are opt-in diagnostic candidates, not part of the default
host or Docker installation. Do not interpret an offline build as hardware
qualification.

`0003-dc-offset-trigger-100us.patch` applies to frozen IgH commit
`2f7f884f1c7d377c02a7d627eb06512126a0e50e` after the PDO-preservation and
sent-application-time patches in the parent directory. It changes only the
initial offset-adjustment trigger from 1,000,000 ns to 100,000 ns.
The 10,000 ns DC acceptance condition, 5,000 ms wait limit, PDOs, SYNC0
configuration and cyclic handoff remain unchanged.

Purpose: test recorded submillisecond clock offsets that were left unchanged
by the 1 ms trigger. This cannot by itself address all observed slow DC
convergence: some measured startups remained slow after every initial offset
was adjusted.

Validation is limited to controlled, disabled startup comparisons until the
experiment record states otherwise. The underlying offset FSM may also run
after reconfiguration; enabled runtime reconfiguration is not qualified.

The September 9 hardware comparison completed three disabled candidate
startups in 17.684, 15.084 and 16.244 seconds. Baseline startups before and
after took 22.644 and 25.524 seconds. Initial offsets and idle intervals
differed, so these observations do not establish a repeatable speedup.
The original 1 ms modules and metadata were restored after the experiment.

Build evidence, module identities, deployment status and rollback prerequisites
are in the [ELECTRI-97 experiment record](../../../domains/rt_control/docs/areas/realtime-host/records/2026-09-09-electri-97-offset-trigger-100us-candidate.md).
