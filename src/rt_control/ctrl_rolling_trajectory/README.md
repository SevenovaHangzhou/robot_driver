# Rolling stationary command readback

The RT-domain operator adapter uses `/<discovered-controller>/query_state`
(`control_msgs/srv/QueryTrajectoryState`) before the first Rolling window.
Controller discovery supplies the name; it is not machine-specific.

- The request time must be zero. It queries the current stationary command, with
  positions in rad and velocity/acceleration in rad/s and rad/s². It does not
  predict future states.
- Success returns all axes in the configured `joints` order and the coherent RT
  desired position, with zero velocity and acceleration. It never substitutes
  measured encoder positions.
- Inactive, moving, stopping, stale-feedback, inconsistent-snapshot and nonzero
  time requests return `success=false` and empty arrays. Empty priming after a
  rejected first batch becomes queryable once its RT epoch is visible.
- This is a read-only non-RT callback. It does not start a session, consume a
  sequence number, switch modes, enable drives, or alter the RT update path.

The primary operator reserves the existing shared arm authority and mode while
querying. Its 200 ms query timeout is a client transport deadline, not a hardware
limit. It checks fresh HOLDING epoch/sequence/result/stop-reason identity before
and after the query and again before first publication. Query failure does not
fall back to measured positions. The first point must equal the returned command
at zero velocity; the first cubic segment must satisfy the configured acceleration
limit. The planner retains full-curve q/v/a and collision validation and the
controller retains final authority over trajectory admission.

Deploy the controller plugin and primary adapter together, then restart the
control runtime to load the plugin. The updated gateway prefers the primary's
`hold_reference.positions`; its previous completed-target cache remains compatible
with the previous primary during staged upgrades. No public Rolling message schema,
dynamic limit, tolerance, point capacity or running-session timing changes.


## Non-realtime rejection evidence

`~/rejection` publishes a standard `diagnostic_msgs/DiagnosticArray` only when
a batch is rejected. The reliable, transient-local, depth-one report identifies
the batch epoch, sequence and public result. Dynamic-limit and stopping-viability
failures from buffer admission also identify the joint, batch-relative segment,
session-relative interval in nanoseconds, quantity, value and configured limit.
Other failures carry no guessed numerical explanation. Consumers must match
epoch, sequence and result to the public Rolling state and validate joint order.

The optional `BufferRejection` output does not change admission decisions or the
accepted image. Formatting and publishing occur in the serialized non-RT batch
callback, after releasing the initial-handoff gate. The RT update path and public
Rolling schemas are unchanged. This topic provides diagnostics, not command
admission, execution acknowledgement or a motion transport.
