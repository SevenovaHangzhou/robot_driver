# Receiver-time trajectory stop

Opt-in, domain-private extension of the pinned JointTrajectoryController. Existing controller
selection is unchanged. Configure an explicit `bounded_stop_service` endpoint and select
`ctrl_stoppable_trajectory/Controller` to use it. Only position commands, bounded joints and the
existing feedback-age admission check are supported. The endpoint accepts the current FJT UUID
and the motion gear's scalar velocity, acceleration and jerk limits in radians.

Normal FJT motion uses the pinned JTC. A stop request reserves one slot and excludes new goals,
topic commands and ordinary cancels while it is pending. The update that consumes the request
checks the UUID again, samples the old reference at that execution time and constructs a bounded
brake from its full position/velocity/acceleration state. Stops are supported for monotone straight
joint-space paths with full position/velocity/acceleration points. No stop time is predicted by
the client. The original endpoint bounds the braking distance.

The service response is deferred until the reference reaches rest and the configured JTC tracking
tolerances are met, or until failure. The interrupted original FJT goal is ABORTED with
INVALID_GOAL, never reported as having reached its original target. A duplicate while pending
returns BUSY and does not restart braking. A stale UUID cannot stop a newer goal. Losing the
client does not cancel an already accepted stop. A stale feedback sample, tracking failure or
backward execution clock returns controller ERROR and latches rejection until a lifecycle reset.
No new physical safety thresholds are introduced. Reference rest is not proof of physical rest.

The last stop reference is retained until a newly admitted normal goal arrives. The ordinary
JTC QueryTrajectoryState service returns `success=false` while this extension owns the reference;
controller state publications continue with the actual stop reference. Standard FJT cancel
outside the explicit bounded-stop transaction retains JTC's hold semantics.

The operator console selects this path only with `stop_backend: controller`. It discovers the
service from the selected controller's `bounded_stop_service` parameter, waits for its current
FJT acceptance before sending the UUID, and keeps motion blocked while completion is unknown.
It does not substitute ordinary FJT cancel when the bounded-stop request fails. Schema, plugin,
controller selection, console Python and the progress UI must be released together. Defaults
remain on the existing controller and planned-stop backend.

Validation: numeric profiles and real ROS service/update tests pass locally. A target-built
plugin also passes the complete dual-arm/head scope with GenericSystem, the console stop client,
rolling-controller switching and orderly shutdown in isolated networking and shared memory.
Physical-motion and production-container validation remain pending. The derived plugin must be rebuilt against the
deployment's pinned and patched JTC ABI; do not copy a library built against another revision.
