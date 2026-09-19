# damiao_head_controller

This V3 package owns the lifecycle boundary around the two DaMiao head axes. It is
deliberately not a `joint_trajectory_controller`: `head_position_controller` is a
plain `forward_command_controller/ForwardCommandController`, while this manager
owns only the hardware `enable_request` and `reset_generation` interfaces.

Services are scoped under `/rt/head` so a future full-robot `enable_manager` can
retain the top-level `/rt/enable`, `/rt/disable`, and `/rt/reset_fault` names.
The head-only launch starts the manager active and the position controller inactive.

The manager waits for both motors to report enabled before activating the position
controller, deactivates that controller before requesting a dual-axis disable, and
latches feedback/fault failures until the explicit reset path completes. Diagnostics
are published as `/robot/rt_control/damiao_head/summary` on `/diagnostics`.

After `/rt/head/enable` succeeds, send a two-element `std_msgs/msg/Float64MultiArray`
to `/head_position_controller/commands` in `[head_joint, head_pitch_joint]` order. The
hardware plugin rejects targets outside the configured mechanical bounds; it does not
silently clamp them.
