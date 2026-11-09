"""Public feedback subscriptions used only to confirm arm-resource idleness."""

from action_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from robot_rt_control_interfaces.msg import RollingJointControlState
import robot_interfaces_qos

from .console import STATE_STALE_S
from .idle import ArmIdleEvidence


class ArmReadback:
    def __init__(self, node, group, console):
        self.node, self.group, self.console = node, group, console
        self.evidence = ArmIdleEvidence(console.config.max_feedback_age, STATE_STALE_S)
        self.bound = False
        console.idle_probe = lambda since: self.evidence.idle(console.joint_names, since)
        node.create_subscription(
            RollingJointControlState,
            "/rt/arms/rolling/state",
            self.evidence.rolling_state,
            robot_interfaces_qos.rolling_state(),
            callback_group=group,
        )
        node.create_timer(0.05, console.poll_authority, callback_group=group)

    def bind(self, controller, action):
        if self.bound:
            return
        self.bound = True
        self.node.create_subscription(
            JointTrajectoryControllerState,
            f"/{controller}/controller_state",
            lambda m: self.evidence.controller_state(
                m.joint_names, m.actual.positions, m.actual.velocities, m.desired.velocities
            ),
            qos_profile_sensor_data,
            callback_group=self.group,
        )
        self.node.create_subscription(
            GoalStatusArray,
            action + "/_action/status",
            lambda m: self.evidence.goal_status([s.status for s in m.status_list]),
            qos_profile_action_status_default,
            callback_group=self.group,
        )
