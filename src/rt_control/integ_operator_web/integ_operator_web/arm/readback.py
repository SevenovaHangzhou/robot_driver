"""Read-only terminal feedback and EnableManager standstill queries."""

from action_msgs.msg import GoalStatusArray
from std_srvs.srv import Trigger
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
        self.motion_joints, self.motion_joints_at = (), -float("inf")
        self.standstill_client, self.pending = None, None
        console.idle_probe = self.probe
        node.create_subscription(
            RollingJointControlState,
            "/rt/arms/rolling/state",
            self.evidence.rolling_state,
            robot_interfaces_qos.rolling_state(),
            callback_group=group,
        )

    def bind(self, controller, action):
        if self.bound:
            return
        self.bound = True
        self.node.create_subscription(
            JointTrajectoryControllerState,
            f"/{controller}/controller_state",
            lambda m: self.evidence.controller_state(m.joint_names, m.desired.velocities),
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

    def probe(self, since):
        # ArmPage owns authority polling. This query only gathers read-only evidence.
        with self.evidence.lock:
            now = self.evidence.clock()
            if self.pending is not None and now - self.pending[1] > STATE_STALE_S:
                self.standstill_client.remove_pending_request(self.pending[0])
                self.pending = None
                self.evidence.confirm_standstill(False, now)
            scope_matches = (
                self.motion_joints == tuple(self.console.joint_names)
                and now - self.motion_joints_at <= STATE_STALE_S
            )
            if not scope_matches:
                self.evidence.confirm_standstill(False, now)
            if (
                scope_matches
                and self.pending is None
                and self.console.joint_names
                and self.standstill_client is not None
                and self.standstill_client.service_is_ready()
            ):
                future = self.standstill_client.call_async(Trigger.Request())
                self.pending = (future, now, self.motion_joints)
                future.add_done_callback(self._confirmed)
            return self.evidence.idle(self.console.joint_names, since)

    def _confirmed(self, future):
        with self.evidence.lock:
            if self.pending is None or future is not self.pending[0]:
                return  # A late response cannot replace a newer query's evidence.
            requested_at, joints = self.pending[1:]
            self.pending = None
            try:
                success = future.result().success and joints == self.motion_joints
                success &= self.evidence.clock() - self.motion_joints_at <= STATE_STALE_S
            except Exception:
                success = False
            self.evidence.confirm_standstill(success, requested_at)
