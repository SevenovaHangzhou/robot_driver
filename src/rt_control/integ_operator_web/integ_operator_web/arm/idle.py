"""Request terminal evidence and the controller-owned standstill confirmation."""

import math
import threading
import time


class ArmIdleEvidence:
    def __init__(self, feedback_age, controller_age, *, clock=time.monotonic):
        self.clock, self.feedback_age, self.controller_age = clock, feedback_age, controller_age
        self.lock = threading.RLock()
        self.controllers_at = self.joints_at = self.rolling_at = -math.inf
        self.confirmed_at = self.confirmed_since = -math.inf
        self.joints, self.desired = (), ()
        self.jtc_active = self.rolling_active = self.motion_inactive = False
        self.status_seen, self.active_goal = False, True
        self.pending, self.unknown = 0, False
        self.rolling = None

    def confirm_standstill(self, success, requested_at):
        with self.lock:
            self.confirmed_since = requested_at if success else -math.inf
            self.confirmed_at = self.clock()

    def controllers(self, controllers, jtc):
        with self.lock:
            self.jtc_active = any(c.name == jtc and c.state == "active" for c in controllers)
            inactive = any(c.name == jtc and c.state == "inactive" for c in controllers)
            if inactive:
                # JTC aborts goals on deactivation; there is no initial empty action status.
                self.status_seen, self.active_goal = True, False
            rolling = [c for c in controllers if "RollingTrajectoryController" in c.type]
            self.rolling_active = any(c.state == "active" for c in rolling)
            self.motion_inactive = inactive and all(c.state == "inactive" for c in rolling)
            self.controllers_at = self.clock()

    def controller_state(self, names, desired_velocities):
        with self.lock:
            self.joints, self.desired = tuple(names), tuple(desired_velocities)
            self.joints_at = self.clock()

    def mode_matches(self, target):
        with self.lock:
            if self.clock() - self.controllers_at > self.controller_age:
                return False
            return (
                (self.jtc_active and not self.rolling_active)
                if target == "jtc"
                else (self.rolling_active and not self.jtc_active)
            )

    def goal_status(self, statuses):
        with self.lock:
            self.status_seen = True
            self.active_goal = any(status not in (4, 5, 6) for status in statuses)

    def rolling_state(self, message):
        with self.lock:
            self.rolling, self.rolling_at = message, self.clock()

    def submitted(self):
        with self.lock:
            self.pending += 1

    def resolved(self, *, unknown=False):
        with self.lock:
            self.pending -= 1
            self.unknown |= unknown

    def idle(self, expected_joints, since):
        with self.lock:
            now = self.clock()
            if (
                not expected_joints
                or len(set(expected_joints)) != len(expected_joints)
                or self.pending
                or self.unknown
                or now - self.controllers_at > self.controller_age
                or self.confirmed_since <= since
                or now - self.confirmed_at > self.feedback_age
            ):
                return False
            if self.rolling_active:
                return (
                    not self.jtc_active
                    and self.rolling is not None
                    and self.rolling.state == self.rolling.HOLDING
                    and now - self.rolling_at <= self.feedback_age
                    and self.rolling_at > since
                )
            if self.motion_inactive:
                return True
            return (
                self.jtc_active
                and self.status_seen
                and not self.active_goal
                and now - self.joints_at <= self.feedback_age
                and self.joints_at > since
                and len(self.joints) == len(expected_joints)
                and set(self.joints) == set(expected_joints)
                and len(self.desired) == len(expected_joints)
                and all(math.isfinite(v) and v == 0 for v in self.desired)
            )
