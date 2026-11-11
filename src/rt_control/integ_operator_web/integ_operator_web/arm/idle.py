"""Read-only evidence for arm-resource startup and handover. No ROS imports."""

from collections import deque
import math
import threading
import time


class ArmIdleEvidence:
    def __init__(self, feedback_age, controller_age, *, clock=time.monotonic):
        self.clock, self.feedback_age, self.controller_age = clock, feedback_age, controller_age
        self.lock = threading.RLock()
        self.controllers_at = self.joints_at = self.rolling_at = -math.inf
        self.joints, self.positions, self.velocities, self.desired = (), (), (), ()
        self.jtc_active = self.rolling_active = False
        self.motion_inactive = False
        self.jtc, self.rolling_present = "", False
        self.configured_motion = None
        self.configured_motion_at = -math.inf
        self.status_seen, self.active_goal = False, True
        self.tolerance = None  # From JTC constraints.stopped_velocity_tolerance, never a fallback.
        self.pending, self.unknown = 0, False
        self.rolling = None
        self.measured_joints, self.measured_velocities = (), ()
        self.measured_at = self.tolerance_at = -math.inf
        self.stationary_policy = None
        self.stationary_at = -math.inf
        self.samples = deque(maxlen=4097)

    def measured(self, names, positions, stamp_ns=None):
        with self.lock:
            now = self.clock()
            valid = (stamp_ns is not None and stamp_ns > 0 and
                     len(names) == len(positions) == len(set(names)) and
                     all(math.isfinite(v) for v in positions))
            if not valid or (self.samples and (stamp_ns <= self.samples[-1][1] or
                                               now - self.samples[-1][0] > self.feedback_age)):
                self.samples.clear()
            if valid:
                self.samples.append((now, stamp_ns, dict(zip(names, positions))))
            self.measured_at = now

    def set_stationary_policy(self, names, thresholds, intervals):
        with self.lock:
            valid = (len(names) == len(thresholds) > 0 and len(set(names)) == len(names)
                     and type(intervals) is int and 1 <= intervals <= 4096
                     and all(math.isfinite(v) and v > 0 for v in thresholds))
            self.stationary_policy = (tuple(names), tuple(thresholds), intervals) if valid else None
            self.stationary_at = self.clock()

    def _measured_stopped(self, expected, since):
        policy = self.stationary_policy
        if policy is None or self.clock() - self.stationary_at > self.controller_age:
            return False
        names, limits, intervals = policy
        if set(names) != set(expected) or len(self.samples) < intervals + 1:
            return False
        samples = list(self.samples)[-intervals-1:]
        if samples[0][0] <= since or self.clock() - samples[-1][0] > self.feedback_age:
            return False
        elapsed = (samples[-1][1] - samples[0][1]) / 1e9
        if elapsed <= 0 or any(not set(names) <= set(q) for _, _, q in samples):
            return False
        return all((max(q[n] for _, _, q in samples) - min(q[n] for _, _, q in samples))
                   / elapsed <= limit for n, limit in zip(names, limits))

    def set_tolerance(self, value):
        with self.lock:
            self.tolerance, self.tolerance_at = value, self.clock()

    def set_motion_controllers(self, jtc, rolling):
        with self.lock:
            self.configured_motion = (jtc, rolling)
            self.configured_motion_at = self.clock()

    def _configured_jtc_only(self):
        return (bool(self.jtc) and self.configured_motion == (self.jtc, "")
                and self.clock() - self.configured_motion_at <= self.controller_age)

    def controllers(self, controllers, jtc):
        with self.lock:
            self.jtc = jtc
            self.jtc_active = any(c.name == jtc and c.state == "active" for c in controllers)
            if any(c.name == jtc and c.state == "inactive" for c in controllers):
                # JTC aborts its goal on deactivation and rejects goals while inactive.
                # Its action server does not publish an initial empty status array.
                self.status_seen, self.active_goal = True, False
            rolling = [c for c in controllers if "RollingTrajectoryController" in c.type]
            self.rolling_present = bool(rolling)
            self.rolling_active = any(
                c.state == "active" for c in rolling
            )
            self.motion_inactive = (
                any(c.name == jtc and c.state == "inactive" for c in controllers)
                and (bool(rolling) or self._configured_jtc_only())
                and all(c.state == "inactive" for c in rolling)
                and not self.jtc_active
            )
            self.controllers_at = self.clock()

    def controller_state(self, names, actual_positions, actual_velocities, desired_velocities):
        with self.lock:
            self.joints = tuple(names)
            self.positions, self.velocities, self.desired = map(
                tuple, (actual_positions, actual_velocities, desired_velocities)
            )
            self.joints_at = self.clock()

    def mode_matches(self, target):
        with self.lock:
            if self.clock() - self.controllers_at > self.controller_age:
                return False
            return ((self.jtc_active and not self.rolling_active) if target == "jtc"
                    else (self.rolling_active and not self.jtc_active))

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
            ):
                return False
            if self.rolling_active:
                state = self.rolling
                return (not self.jtc_active and state is not None and state.state == 0
                        and now - self.rolling_at <= self.feedback_age and self.rolling_at > since
                        and self._measured_stopped(expected_joints, since))
            if self.motion_inactive:
                # Inactive controllers publish no live target state. Use the same
                # configured position window as Rolling, after the handover barrier.
                if not self.rolling_present and not self._configured_jtc_only():
                    return False
                return self._measured_stopped(expected_joints, since)
            limit = self.tolerance
            if not (isinstance(limit, (int, float)) and math.isfinite(limit) and limit > 0
                    and now - self.tolerance_at <= self.controller_age):
                return False
            if (
                not self.jtc_active
                or not self.status_seen
                or self.active_goal
                or now - self.joints_at > self.feedback_age
                or self.joints_at <= since
                or set(self.joints) != set(expected_joints)
                or len(self.joints) != len(expected_joints)
            ):
                return False
            count = len(self.joints)
            if not (len(self.positions) == len(self.desired) == count):
                return False
            if (not all(math.isfinite(v) for v in self.positions)
                    or not all(math.isfinite(v) and v == 0 for v in self.desired)):
                return False
            if not self.velocities:
                # Position-only hardware needs measured motion evidence, not invented zeros.
                return self._measured_stopped(expected_joints, since)
            return len(self.velocities) == count and all(
                math.isfinite(v) and abs(v) <= limit for v in self.velocities)
