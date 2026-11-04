"""R-OUT-20 /rt/sensors/state -> readiness warnings behind a watchdog; pure, no ROS.

User 2026-11-04: a single PLC/Modbus timeout recovers by itself and must not reach readiness.
A sensor enters readiness only after it has stayed invalid (or silent) for WATCHDOG_S, and
leaves on its first good status. Entries are warnings: readiness degrades, it does not block.
"""

# Long enough for two consecutive Modbus timeouts (1 s each) plus one 0.2 s poll period.
WATCHDOG_S = 3.0
# SensorStatus measurement states that count as a problem (INVALID, COMMUNICATION_ERROR, STALE).
PROBLEM_STATES = {3, 4, 5}


class SensorWatch:
    def __init__(self):
        self._sensors = {}

    def observe(self, sensor, now):
        """sensor: sensor_id, module_id, state (measurement_state), dree (name), message."""
        previous = self._sensors.get(sensor["sensor_id"])
        since = None
        if sensor["state"] in PROBLEM_STATES:
            since = previous["since"] if previous and previous["since"] is not None else now
        self._sensors[sensor["sensor_id"]] = {**sensor, "seen": now, "since": since}

    def entries(self, now):
        result = []
        for sensor_id, sensor in sorted(self._sensors.items()):
            silent = now - sensor["seen"]
            if silent > WATCHDOG_S:
                dree, detail = "RT_DEVICE_FEEDBACK_STALE", f"no sensor status for {silent:.1f} s"
            elif sensor["since"] is not None and now - sensor["since"] >= WATCHDOG_S:
                dree = sensor["dree"]
                detail = f"{sensor['message']} (for {now - sensor['since']:.1f} s)"
            else:
                continue
            result.append(
                {
                    "source": f"{sensor['module_id']}/{sensor_id}",
                    "dree": dree,
                    "severity": "WARN",
                    "blocking": False,
                    "detail": detail,
                }
            )
        return result
