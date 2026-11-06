"""Sensor problems reach readiness only after the watchdog, and leave on the first good status."""

from integ_diagnostics.sensors import SensorWatch, WATCHDOG_S

VALID, INVALID, COMMUNICATION_ERROR, NOT_CONFIGURED = 1, 3, 4, 6


def status(state, sensor_id="left_vacuum_sensor"):
    return {
        "sensor_id": sensor_id,
        "module_id": "plc_analog_module",
        "state": state,
        "dree": "RT_PLC_UNAVAILABLE",
        "message": "Modbus timeout",
    }


def test_a_single_timeout_recovers_without_reaching_readiness():
    watch = SensorWatch()
    watch.observe(status(VALID), 0.0)
    watch.observe(status(COMMUNICATION_ERROR), 1.0)
    assert watch.entries(1.0 + WATCHDOG_S - 0.1) == []
    watch.observe(status(VALID), 2.2)
    assert watch.entries(4.0) == []


def test_a_lasting_problem_is_a_warning_and_clears_on_the_first_good_status():
    watch = SensorWatch()
    for second in range(5):
        watch.observe(status(COMMUNICATION_ERROR), float(second))
    [entry] = watch.entries(4.0)
    assert (entry["source"], entry["dree"], entry["blocking"]) == (
        "plc_analog_module/left_vacuum_sensor",
        "RT_PLC_UNAVAILABLE",
        False,
    )
    assert "Modbus timeout" in entry["detail"]
    watch.observe(status(VALID), 4.5)
    assert watch.entries(4.5) == []


def test_silent_sensor_is_stale_and_unconfigured_sensors_are_ignored():
    watch = SensorWatch()
    watch.observe(status(VALID), 0.0)
    watch.observe(status(NOT_CONFIGURED, "right_vacuum_sensor"), 0.0)
    for second in range(1, 10):
        watch.observe(status(NOT_CONFIGURED, "right_vacuum_sensor"), float(second))
    [entry] = watch.entries(9.0)
    assert entry["source"].endswith("left_vacuum_sensor")
    assert entry["dree"] == "RT_DEVICE_FEEDBACK_STALE"
