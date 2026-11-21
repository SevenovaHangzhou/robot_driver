"""Explicit local-test deployment data. These numbers are not field safety settings."""

from copy import deepcopy
import yaml

EMERGENCY_INPUT = {
    "receiver_timeout_ms": 250,
    "input": {
        "host": "127.0.0.1",
        "port": 1502,
        "unit_id": 1,
        "input_address": 0,
        "pressed_level": 0,
        "poll_period_ms": 40,
        "request_timeout_ms": 150,
        "stale_timeout_ms": 200,
    },
}


def configure_test_units(config_root):
    for path in (config_root / "units").glob("*/deployment.yaml"):
        data = yaml.safe_load(path.read_text())
        data["emergency_stop"] = deepcopy(EMERGENCY_INPUT)
        path.write_text(yaml.safe_dump(data, sort_keys=False))
