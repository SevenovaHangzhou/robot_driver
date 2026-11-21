"""Bind a unit's emergency input to its consumers and owned launch process."""

import yaml


def configure_emergency_input(deployment, controllers, files, *, mock):
    consumers = [
        value["ros__parameters"]
        for value in controllers.values()
        if value["ros__parameters"].get("emergency_stop_response", False)
    ]
    if not consumers:
        return []
    settings = deployment.get("emergency_stop", {})
    timeout = settings.get("receiver_timeout_ms", "TBD")
    source = settings.get("input", {})
    if timeout != "TBD":
        for parameters in consumers:
            parameters["emergency_stop_timeout_ms"] = timeout
    if mock:
        # A draft may be composed offline. With no explicit timeout its controller still
        # refuses configure; tests that activate it must supply their own unit timing.
        # Never connect a mock scope to physical emergency IO.
        return []
    if timeout == "TBD" or not source or "TBD" in source.values():
        raise ValueError(
            "Emergency stop deployment is incomplete: confirm unit mapping and all four timings"
        )
    filename = "peripherals/emergency_stop.yaml"
    files[filename] = yaml.safe_dump(
        {"emergency_stop": {"ros__parameters": source}}, sort_keys=False
    )
    return [
        {
            "name": "emergency_stop",
            "package": "periph_emergency_stop",
            "executable": "emergency_stop_node",
            "config_file": filename,
        }
    ]
