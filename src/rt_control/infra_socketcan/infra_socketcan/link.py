"""SocketCAN interface admission for CAN owners' startup preflights (hw_canopen, hw_can_damiao)."""

import json
import subprocess


def link_problem(interface, run=None):
    """None when the interface is up and error-active, else (problem, detail) for a finding."""
    execute = subprocess.check_output if run is None else run
    link = json.loads(
        execute(["ip", "-details", "-json", "link", "show", interface], text=True, timeout=5)
    )[0]
    if "UP" not in link.get("flags", []):
        return "offline", f"CAN interface {interface} is not up"
    state = link.get("linkinfo", {}).get("info_data", {}).get("state")
    if state != "ERROR-ACTIVE":
        return "offline", f"CAN interface {interface} is not error-active: {state}"
    return None
