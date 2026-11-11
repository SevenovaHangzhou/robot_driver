"""Bounded startup discovery through the protocol owner; no background CAN listener."""

from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_prefix


def probe_module(machine, declaration, deployment):
    spec = machine["spec"]
    interface = deployment["can_interfaces"][declaration["can_interface_key"]]
    if interface == "TBD" or "/" in interface or not interface:
        raise ValueError("Head CAN interface is not configured")
    flags = Path("/sys/class/net") / interface / "flags"
    joints = spec["joints"]
    if not flags.exists() or not (int(flags.read_text().strip(), 16) & 1):
        return {
            "present": False,
            "missing_joints": [j["robot_model_joint"] for j in joints],
            "reason": "CAN interface absent or down",
        }
    executable = (
        Path(get_package_prefix("hw_can_damiao")) / "lib/hw_can_damiao/damiao_presence_probe"
    )
    missing = []
    for joint in joints:
        result = subprocess.run(
            [
                str(executable),
                interface,
                str(joint["can_id"]),
                str(joint["master_id"]),
                str(spec["configure_timeout_ms"]),
            ],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        if result.returncode == 3:
            missing.append(joint["robot_model_joint"])
        elif result.returncode != 0:
            raise ValueError(result.stderr.strip() or "Head discovery failed")
    return {
        "present": not missing,
        "missing_joints": missing,
        "reason": "missing response" if missing else "all configured joints responded",
    }
