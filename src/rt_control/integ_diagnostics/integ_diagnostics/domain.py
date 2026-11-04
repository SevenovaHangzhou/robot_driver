"""R-OUT-09 readiness from the controllers' own state; pure functions, no ROS.

The real-time controllers detect and handle faults. This module only answers "may RT-Control
accept commands now, and if not, why": self-check result, enable_manager state and latched
failure, motion controllers and hardware components active.
"""

from integ_composer.self_check import describe_fault

# Self-check DREE names that block with a dedicated token; other blocking entries are device faults.
SELF_CHECK_TOKENS = {
    "RT_DEVICE_OFFLINE": "device_offline",
    "RT_DEVICE_IDENTITY_MISMATCH": "identity_mismatch",
    "RT_STO_ACTIVE": "sto_active",
}
# enable_manager failure stage -> (blocker token, DREE name).
FAILURE = {
    "device_offline": ("device_offline", "RT_DEVICE_OFFLINE"),
    "restart_required": ("restart_required", "RT_RESTART_REQUIRED"),
    "fault_detected": ("device_fault", "RT_DEVICE_FAULT"),
    "fault_requires_reset": ("device_fault", "RT_DEVICE_FAULT"),
    "fault_reset_timeout": ("device_fault", "RT_DEVICE_FAULT"),
    "unexpected_drive_state": ("device_fault", "RT_DEVICE_FAULT"),
}
DEFAULT_FAILURE = ("enable_failed", "RT_ENABLE_FAILED")
# Every DREE name this module can publish, for the pinned-contract check.
DREE_NAMES = {dree for _, dree in FAILURE.values()} | {
    DEFAULT_FAILURE[1],
    "RT_ENABLE_MANAGER_NOT_READY",
}


def failure_entry(enable, devices, tables, values):
    """The latched enable_manager failure as one FAULT entry, with the vendor code when known."""
    token, dree = FAILURE.get(enable["stage"], DEFAULT_FAILURE)
    device = _failed_device(enable, devices)
    detail = f"enable_manager {enable['stage']}"
    if device is not None:
        code_name = device["state"].get("error_code") or device["state"].get("fault_code")
        code = int(values.get(code_name, 0)) if code_name else 0
        if code:
            listed, text = describe_fault(device, tables, code)
            dree = listed if token == "device_fault" and listed else dree
            detail += f"; {text}"
    if enable.get("offline_feedback"):
        detail += f"; stale {enable['offline_feedback']}"
    source = device["source"] if device is not None else "rt_control/enable_manager"
    return token, {
        "source": source,
        "dree": dree,
        "severity": "FAULT",
        "blocking": True,
        "detail": detail,
    }


def _failed_device(enable, devices):
    joint = enable.get("failed_joint", "")
    stale = enable.get("offline_feedback", "")
    for device in devices:
        names = device["state"].values()
        if (joint and any(name.startswith(joint + "/") for name in names)) or stale in names:
            return device
    return None


def domain_readiness(self_check, enable, controllers, hardware, failure=None):
    """DomainReadiness fields.

    self_check: {"pending": bool, "entries": [...]}; enable: enable_manager diagnostic values or
    None when stale; controllers/hardware: name -> state, or None while not yet queried;
    failure: (token, entry) from failure_entry() when enable_manager is FAILED.
    """
    blockers, errors = [], []
    if self_check["pending"]:
        blockers.append("self_check_pending")
    for entry in self_check["entries"]:
        if entry["blocking"]:
            blockers.append(SELF_CHECK_TOKENS.get(entry["dree"], "device_fault"))
    state = None if enable is None else enable["state"]
    if state is None:
        blockers.append("enable_manager_unavailable")
    elif state == "FAILED":
        token, entry = failure
        blockers.append(token)
        errors.append(entry)
    elif state != "ENABLED":
        blockers.append("not_enabled")
    if hardware is not None and any(value != "active" for value in hardware.values()):
        blockers.append("hardware_inactive")
    if (
        state == "ENABLED"
        and controllers is not None
        and any(value != "active" for value in controllers.values())
    ):
        blockers.append("controller_inactive")
    # Self-check warnings (e.g. a declared startup code) matter until enable clears them.
    if state != "ENABLED":
        errors += [entry for entry in self_check["entries"] if not entry["blocking"]]
    errors += [entry for entry in self_check["entries"] if entry["blocking"]]
    blockers = sorted(set(blockers))
    ready = not blockers
    return {
        "ready": ready,
        "status": "UNAVAILABLE" if not ready else "DEGRADED" if errors else "HEALTHY",
        "operational_state": "self_check_pending"
        if self_check["pending"]
        else (state or "unknown").lower(),
        "blockers": blockers,
        "errors": errors,
    }
