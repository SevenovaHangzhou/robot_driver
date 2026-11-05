"""Self-check device rules over one state sample (startup stage B and on demand); no ROS.

Devices come from the artifact's diagnostics.yaml; each names the state interfaces it reads.
Entries have the same shape as integ_composer.self_check entries.
"""

import math

from integ_composer.self_check import ERROR_WORDS, describe_error_bits, describe_fault

AL_STATE_OP = 8  # EtherCAT AL state code for Operational.
CIA402_FAULT_BIT = 0x0008  # Status word bit 3: Fault or Fault reaction active.


def complete(devices, values):
    """True once every declared state interface has a finite value."""
    return all(
        name in values and math.isfinite(values[name])
        for device in devices
        for name in device["state"].values()
    )


def evaluate(devices, tables, values):
    entries = []

    def add(device, dree, blocking, detail):
        entries.append(
            {
                "source": device["source"],
                "dree": dree,
                "severity": "FAULT" if blocking else "WARN",
                "blocking": blocking,
                "detail": detail,
            }
        )

    for device in devices:
        state = {role: values[name] for role, name in device["state"].items()}
        if device["role"] == "ethercat_master":
            expected = device.get("expected", {}).get("slaves_responding")
            if state["link_up"] != 1:
                add(device, "RT_DEVICE_OFFLINE", True, "EtherCAT link down")
            elif expected is not None and int(state["slaves_responding"]) != expected:
                add(
                    device,
                    "RT_DEVICE_OFFLINE",
                    True,
                    f"{int(state['slaves_responding'])} of {expected} slaves responding",
                )
        elif device["role"] == "drive":
            if "al_state" in state and int(state["al_state"]) != AL_STATE_OP:
                add(
                    device,
                    "RT_DEVICE_OFFLINE",
                    True,
                    f"AL state {int(state['al_state'])}, expected OP ({AL_STATE_OP})",
                )
                continue
            words = {role: state[role] for role in ERROR_WORDS if role in state}
            faults = describe_error_bits(device, tables, words)
            if faults:
                for dree, text in faults:
                    add(device, dree or "RT_DEVICE_FAULT", True, text)
                continue
            code = int(state.get("error_code", 0))
            if code:
                dree, text = describe_fault(device, tables, code)
                # Declared startup codes (e.g. 0xA000) are cleared by enable_manager's reset.
                allowed = f"0x{code:04X}" in device.get("allowed_startup_codes", [])
                add(device, dree or "RT_DEVICE_FAULT", not allowed, f"0x603F: {text}")
            elif "status_word" in state and int(state["status_word"]) & CIA402_FAULT_BIT:
                add(device, "RT_DEVICE_FAULT", True, "CiA402 fault bit set, no error code read")
        elif device["role"] == "encoder":
            if int(state["ready"]) != 1:
                add(
                    device,
                    "RT_DEVICE_FEEDBACK_STALE",
                    True,
                    f"encoder not ready (feedback age {state['feedback_age_ms']:.0f} ms)",
                )
        elif device["role"] == "head_motor":
            code = int(state["fault_code"])
            limit = device["expected"]["max_feedback_age_ms"]
            if code:
                dree, text = describe_fault(device, tables, code)
                add(device, dree or "RT_DEVICE_FAULT", True, f"feedback ERR: {text}")
            elif not 0.0 <= state["feedback_age_ms"] <= limit:
                add(
                    device,
                    "RT_DEVICE_FEEDBACK_STALE",
                    True,
                    f"feedback age {state['feedback_age_ms']:.0f} ms > {limit} ms",
                )
    return entries
