"""Startup self-check stage A: owner preflight findings -> located DREE entries.

Owners report facts (address, problem, vendor fault code); this module is the one place that
turns them into ErrorInfo-shaped entries using the artifact's diagnostics.yaml (device
sources and vendor fault tables). DREE names are robot_interfaces ErrorCode constants.
"""

import yaml

# Vendor fault codes are looked up in the device's fault table; this is the default.
PROBLEM_DREE = {
    "offline": "RT_DEVICE_OFFLINE",
    "identity_mismatch": "RT_DEVICE_IDENTITY_MISMATCH",
    "topology_mismatch": "RT_DEVICE_IDENTITY_MISMATCH",
    "configuration_mismatch": "RT_DEVICE_IDENTITY_MISMATCH",
    "already_controlled": "RT_DEVICE_FAULT",
    "position_window": "RT_DEVICE_FAULT",
    "fault_code": "RT_DEVICE_FAULT",
}


class SelfCheckFailed(RuntimeError):
    """A blocking self-check entry; the runtime must not start. Carries every entry."""

    def __init__(self, entries):
        self.entries = entries
        super().__init__(
            "Self-check failed: "
            + "; ".join(f"{e['source']}: {e['detail']}" for e in entries if e["blocking"])
        )


def _key(address):
    return tuple(sorted(address.items()))


def _fallback_source(address):
    if "ring_position" in address:
        return f"ethercat/master{address['master_id']}/slave{address['ring_position']}"
    if "master_id" in address:
        return f"ethercat/master{address['master_id']}"
    return f"can/{address['can_interface']}"


def entries(diagnostics_text, findings):
    diagnostics = yaml.safe_load(diagnostics_text) if diagnostics_text else {}
    devices = {_key(device["address"]): device for device in diagnostics.get("devices", [])}
    tables = diagnostics.get("fault_tables", {})
    result = []
    for finding in findings:
        device = devices.get(_key(finding["address"]), {})
        dree, detail = PROBLEM_DREE[finding["problem"]], finding["detail"]
        if "fault_code" in finding:
            code = f"0x{finding['fault_code']:04X}"
            table = tables.get(device.get("fault_table"), {})
            fault = next((f for f in table.get("faults", []) if f.get("code") == code), None)
            if fault is None:
                detail = f"{detail}; 未登记故障码 {code}"
            else:
                dree = fault["dree"]
                detail = f"{detail}; {code} {fault['message']}（{table['vendor']} 手册 p{fault['page']}）"
        result.append(
            {
                "source": device.get("source", _fallback_source(finding["address"])),
                "owner": finding["owner"],
                "dree": dree,
                "severity": "FAULT" if finding["blocking"] else "WARN",
                "blocking": finding["blocking"],
                "detail": detail,
            }
        )
    return result
