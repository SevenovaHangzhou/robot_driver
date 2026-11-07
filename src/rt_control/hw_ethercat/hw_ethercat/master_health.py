"""One set of master health interfaces per rendered hardware group."""

import xml.etree.ElementTree as ET

from infra_renderer import xml_param


def responder_count(request):
    """Slaves on the master; only the first peer module renders the master health group."""
    peers = request.get("peer_machines", [request["machine"]])
    if peers[0]["module"] != request["machine"]["module"]:
        return None
    count = 0
    for peer in peers:
        spec = peer["spec"]
        count += (
            len(spec["axes"])
            if "axes" in spec
            else sum(len(branch["axes"]) for branch in spec["branches"].values())
        )
        count += len(spec.get("extra_responders", []))
    return count


def offline_exports(request, axes):
    """enable_manager offline sources: the domain age (first peer module) and each slave's AL state."""
    exports = {}
    if responder_count(request) is not None:
        exports["feedback_age_interfaces"] = ["ethercat_domain/process_data_age_ms"]
    al_states = [
        f"ethercat_slave_{axis['ring_position']}/al_state"
        for axis in axes
        if type(axis["ring_position"]) is int
    ]
    if al_states:
        exports["al_state_interfaces"] = al_states
    return exports


def sensors(request, axes):
    health = {}
    count = responder_count(request)
    if count is not None:
        health = {
            "ethercat_domain": {"process_data_age_ms": 0},
            "ethercat_master": {"link_up": 1, "slaves_responding": count, "wc_error_count": 0},
        }
    health.update(
        {
            f"ethercat_slave_{axis['ring_position']}": {"al_state": 8}
            for axis in axes
            if type(axis["ring_position"]) is int
        }
    )
    result = []
    for name, fields in health.items():
        sensor = ET.Element("sensor", name=name)
        for field, value in fields.items():
            interface = ET.SubElement(sensor, "state_interface", name=field)
            if request["use_mock_hardware"]:
                xml_param(interface, "initial_value", value)
        result.append(sensor)
    return result


def master_entry(master_id, slaves):
    """Diagnostics bus entry over the master health group's state interfaces."""
    return {
        "name": f"master{master_id}",
        "role": "ethercat_master",
        "bus": True,
        "address": {"master_id": master_id},
        "state": {
            "link_up": "ethercat_master/link_up",
            "slaves_responding": "ethercat_master/slaves_responding",
            "wc_error_count": "ethercat_master/wc_error_count",
            "process_data_age_ms": "ethercat_domain/process_data_age_ms",
        },
        "expected": {"slaves_responding": slaves},
    }


def master_diagnostic(request, master_id):
    """Bus entry for the diagnostics node; only the module that renders the health group has it."""
    slaves = responder_count(request)
    if slaves is None:
        return []
    return [master_entry(master_id, slaves)]
