"""One set of master health interfaces per rendered hardware group."""

import xml.etree.ElementTree as ET


def sensors(request, axes):
    peers = request.get("peer_machines", [request["machine"]])
    health = {}
    if peers[0]["module"] == request["machine"]["module"]:
        count = 0
        for peer in peers:
            spec = peer["spec"]
            count += (
                len(spec["axes"])
                if "axes" in spec
                else sum(len(branch["axes"]) for branch in spec["branches"].values())
            )
            count += len(spec.get("extra_responders", []))
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
                ET.SubElement(interface, "param", name="initial_value").text = str(value)
        result.append(sensor)
    return result
