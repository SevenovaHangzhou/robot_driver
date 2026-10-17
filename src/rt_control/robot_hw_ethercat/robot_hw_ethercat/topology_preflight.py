"""Read-only CiA402 topology checks shared by the arm and cyclic-drive families."""

import re
import subprocess


def preflight(expected_topology, *, run=None):
    """Reproduce the old stationary checks. Motion has no new admission preflight."""
    topology = {
        **expected_topology,
        "responders": sorted(
            expected_topology["responders"], key=lambda item: item["ring_position"]
        ),
    }
    if not topology["preflight_required"]:
        return
    execute = subprocess.check_output if run is None else run
    master_id = str(topology["master_id"])

    def output(*arguments):
        return execute(["ethercat", *arguments], text=True, timeout=10)

    master = output("master", "-m", master_id)
    if not re.search(r"Active:\s+no", master) or not re.search(
        rf"Slaves:\s+{len(topology['responders'])}\b", master
    ):
        raise RuntimeError("Expected inactive master with the declared responder count")
    slaves = output("slaves", "-m", master_id, "-v")
    blocks = re.findall(
        rf"=== Master {master_id}, Slave (\d+) ===(.*?)(?==== Master|\Z)", slaves, re.S
    )
    if [int(position) for position, _ in blocks] != [
        item["ring_position"] for item in topology["responders"]
    ]:
        raise RuntimeError("Unexpected EtherCAT enumeration")
    for (_, block), identity in zip(blocks, topology["responders"]):
        for label, key in (
            ("Vendor Id", "vendor_id"),
            ("Product code", "product_code"),
            ("Revision number", "revision_number"),
        ):
            match = re.search(label + r":\s+(0x[0-9a-fA-F]+)", block)
            if not match or int(match.group(1), 16) != identity[key]:
                raise RuntimeError("Unexpected slave identity")
        if "State: PREOP" not in block:
            raise RuntimeError("Unexpected slave AL state")
    main = blocks[0][1]
    for branch in topology["branches"] + topology["internal_ports"]:
        match = re.search(
            rf"^\s+{branch['main_esc_port']}\s+(?:MII|EBUS)\s+up\s+open\s+yes\s+(\d+)\b", main, re.M
        )
        if not match or int(match.group(1)) != branch["downstream"]:
            raise RuntimeError("Branch wiring differs from the declared layout")
    for axis in topology["axes"]:

        def value(index):
            return int(
                output(
                    "upload",
                    "-m",
                    master_id,
                    "-p",
                    str(axis["ring_position"]),
                    "-t",
                    "uint16",
                    hex(index),
                    "0",
                ).split()[-1]
            )

        status, error = value(0x6041), value(0x603F)
        if status & 8 or status & 0x6F == 0x27 or value(0x6040) != 0:
            raise RuntimeError("Axis is faulted or already controlled")
        if error != 0 and error not in axis["allowed_errors"]:
            raise RuntimeError(f"Unqualified error 0x{error:04x}")
