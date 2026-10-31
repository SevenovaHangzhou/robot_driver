"""Read-only CiA402 topology checks shared by the arm and cyclic-drive families.

Returns one finding per problem (integration maps problems to DREE codes). A bus-level
problem ends the scan because later reads would be meaningless.
"""

import re
import subprocess


def _finding(address, problem, detail, **extra):
    return {"address": address, "problem": problem, "detail": detail, "blocking": True, **extra}


def preflight(expected_topology, *, run=None):
    """Validate declared topology; only full preflight reads drive objects via SDO."""
    topology = {
        **expected_topology,
        "responders": sorted(
            expected_topology["responders"], key=lambda item: item["ring_position"]
        ),
    }
    if not topology["preflight_required"]:
        return []
    execute = subprocess.check_output if run is None else run
    master_id = str(topology["master_id"])
    bus = {"master_id": topology["master_id"]}

    def output(*arguments):
        return execute(["ethercat", *arguments], text=True, timeout=10)

    master = output("master", "-m", master_id)
    mac = topology.get("ethercat_mac")
    if mac and f"{mac} (attached)" not in master:
        return [_finding(bus, "offline", f"EtherCAT master {master_id} is not attached to {mac}")]
    if not re.search(r"Active:\s+no", master) or not re.search(
        rf"Slaves:\s+{len(topology['responders'])}\b", master
    ):
        return [
            _finding(
                bus,
                "offline",
                "Expected inactive master with the declared responder count "
                f"{len(topology['responders'])}",
            )
        ]
    slaves = output("slaves", "-m", master_id, "-v")
    blocks = re.findall(
        rf"=== Master {master_id}, Slave (\d+) ===(.*?)(?==== Master|\Z)", slaves, re.S
    )
    if [int(position) for position, _ in blocks] != [
        item["ring_position"] for item in topology["responders"]
    ]:
        return [_finding(bus, "topology_mismatch", "Unexpected EtherCAT enumeration")]
    findings = []
    for (_, block), identity in zip(blocks, topology["responders"]):
        address = {**bus, "ring_position": identity["ring_position"]}
        for label, key in (
            ("Vendor Id", "vendor_id"),
            ("Product code", "product_code"),
            ("Revision number", "revision_number"),
        ):
            match = re.search(label + r":\s+(0x[0-9a-fA-F]+)", block)
            if not match or int(match.group(1), 16) != identity[key]:
                findings.append(
                    _finding(
                        address,
                        "identity_mismatch",
                        f"Unexpected slave identity: {label} "
                        f"{match.group(1) if match else 'missing'}, expected 0x{identity[key]:x}",
                    )
                )
                break
        if "State: PREOP" not in block:
            findings.append(_finding(address, "offline", "Unexpected slave AL state (not PREOP)"))
    main = blocks[0][1]
    for branch in topology["branches"] + topology["internal_ports"]:
        match = re.search(
            rf"^\s+{branch['main_esc_port']}\s+(?:MII|EBUS)\s+up\s+open\s+yes\s+(\d+)\b", main, re.M
        )
        if not match or int(match.group(1)) != branch["downstream"]:
            findings.append(
                _finding(
                    bus,
                    "topology_mismatch",
                    f"Branch wiring differs from the declared layout at port {branch['main_esc_port']}",
                )
            )
    if findings or topology.get("topology_only", False):
        return findings
    for axis in topology["axes"]:
        address = {**bus, "ring_position": axis["ring_position"]}

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
        if status & 0x6F == 0x27 or value(0x6040) != 0:
            findings.append(_finding(address, "already_controlled", "Axis is already controlled"))
            continue
        # A Fault is accepted only with a declared startup code (e.g. 0xA000 after the master
        # left OP); enable_manager must clear it with a CiA402 fault reset before enabling.
        if error in axis["allowed_errors"]:
            findings.append(
                {
                    **_finding(address, "fault_code", f"Declared startup code 0x{error:04x}"),
                    "blocking": False,
                    "fault_code": error,
                }
            )
        elif status & 8:
            findings.append(
                _finding(
                    address,
                    "fault_code",
                    f"Axis is faulted with unqualified error 0x{error:04x}",
                    fault_code=error,
                )
            )
        elif error != 0:
            findings.append(
                _finding(address, "fault_code", f"Unqualified error 0x{error:04x}", fault_code=error)
            )

    def upload(ring, index, subindex, kind):
        return int(
            output(
                "upload", "-m", master_id, "-p", str(ring), "-t", kind, hex(index), str(subindex)
            ).split()[-1],
            0,
        )

    # The cyclic Kinco plugin uses the drive's saved PDOs: they must equal the profile.
    for check in topology.get("pdo_checks", []):
        ring = check["ring_position"]
        for index, expected in check["objects"].items():
            index = int(index)
            kind = "uint16" if index in (0x1C12, 0x1C13) else "uint32"
            if upload(ring, index, 0, "uint8") != len(expected) or [
                upload(ring, index, sub, kind) for sub in range(1, len(expected) + 1)
            ] != list(expected):
                findings.append(
                    _finding(
                        {**bus, "ring_position": ring},
                        "configuration_mismatch",
                        f"Slave {ring} saved PDO 0x{index:04x} differs from the profile",
                    )
                )
    for window in topology.get("position_windows", []):
        actual = upload(window["ring_position"], 0x6064, 0, "int32")
        if not window["raw_min"] <= actual <= window["raw_max"]:
            findings.append(
                _finding(
                    {**bus, "ring_position": window["ring_position"]},
                    "position_window",
                    f"Slave {window['ring_position']} starts outside its position range",
                )
            )
    return findings
