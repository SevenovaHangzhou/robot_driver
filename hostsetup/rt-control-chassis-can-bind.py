#!/usr/bin/env python3
"""Restore this chassis unit's recorded USB-CAN names before runtime startup.

Explicit startup only: do not install as a boot service. Never change calibration
or accept an unknown adapter in a reserved name. Displaced buses remain DOWN;
their respective startup owners configure bitrate and bring them UP.
"""

import argparse
import os
from pathlib import Path
import subprocess


# Alfa_V3-Advantech-01 deployment.yaml and hostsetup/rt-control-can-names.sh.
EXPECTED = {"can0": "003000265230500720333159", "can1": "004D00675230500720333159"}


def rename_plan(interfaces):
    """Validate the complete plan before touching any interface."""
    by_serial = {}
    for name, serial in interfaces.items():
        if serial in EXPECTED.values():
            if serial in by_serial:
                raise RuntimeError("duplicate USB-CAN serial: " + serial)
            by_serial[serial] = name
    if EXPECTED["can0"] not in by_serial:
        raise RuntimeError("chassis USB-CAN adapter is missing")
    for name, serial in EXPECTED.items():
        if name in interfaces and interfaces[name] not in EXPECTED.values():
            raise RuntimeError("reserved CAN name belongs to an unknown adapter: " + name)
    moves = [
        (by_serial[serial], name)
        for name, serial in EXPECTED.items()
        if serial in by_serial and by_serial[serial] != name
    ]
    for index in range(len(moves)):
        if "rtcan_tmp" + str(index) in interfaces:
            raise RuntimeError("temporary CAN name is already occupied")
    return moves


def discover(net_root=Path("/sys/class/net")):
    interfaces = {}
    for net in net_root.iterdir():
        if (net / "type").read_text().strip() != "280":
            continue
        device = (net / "device").resolve()
        serials = [
            (parent / "serial").read_text().strip()
            for parent in (device, *device.parents)
            if (parent / "serial").is_file()
        ]
        interfaces[net.name] = serials[0] if serials else ""
    return interfaces


def apply_plan(moves, run=subprocess.run):
    if not moves:
        return
    # Runtime owns buses once started. Refuse to rename beneath an active manager.
    probe = run(["pgrep", "-x", "ros2_control_no"], check=False, stdout=subprocess.DEVNULL)
    if probe.returncode != 1:
        raise RuntimeError("cannot rebind CAN while controller_manager runs or process check fails")
    # All old names move aside first, so a can0/can1 swap cannot collide.
    for index, (old, _) in enumerate(moves):
        run(["ip", "link", "set", "dev", old, "down"], check=True)
        run(["ip", "link", "set", "dev", old, "name", "rtcan_tmp" + str(index)], check=True)
    for index, (_, new) in enumerate(moves):
        run(["ip", "link", "set", "dev", "rtcan_tmp" + str(index), "name", new], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="read-only verification")
    args = parser.parse_args()
    interfaces = discover()
    moves = rename_plan(interfaces)
    if args.check and moves:
        raise RuntimeError("USB-CAN names require rebinding: " + str(moves))
    if not args.check:
        if os.geteuid() != 0:
            raise RuntimeError("binding requires root")
        apply_plan(moves)
        if rename_plan(discover()):
            raise RuntimeError("USB-CAN name verification failed after rebinding")
    print(
        "CHASSIS_CAN_BIND_OK can0="
        + EXPECTED["can0"]
        + (" (names corrected; displaced buses left DOWN)" if moves else " (no changes)")
    )


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error))
