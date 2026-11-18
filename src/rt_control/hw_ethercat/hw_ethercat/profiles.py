"""EtherCAT owner profile readers; values live in installed protocol profiles."""

import yaml

MASTER_PARAMETERS = ("startup_bus_timeout_ms", "preload_timeout_ms")


def load_profile(share, name):
    return yaml.safe_load((share / "config/profiles" / (name + ".yaml")).read_text())


def master_parameters(share):
    """Master startup policy shared by every family on an EtherCAT hardware system."""
    profile = load_profile(share, "ethercat_master")
    values = {key: profile[key] for key in MASTER_PARAMETERS}
    for key, value in values.items():
        if type(value) is not int or value <= 0:
            raise ValueError(f"EtherCAT master profile {key} must be a positive integer")
    return values


_BITS = {"uint8": 8, "int8": 8, "uint16": 16, "int16": 16, "uint32": 32, "int32": 32}


def pdo_objects(driver_config):
    """Expected PDO assignment and mapping objects: the plugin uses the drive's saved PDOs.

    Keys are hex strings: the artifact is YAML and its loader accepts string keys only.
    """
    objects = {}
    for assign, direction in ((0x1C12, "rpdo"), (0x1C13, "tpdo")):
        pdos = driver_config[direction]
        objects[f"0x{assign:04X}"] = [pdo["index"] for pdo in pdos]
        for pdo in pdos:
            objects[f"0x{pdo['index']:04X}"] = [
                (channel["index"] << 16) | (channel["sub_index"] << 8) | _BITS[channel["type"]]
                for channel in pdo["channels"]
            ]
    return objects
