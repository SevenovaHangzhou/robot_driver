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
