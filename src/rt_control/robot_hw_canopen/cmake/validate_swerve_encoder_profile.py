#!/usr/bin/env python3
"""Validate the owner CiA406 protocol profile, independently of machine facts."""

import argparse
from pathlib import Path

import yaml


def validate(path):
    profile = yaml.safe_load(path.read_text())
    if set(profile) != {
        "plugin",
        "node_driver",
        "master_driver",
        "position_pdo",
        "state_interfaces",
    }:
        raise ValueError("Encoder protocol profile has missing or unknown fields")
    if profile["plugin"] != "robot_hw_canopen/SwerveEncoderSystem":
        raise ValueError("Encoder system plugin is incorrect")
    if profile["position_pdo"] != {
        "index": 0x6004,
        "subindex": 0,
        "bit_length": 32,
        "signed": False,
    }:
        raise ValueError("Encoder position PDO must be unsigned 0x6004:00/32")
    if profile["state_interfaces"] != ["position", "feedback_age_ms"]:
        raise ValueError("Encoder states must remain position and feedback_age_ms")
    if profile["node_driver"] != {
        "class": "ros2_canopen::ProxyDriver",
        "package": "canopen_proxy_driver",
    }:
        raise ValueError("Encoder node driver is incorrect")
    if profile["master_driver"] != {
        "class": "ros2_canopen::MasterDriver",
        "package": "canopen_master_driver",
    }:
        raise ValueError("Encoder master driver is incorrect")
    return profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    validate(args.profile)
    print("Validated CiA406 encoder protocol profile")


if __name__ == "__main__":
    main()
