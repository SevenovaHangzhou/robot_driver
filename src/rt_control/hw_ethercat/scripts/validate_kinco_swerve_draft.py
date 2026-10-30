#!/usr/bin/env python3
"""Validate the protocol-owned Kinco cyclic profile without machine facts."""

import argparse
from pathlib import Path

import yaml


def validate(path):
    data = yaml.safe_load(path.read_text())
    expected = {
        "rpdo": [(0x6040, "uint16"), (0x607A, "int32"), (0x60FF, "int32"), (0x6060, "int8")],
        "tpdo": [(0x6041, "uint16"), (0x6064, "int32"), (0x606C, "int32"), (0x6061, "int8")],
    }
    profile = data["driver_config"]
    for name, entries in expected.items():
        channels = profile[name][0]["channels"]
        if [(entry["index"], entry["type"]) for entry in channels] != entries:
            raise ValueError("Kinco protocol PDO shape differs from the qualified adapter")
        if any(entry["sub_index"] != 0 for entry in channels):
            raise ValueError("Kinco cyclic PDO sub-index must remain zero")
    if data["velocity_dec"] != {"numerator": 512, "denominator": 1875, "base_unit": "rpm"}:
        raise ValueError("Kinco DEC conversion differs from the documented protocol")
    if not data["mock_adapter"]["mock_only"]:
        raise ValueError("Synthetic adapter settings must remain mock-only")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    validate(args.profile)
    print("Validated owner Kinco cyclic protocol profile")


if __name__ == "__main__":
    main()
