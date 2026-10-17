"""Compatibility changes are limited to measured width and tool classification."""

import configparser
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "config/eds"
ORIGINAL = "BRT-Multi-turn-V1.0.eds"
COMPAT = "BRT-Multi-turn-V1.0.compat.eds"
ORIGINAL_SHA256 = "8ed45c23f567868c79903c77dde65ffa1bdb20d5078418d07a32065ea7fb6a53"


def read(name):
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(ROOT / name)
    return parser


def inventory(parser):
    values = []
    for group in ("MandatoryObjects", "OptionalObjects", "ManufacturerObjects"):
        if group not in parser:
            continue
        section = parser[group]
        count = int(section["supportedobjects"])
        assert len(section) == count + 1
        values.extend(int(section[str(index)], 0) for index in range(1, count + 1))
    assert len(values) == len(set(values))
    return set(values)


def test_vendor_original_is_unchanged():
    assert hashlib.sha256((ROOT / ORIGINAL).read_bytes()).hexdigest() == ORIGINAL_SHA256


def test_only_readback_width_changes_in_object_definitions():
    original, compat = read(ORIGINAL), read(COMPAT)
    assert set(original.sections()) == set(compat.sections())
    for section in original.sections():
        if section in ("FileInfo", "MandatoryObjects", "OptionalObjects"):
            continue
        expected = dict(original[section])
        if section == "6502":
            expected["datatype"] = "0x0007"
        assert dict(compat[section]) == expected, section


def test_object_inventory_preserved_without_duplicates():
    original, compat = read(ORIGINAL), read(COMPAT)
    assert inventory(original) == inventory(compat)
    assert int(compat["MandatoryObjects"]["supportedobjects"]) == 3
