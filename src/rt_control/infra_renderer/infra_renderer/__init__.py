"""Offline helpers shared by the protocol-owned rt_control renderers."""

from collections.abc import Mapping
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from jsonschema import Draft7Validator


def validated_spec(envelope: Mapping, schema_path: Path) -> dict:
    """A copy of `envelope` whose `spec` passed the owner's Draft 7 schema at `schema_path`."""
    Draft7Validator(json.loads(Path(schema_path).read_text())).validate(envelope["spec"])
    return deepcopy(dict(envelope))


def xml_param(parent: ET.Element, name: str, value) -> None:
    ET.SubElement(parent, "param", name=name).text = str(value)
