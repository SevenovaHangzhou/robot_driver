"""Offline helpers shared by the protocol-owned rt_control renderers."""

from collections.abc import Mapping
from copy import deepcopy
import hashlib
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


def calibration_id(owner: str, *facts) -> str:
    """Identity `<owner>:sha256:<16 hex>` of the facts a renderer used; any change changes it.

    Facts must be JSON data (the validated YAML documents); key order does not matter, and the
    provenance keys `authorities` and `pending_facts` of a top-level document are ignored.
    """
    provenance = {"authorities", "pending_facts"}
    facts = [
        {k: v for k, v in fact.items() if k not in provenance} if isinstance(fact, Mapping) else fact
        for fact in facts
    ]
    text = json.dumps(facts, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return f"{owner}:sha256:{hashlib.sha256(text.encode()).hexdigest()[:16]}"
