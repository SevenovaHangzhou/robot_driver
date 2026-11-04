import json
import xml.etree.ElementTree as ET

from jsonschema import ValidationError
import pytest

from infra_renderer import validated_spec, xml_param


@pytest.fixture
def schema(tmp_path):
    path = tmp_path / "spec.schema.json"
    path.write_text(json.dumps({"type": "object", "required": ["id"]}))
    return path


def test_validated_spec_returns_an_independent_copy(schema):
    envelope = {"kind": "machine_module", "spec": {"id": 1, "nested": [1]}}
    result = validated_spec(envelope, schema)
    assert result == envelope
    result["spec"]["nested"].append(2)
    assert envelope["spec"]["nested"] == [1]


def test_validated_spec_rejects_a_spec_outside_the_schema(schema):
    with pytest.raises(ValidationError):
        validated_spec({"spec": {}}, schema)


def test_xml_param_writes_the_value_as_text():
    parent = ET.Element("hardware")
    xml_param(parent, "cycle_hz", 1000)
    assert ET.tostring(parent) == b'<hardware><param name="cycle_hz">1000</param></hardware>'
