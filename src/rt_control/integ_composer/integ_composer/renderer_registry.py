"""Shared envelope checks and XML projection before integration consumes a renderer."""

from collections.abc import Mapping
import importlib
import xml.etree.ElementTree as ET

from .schema.primitives import validate_data


def select_renderer(owner_refs, transport):
    owner = owner_refs[transport]
    renderer = importlib.import_module(f"{owner}.rt_control_renderer")
    if renderer.RENDERER_API_VERSION != 1:
        raise ValueError(f"Unsupported renderer API from {owner}")
    return renderer


def project(element):
    if not isinstance(element, ET.Element):
        raise ValueError("Renderer XML fragment must be a standard-library Element")
    if not isinstance(element.tag, str) or element.tag.startswith("{"):
        raise ValueError("Renderer XML namespaces and non-element tags are forbidden")
    if element.tail and element.tail.strip():
        raise ValueError("Non-whitespace XML tail cannot be discarded")
    if element.tag != "param" and element.text and element.text.strip():
        raise ValueError("Only param may contain non-whitespace XML text")
    return {
        "tag": element.tag,
        "attrib": dict(element.attrib),
        "text": (element.text or "") if element.tag == "param" else element.text,
        "children": [project(child) for child in element],
    }


def render_checked(renderer, request, schema_dir, share_root):
    validate_data(request, "render_request.schema.json", schema_dir, share_root)
    result = renderer.render(request)
    if not isinstance(result, Mapping):
        raise ValueError("Renderer result must be a Mapping")
    projection = dict(result)
    for key in ("joints", "sensors", "gpios"):
        if key in projection:
            projection[key] = [project(element) for element in projection[key]]
    validate_data(projection, "render_result.schema.json", schema_dir, share_root)
    return result
