"""EtherCAT owner API 1: family-registry dispatch and explicit read-only preflight.

`config/profiles/renderer_families.yaml` maps a module spec `family` to a renderer
module and its private schemas. This file holds no family names.
"""

from collections.abc import Mapping
from copy import deepcopy
from functools import partial
import importlib
import math
from pathlib import Path
import subprocess  # noqa: F401  # Preflight implementations resolve it through this owner.

from ament_index_python.packages import get_package_share_directory
from infra_renderer import validated_spec

from .arm_renderer import position_factors  # noqa: F401  # Stable owner-level helper.
from .profiles import load_profile

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("hw_ethercat"))


def _registry():
    return load_profile(_share(), "renderer_families")


def _family(name):
    families = _registry()["families"]
    if name not in families:
        raise ValueError(f"Unregistered EtherCAT renderer family: {name}")
    return families[name]


def _module(name):
    return importlib.import_module(f"{__package__}.{name}")


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    if "family" not in data["spec"]:
        raise ValueError("EtherCAT module spec must declare its renderer family")
    schema = _family(data["spec"]["family"])["module_schema"]
    return validated_spec(data, _share() / "schema" / schema)


def validate_unit(module: Mapping, data: Mapping | None, *, family=None) -> Mapping:
    """The owner binds `family` from the module spec; units carry no family of their own."""
    if family is None:
        raise ValueError("EtherCAT unit validation requires the module renderer family")
    schema = _family(family).get("unit_schema")
    if schema is None:
        return {} if data is None else deepcopy(dict(data))
    if data is None:
        raise ValueError(f"Calibration is required for EtherCAT family {family}")
    return validated_spec(data, _share() / "schema" / schema)


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    """The device family declares which position count fact the real path requires."""
    families = load_profile(_share(), "families")["families"]
    field = families.get(data["family"], {}).get("position_counts_field")
    if field is None:
        raise ValueError(f"Device family {data['family']} declares no position count field")
    value = data.get(field)
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"Required {field} must be positive and finite")
    return deepcopy(dict(data))


def render(request: Mapping) -> Mapping:
    """Render content only. The composer registry owns the shared envelope."""
    family = request["machine"]["spec"].get("family")
    if family is None:
        raise ValueError("EtherCAT module spec must declare its renderer family")
    module = _module(_family(family)["module"])
    return module.render(
        request,
        _share(),
        validate_module,
        partial(validate_unit, family=family),
        validate_device,
    )


def preflight(expected_topology: Mapping, *, run=None) -> None:
    kinds = _registry()["preflight"]
    kind = expected_topology.get("kind")
    if kind not in kinds:
        raise ValueError(f"Unregistered EtherCAT preflight kind: {kind}")
    return _module(kinds[kind]).preflight(expected_topology, run=run)
