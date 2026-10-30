"""Bind declared controller parameters to public renderer exports, never private specs."""

from copy import deepcopy
import math


def bind_parameters(binding, parameters, rendered_modules, *, mock):
    """Keyed exports follow explicit instance order, independent of bus enumeration."""
    result = deepcopy(parameters)
    for name, reference in binding.get("parameters", {}).items():
        module = reference["module"]
        if module not in rendered_modules:
            raise ValueError(f"Unselected controller parameter module: {module}")
        owner = rendered_modules[module]
        exports = owner.get("parameter_exports", {})
        if reference["export"] not in exports:
            raise ValueError(f"Missing controller parameter export: {module}.{reference['export']}")
        value = deepcopy(exports[reference["export"]])
        if isinstance(value, dict):
            order = binding.get("instance_order", [])
            if not order or len(order) != len(set(order)) or set(order) != set(value):
                raise ValueError(f"Controller instance keys do not match export: {name}")
            value = [value[key] for key in order]
        values = value if isinstance(value, list) else [value]
        if not mock and "TBD" in values:
            raise ValueError(f"Required controller parameter is TBD: {name}")
        for item in values:
            if isinstance(item, float) and not math.isfinite(item):
                raise ValueError(f"Non-finite controller parameter: {name}")
        if "resource_kind" in reference:
            resources = {element.attrib["name"] for element in owner[reference["resource_kind"]]}
            if len(values) != len(set(values)) or not set(values).issubset(resources):
                raise ValueError(f"Controller parameter references invalid resources: {name}")
        # Two authorities are an error even when today's values happen to agree.
        if name in result:
            raise ValueError(f"Controller parameter duplicates a hardware fact: {name}")
        result[name] = value
    return result
