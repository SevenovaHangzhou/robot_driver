"""Select the machine's explicit additive-gravity view without changing the default view."""

import math


def select_gravity(data, enabled):
    if type(enabled) is not bool:
        raise ValueError("gravity_feedforward must be a boolean")
    if not enabled:
        return
    scope = data["scope"]
    selected = scope.get("gravity_feedforward")
    if not selected or scope["render_mode"] != "normal":
        raise ValueError("The selected scope has no gravity_feedforward view")
    module = selected["module"]
    if module not in scope["enabled_modules"]:
        raise ValueError("Gravity module is not enabled in this scope")
    bindings = {item["name"]: item for item in data["controllers"].values()}
    for name in selected["controllers"]:
        binding = bindings.get(name, {})
        if binding.get("module") != module or "enable_companion" not in binding.get("tags", []):
            raise ValueError(f"Gravity controller {name} must be a companion of {module}")
        if name in scope["controller_order"]["inactive"] + scope["controller_order"]["active"]:
            raise ValueError(f"Gravity controller {name} is already selected")
        scope["controller_order"]["inactive"].append(name)
    scope["policies"].append(selected["policy"])
    data["gravity_module"] = module


def validate_gravity_parameters(data, controllers):
    if "gravity_module" not in data:
        return
    for name in data["scope"]["gravity_feedforward"]["controllers"]:
        params = controllers[name]["ros__parameters"]
        scale = params.get("scale")
        if type(scale) not in (int, float) or not math.isfinite(scale) or not 0 <= scale <= 1:
            raise ValueError(f"{name}: explicit commissioning scale in [0, 1] required")
        for key in ("max_effort_nm", "max_effort_rate_nm_s"):
            values = params.get(key, [])
            if len(values) != len(params["joints"]) or any(
                type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values
            ):
                raise ValueError(f"{name}: explicit positive per-joint {key} required")
        if len(params["health_interfaces"]) != len(params["health_values"]):
            raise ValueError(f"{name}: inconsistent gravity health bindings")
