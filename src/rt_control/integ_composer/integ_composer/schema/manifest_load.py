"""Load machine envelopes and select only explicitly referenced scope data."""

from pathlib import Path

from .primitives import load_and_validate, load_yaml, resolve_share_path, validate_data


def read_manifest(path):
    """manifest.yaml merged with the part files it lists under `parts` (not validated)."""
    path = Path(path)
    manifest = load_yaml(path)
    for part in manifest.pop("parts", []):
        piece = load_yaml(resolve_share_path(path.parent, part))
        repeated = set(piece) & set(manifest)
        if repeated:
            raise ValueError(f"Manifest part {part} repeats {sorted(repeated)}")
        manifest.update(piece)
    return manifest


def load_manifest(path, schema_dir, share_root):
    """The merged manifest, validated as one document."""
    manifest = read_manifest(path)
    validate_data(manifest, "manifest.schema.json", schema_dir, share_root)
    return manifest


def load_machine(
    config_share, schema_dir, robot_variant, machine_unit, physical_profile, control_scope
):
    root = Path(config_share)
    machine_dir = resolve_share_path(root, f"machines/{robot_variant}")
    unit_dir = resolve_share_path(root, f"units/{machine_unit}")

    def load(path, schema):
        return load_and_validate(path, schema, schema_dir, root)

    manifest = load_manifest(machine_dir / "manifest.yaml", schema_dir, root)
    deployment = load(unit_dir / "deployment.yaml", "unit_deployment.schema.json")
    if (
        manifest["robot_variant"] != deployment["robot_variant"]
        or manifest["robot_variant"] != robot_variant
    ):
        raise ValueError("Machine/unit robot variant mismatch")
    profile = manifest["profiles"][physical_profile]
    if control_scope not in profile["allowed_scopes"]:
        raise ValueError("Scope is not allowed by the selected physical profile")
    scope = manifest["scopes"][control_scope]
    selected = list(profile["modules"])
    option_modules = []
    for option, selection in deployment["hardware_options"].items():
        if selection not in profile["hardware_options"][option]:
            raise ValueError(f"Hardware option {option}={selection} is not allowed")
        option_modules += manifest["hardware_options"][option][selection]["modules"]
    selected += option_modules
    active = scope["enabled_modules"] + scope["required_state_modules"] + option_modules
    if not set(active).issubset(selected):
        raise ValueError("Scope references modules absent from the physical profile")
    modules = {}
    calibrations = {}
    devices = {}
    for name in dict.fromkeys(active):
        entry = manifest["modules"][name]
        module = load(resolve_share_path(root, entry["config_file"]), "module_envelope.schema.json")
        if module["module"] != name or module["robot_variant"] != robot_variant:
            raise ValueError(f"Module envelope mismatch: {name}")
        modules[name] = module
        calibration_path = resolve_share_path(unit_dir, entry.get("unit_file", f"{name}_zero.yaml"))
        # Missing calibration is explicit None; only the protocol owner decides if it is required.
        calibrations[name] = (
            load(calibration_path, "unit_calibration_envelope.schema.json")
            if calibration_path.exists()
            else None
        )
        if calibrations[name] is not None and (
            calibrations[name]["module"] != entry.get("unit_module", name)
            or calibrations[name]["robot_variant"] != robot_variant
        ):
            raise ValueError(f"Calibration envelope mismatch: {name}")
        for reference in entry["device_refs"]:
            if reference not in devices:
                devices[reference] = load(resolve_share_path(root, reference), "device.schema.json")
    controllers = load(machine_dir / "controllers.yaml", "controllers.schema.json")
    safety = load(machine_dir / "safety_policy.yaml", "safety_policy.schema.json")
    return {
        "manifest": manifest,
        "deployment": deployment,
        "profile": profile,
        "scope": scope,
        "modules": modules,
        "calibrations": calibrations,
        "devices": devices,
        "controllers": controllers["controllers"],
        "safety_policy": safety,
    }
