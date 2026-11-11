"""Resolve declared startup policies through protocol owners, then compose normally."""

from pathlib import Path
import json
import shutil
import yaml
from ament_index_python.packages import get_package_share_directory
from .artifact import write_artifact
from .compose import compose
from .renderer_registry import select_renderer
from .schema.manifest_load import load_machine


def write_startup_artifact(
    output,
    *,
    workspace,
    choices=None,
    scan=None,
    head_probe=None,
    package_share=get_package_share_directory,
    **selection,
):
    root = Path(package_share("hwcfg_machines"))
    schema = Path(package_share("integ_composer")) / "schema"
    data = load_machine(
        root,
        schema,
        *[
            selection[k]
            for k in ("robot_variant", "machine_unit", "physical_profile", "control_scope")
        ],
    )
    declarations = data["manifest"]["modules"]
    declared = {
        name: declarations[name] for name in data["modules"] if "startup" in declarations[name]
    }
    if not declared:
        raise ValueError("This scope has no declared startup policies")
    chosen = {}
    for key, mode in (choices or {}).items():
        matches = [
            name
            for name, entry in declared.items()
            if key in (name, entry["startup"].get("option"))
        ]
        if len(matches) != 1:
            raise ValueError(f"Startup selection does not identify one module: {key}")
        name = matches[0]
        if name in chosen:
            raise ValueError(f"Startup module specified twice: {name}")
        if declared[name]["startup"]["policy"] == "required" and mode not in ("required", "auto"):
            raise ValueError(f"Required module cannot be excluded: {name}")
        chosen[name] = mode
    reference = compose(**selection, runtime_dir=Path(output), package_share=package_share)
    groups = {}
    for name, entry in declared.items():
        groups.setdefault(entry["transport"], {})[name] = data["modules"][name]
    results = []
    for transport, modules in groups.items():
        owner = select_renderer(data["manifest"]["owner_refs"], transport)
        hook = getattr(owner, "select_startup", None)
        if hook is None:
            raise ValueError(f"No startup selection implementation for {transport}")
        result = hook(
            dict(
                modules=modules,
                declarations={name: declared[name] for name in modules},
                calibrations={name: data["calibrations"][name] for name in modules},
                deployment=data["deployment"],
                choices=chosen,
                systems=reference.systems,
                mock=selection.get("use_mock_hardware", True),
                scan=scan,
                probe=head_probe,
            )
        )
        results.append(result)
    work = Path(workspace)
    if not work.is_absolute():
        raise ValueError("Startup selection workspace must be absolute")
    work.mkdir(parents=True, exist_ok=False)
    source = work / "hwcfg_machines"
    shutil.copytree(root, source)
    manifest = data["manifest"]
    scope = manifest["scopes"][selection["control_scope"]]
    rows = []
    mapping = None
    for result in results:
        for name, machine in result["modules"].items():
            _save(source / declarations[name]["config_file"], machine)
        for name, calibration in result.get("calibrations", {}).items():
            path = f"units/{selection['machine_unit']}/{declarations[name].get('unit_file', name + '_zero.yaml')}"
            _save(source / path, calibration)
        _remove_modules(scope, result.get("removed", []), result.get("excluded_controllers", []))
        rows.extend(result["rows"])
        if "position_mapping" in result:
            mapping = result["position_mapping"]
            manifest["profiles"][selection["physical_profile"]]["layout"]["ethercat"][
                "ring_positions"
            ] = list(mapping.values())
            (work / "ethercat-scan.txt").write_text(result["scan"])
    _save(source / f"machines/{selection['robot_variant']}/manifest.yaml", manifest)
    report = dict(
        schema_version=1,
        choices=chosen,
        modules=rows,
        selection=selection,
        source=str(source),
        position_mapping=mapping,
    )
    (work / "selection.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    result = write_artifact(
        output,
        package_share=lambda name: str(source) if name == "hwcfg_machines" else package_share(name),
        additional_files={
            "module_selection.yaml": yaml.safe_dump(report, sort_keys=False, allow_unicode=True)
        },
        **selection,
    )
    return result, report


def _save(path, value):
    path.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True))


def _remove_modules(scope, removed, excluded_controllers):
    removed = set(removed)
    excluded = set(excluded_controllers)
    for key in ("enabled_modules", "required_state_modules"):
        scope[key] = [item for item in scope.get(key, []) if item not in removed]
    for key in ("actuator_groups", "jtc_groups"):
        scope[key] = [group for group in scope.get(key, []) if group["module"] not in removed]
    bindings = scope.get("controller_parameter_bindings", {})
    for controller, binding in list(bindings.items()):
        parameters = binding["parameters"]
        keys = [key for key, value in parameters.items() if value.get("module") in removed]
        if "joints" in keys and not parameters["joints"].get("append", False):
            excluded.add(controller)
        for key in keys:
            del parameters[key]
        if not parameters or controller in excluded:
            del bindings[controller]
    for phase in ("active", "inactive"):
        scope["controller_order"][phase] = [
            name for name in scope["controller_order"][phase] if name not in excluded
        ]
