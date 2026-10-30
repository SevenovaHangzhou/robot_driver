"""Deploy-time composition artifacts.

The explicit compose command is the only writer and the only place configuration is validated.
Launch reads the artifact, checks its selection and file hashes, and uses it as written; it does
not recompose. Changing installed configuration therefore requires regenerating the artifact.
"""

from hashlib import sha256
from pathlib import Path
import shlex
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from .compose import compose
from .model import Composed
from .model import ARTIFACT
from .schema.primitives import load_yaml, resolve_share_path

# 2: the manifest also carries runtime parameter keys and owner preflight topology.
SCHEMA_VERSION = 2
KIND = "rt_control_composition_artifact"
# Startup-specific values such as startup_id are deliberately not part of a selection.
SELECTION_DEFAULTS = {"use_mock_hardware": True, "jtc_only": False, "sync_mode": ""}
SELECTION_KEYS = (
    "robot_variant",
    "machine_unit",
    "physical_profile",
    "control_scope",
    *SELECTION_DEFAULTS,
)


class _PlainDumper(yaml.SafeDumper):
    """Shared objects are written out in full: the loader rejects YAML anchors and aliases."""

    def ignore_aliases(self, data):
        return True


def _digest(text):
    return sha256(text.encode("utf-8")).hexdigest()


def selection(**values):
    unknown = set(values).difference(SELECTION_KEYS)
    if unknown:
        raise ValueError(f"Unknown artifact selection keys: {sorted(unknown)}")
    result = {**SELECTION_DEFAULTS, **values}
    missing = [key for key in SELECTION_KEYS if key not in result]
    if missing:
        raise ValueError(f"Missing artifact selection keys: {missing}")
    return {key: result[key] for key in SELECTION_KEYS}


def generate_command(directory, chosen):
    """Exact regeneration command reported with every rejection."""
    arguments = [
        "ros2",
        "run",
        "rt_control_composer",
        "compose_artifact",
        "--output",
        str(directory),
    ]
    for key in ("robot_variant", "machine_unit", "physical_profile", "control_scope"):
        arguments += ["--" + key.replace("_", "-"), chosen[key]]
    arguments.append("--mock" if chosen["use_mock_hardware"] else "--real")
    if chosen["jtc_only"]:
        arguments.append("--jtc-only")
    if chosen["sync_mode"]:
        arguments += ["--sync-mode", chosen["sync_mode"]]
    return shlex.join(arguments)


def _absolute(directory):
    path = Path(directory)
    if not str(directory) or not path.is_absolute():
        raise ValueError("The composition artifact directory must be an absolute path")
    return path


def write_artifact(directory, *, package_share=get_package_share_directory, **values):
    """Compose for the final path, stage beside it, then replace atomically by rename."""
    output = _absolute(directory)
    chosen = selection(**values)
    composed = compose(**chosen, runtime_dir=output, package_share=package_share)
    if output.exists() and not (output / ARTIFACT).is_file():
        raise ValueError(f"Refusing to replace a directory that is not an artifact: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "kind": KIND,
        "selection": chosen,
        "files": {name: _digest(text) for name, text in sorted(composed.files.items())},
        "runtime_parameters": composed.runtime_parameters,
        "preflight": [
            {"owner": system["owner"], "expected_topology": system["result"]["expected_topology"]}
            for system in composed.systems
        ],
    }
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent))
    previous = output.parent / f".{output.name}.previous"
    try:
        for name, text in composed.files.items():
            path = resolve_share_path(staging, name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
        (staging / ARTIFACT).write_text(
            yaml.dump(manifest, Dumper=_PlainDumper, sort_keys=False), "utf-8"
        )
        if previous.exists():
            shutil.rmtree(previous)
        if output.exists():
            output.rename(previous)
        staging.rename(output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        if previous.exists() and not output.exists():
            previous.rename(output)
        raise
    shutil.rmtree(previous, ignore_errors=True)
    return composed


def load_artifact(directory, **expected):
    """Return the composition as written; reject a missing, modified or mismatched artifact."""
    root = _absolute(directory)
    wanted = selection(**expected)
    command = generate_command(root, wanted)
    path = root / ARTIFACT
    if not path.is_file():
        raise ValueError(f"Missing composition artifact {root}; generate it with: {command}")
    manifest = load_yaml(path)
    if manifest.get("schema_version") != SCHEMA_VERSION or manifest.get("kind") != KIND:
        raise ValueError(f"Unsupported composition artifact {path}; regenerate with: {command}")
    if manifest["selection"] != wanted:
        different = sorted(
            key for key in SELECTION_KEYS if manifest["selection"].get(key) != wanted[key]
        )
        raise ValueError(f"Artifact selection differs in {different}; regenerate with: {command}")
    present = {item.relative_to(root).as_posix() for item in root.rglob("*") if item.is_file()} - {
        ARTIFACT
    }
    if present != set(manifest["files"]):
        raise ValueError(f"Artifact file set differs from its manifest; regenerate with: {command}")
    files = {}
    for name, digest in manifest["files"].items():
        files[name] = (root / name).read_text(encoding="utf-8")
        if _digest(files[name]) != digest:
            raise ValueError(f"Artifact file was modified: {name}; regenerate with: {command}")
    lifecycle = yaml.safe_load(files["lifecycle.yaml"])
    return Composed(
        robot_description=files["robot_description.xml"],
        controllers=yaml.safe_load(files["controllers.yaml"]),
        controller_order=yaml.safe_load(files["controller_order.yaml"]),
        runtime_dir=root,
        systems=tuple(
            {"owner": entry["owner"], "result": {"expected_topology": entry["expected_topology"]}}
            for entry in manifest["preflight"]
        ),
        use_mock_hardware=wanted["use_mock_hardware"],
        files=files,
        runtime_parameters=manifest["runtime_parameters"],
        hardware_activation_order=tuple(lifecycle["hardware_activation_order"]),
        staged_hardware_activation=lifecycle["staged_hardware_activation"],
        lifecycle=lifecycle,
    )


def runtime_arguments(composed, values):
    """Node-scoped `-p controller:parameter:=value` overrides for per-start values."""
    arguments = []
    for controller, parameters in sorted(composed.runtime_parameters.items()):
        for parameter, key in sorted(parameters.items()):
            if key not in values:
                raise ValueError(f"Missing runtime value {key} for {controller}.{parameter}")
            arguments += ["-p", f"{controller}:{parameter}:={values[key]}"]
    return ["--ros-args", *arguments] if arguments else []
