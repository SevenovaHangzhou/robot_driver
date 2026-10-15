"""Deploy-time composition artifacts.

The explicit compose command is the only writer. Launch reads an artifact,
checks file hashes, and recomposes in memory from the installed configuration.
Any difference rejects startup before a Node exists.
"""

from hashlib import sha256
from pathlib import Path
import shlex
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from .compose import compose
from .model import ARTIFACT
from .schema.primitives import load_yaml, resolve_share_path

SCHEMA_VERSION = 1
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
    }
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent))
    previous = output.parent / f".{output.name}.previous"
    try:
        for name, text in composed.files.items():
            path = resolve_share_path(staging, name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
        (staging / ARTIFACT).write_text(yaml.safe_dump(manifest, sort_keys=False), "utf-8")
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


def load_artifact(directory, *, package_share=get_package_share_directory, **expected):
    """Return the verified composition; reject missing, modified, mismatched or stale output."""
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
    for name, digest in manifest["files"].items():
        if _digest((root / name).read_text(encoding="utf-8")) != digest:
            raise ValueError(f"Artifact file was modified: {name}; regenerate with: {command}")
    composed = compose(**wanted, runtime_dir=root, package_share=package_share)
    current = {name: _digest(text) for name, text in composed.files.items()}
    if current != manifest["files"]:
        stale = sorted(
            name
            for name in set(current) | set(manifest["files"])
            if current.get(name) != manifest["files"].get(name)
        )
        raise ValueError(
            f"Artifact is stale against installed configuration {stale}; regenerate with: {command}"
        )
    return composed


def runtime_arguments(composed, values):
    """Node-scoped `-p controller:parameter:=value` overrides for per-start values."""
    arguments = []
    for controller, parameters in sorted(composed.runtime_parameters.items()):
        for parameter, key in sorted(parameters.items()):
            if key not in values:
                raise ValueError(f"Missing runtime value {key} for {controller}.{parameter}")
            arguments += ["-p", f"{controller}:{parameter}:={values[key]}"]
    return ["--ros-args", *arguments] if arguments else []
