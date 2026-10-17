"""Evaluate declarative controller parameter sources.

Every controller-specific choice lives in machine data. This module only reads
selected data, applies generic transforms and checks, and rejects duplicates.
"""

from copy import deepcopy
from pathlib import Path

import yaml

from .schema.primitives import resolve_share_path

KINDS = (
    "yaml",
    "policy",
    "controllers",
    "owner_parameters",
    "topics",
    "runtime",
    "policy_file",
)


def _lookup(value, path, context):
    for part in path.format(**context).split("."):
        if not isinstance(value, dict) or part not in value:
            raise ValueError(f"Missing parameter source path: {path}")
        value = value[part]
    return deepcopy(value)


def _transform(value, transform):
    if transform is None:
        return value
    if transform == "flatten":
        return [item for group in value for item in group]
    if transform == "lengths":
        return [len(group) for group in value]
    raise ValueError(f"Unsupported parameter transform: {transform}")


def _unique(values, context):
    if len(values) != len(set(values)):
        raise ValueError(f"Duplicate entries in {context}")


def _check_policy(value, source, base, env, binding, label):
    """Generic consistency checks between a policy value and composed resources."""
    match = source.get("match")
    if match == "managed_joints":
        _unique(value, label)
        if set(value) != set(env["managed_joints"]):
            raise ValueError(f"{label} must cover exactly the current managed joints")
    elif match == "actuator_groups":
        _unique(value, label)
        owner = env["rendered_modules"][binding["module"]]
        groups = [
            joint
            for group in binding["actuator_groups"]
            for joint in owner["actuator_groups"][group]
        ]
        if set(value) != set(groups):
            raise ValueError(f"{label} does not match controller actuator groups")
        _per_joint_arrays(base, len(value), label)
    elif match is not None:
        raise ValueError(f"Unsupported parameter check: {match}")
    if "partition" in source:
        batches = _lookup(base, source["partition"], env["context"])
        flattened = [item for batch in batches for item in batch]
        _unique(flattened, label + " partition")
        if any(not batch for batch in batches) or set(flattened) != set(value):
            raise ValueError(f"{label} partition must cover exactly its value")


def _per_joint_arrays(value, size, label):
    if isinstance(value, dict):
        for name, child in value.items():
            if name.endswith(("tolerances", "thresholds")) and len(child) != size:
                raise ValueError(f"Per-joint array length mismatch in {label}: {name}")
            _per_joint_arrays(child, size, label)


def _policy(source, env, binding, label):
    base = deepcopy(env["policies"][source["name"]])
    if "overlay" in source:
        base = {**base, **_lookup(base, source["overlay"], env["context"])}
    if "keys_from" in source:
        keys = _lookup(base, source["keys_from"], env["context"])
        value = {key: deepcopy(base[key]) for key in keys}
    elif "path" in source:
        value = _lookup(base, source["path"], env["context"])
    else:
        value = base
    _check_policy(value, source, base, env, binding, label)
    return _transform(value, source.get("transform"))


def _controllers(source, env):
    names = [name for name in env["selected"] if source["tag"] in env["tags"][name]]
    select = source["select"]
    if select == "all":
        return names
    if select == "any":
        return bool(names)
    if select == "first":
        if names:
            return names[0]
        if "default" in source:
            return source["default"]
        raise ValueError(f"No selected controller has tag {source['tag']}")
    raise ValueError(f"Unsupported controller selection: {select}")


def _evaluate(kind, source, name, binding, env):
    owner = env["rendered_modules"].get(binding["module"], {})
    if kind == "yaml":
        share = Path(env["package_share"](source["package"]))
        data = yaml.safe_load(resolve_share_path(share, source["file"]).read_text())
        return _lookup(data, source["path"], env["context"])
    if kind == "policy":
        return _policy(source, env, binding, f"{name}:{source['name']}")
    if kind == "controllers":
        return _controllers(source, env)
    if kind == "owner_parameters":
        return deepcopy(owner["controller_parameters"][name])
    if kind == "topics":
        context = owner["controller_context"][name]
        return {
            parameter: env["naming"][template].format(**context)
            for parameter, template in source["templates"].items()
        }
    if kind == "runtime":
        value = env["runtime_values"].get(source["key"])
        if value is None:
            env["runtime_parameters"].setdefault(name, {})[source["as"]] = source["key"]
        return value
    if kind == "policy_file":
        reference = env["policy_refs"][source["name"]]
        path = resolve_share_path(
            Path(env["package_share"](reference["package"])), reference["file"]
        )
        if path.name in env["runtime_files"]:
            raise ValueError(f"Duplicate runtime path: {path.name}")
        env["runtime_files"][path.name] = path.read_text()
        return str(Path(env["runtime_dir"]) / path.name)
    raise ValueError(f"Unsupported parameter source: {kind}")


def controller_parameters(name, binding, env):
    """Merge sources in order. A key from two sources has two authorities: reject it."""
    result = {}
    for source in binding.get("parameters", []):
        kinds = [kind for kind in KINDS if kind in source]
        if len(kinds) != 1:
            raise ValueError(f"Parameter source for {name} needs exactly one kind")
        if "when_policy" in source and source["when_policy"] not in env["policies"]:
            continue
        kind = kinds[0]
        if kind == "policy" and source[kind]["name"] not in env["policies"]:
            raise ValueError(f"{name} references unselected policy {source[kind]['name']}")
        value = _evaluate(kind, source[kind], name, binding, env)
        if kind == "runtime" and value is None:
            continue
        target = source[kind].get("as") if isinstance(source[kind], dict) else None
        if target is not None:
            value = {target: value}
        elif not isinstance(value, dict):
            raise ValueError(f"Parameter source {kind} for {name} needs 'as'")
        duplicate = set(result).intersection(value)
        if duplicate:
            raise ValueError(f"Duplicate parameter source for {name}: {sorted(duplicate)}")
        result.update(value)
    return result
