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
    "state_ready",
    "exports",
    "interface",
    "model_limits",
    "joint_values",
)
# A source that resolves to nothing contributes no parameter (keeps unrelated outputs unchanged).
_SKIP = object()


def _lookup(value, path, context):
    for part in path.format(**context).split("."):
        if not isinstance(value, dict) or part not in value:
            raise ValueError(f"Missing parameter source path: {path}")
        value = value[part]
    return deepcopy(value)


def _has(value, path, context):
    for part in path.format(**context).split("."):
        if not isinstance(value, dict) or part not in value:
            return False
        value = value[part]
    return True


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
        if source.get("optional") and not _has(base, source["path"], env["context"]):
            return _SKIP
        value = _lookup(base, source["path"], env["context"])
    else:
        value = base
    _check_policy(value, source, base, env, binding, label)
    return _transform(value, source.get("transform"))


def _controllers(source, env):
    names = [name for name in env["selected"] if source["tag"] in env["tags"][name]]
    select = source["select"]
    if select == "all":
        # ROS parameter YAML cannot carry a typed empty array; optional omits it instead.
        return _SKIP if not names and source.get("optional") else names
    if select == "any":
        return bool(names)
    if select == "first":
        if names:
            return source["format"].format(name=names[0]) if "format" in source else names[0]
        if source.get("optional"):
            return _SKIP
        if "default" in source:
            return source["default"]
        raise ValueError(f"No selected controller has tag {source['tag']}")
    raise ValueError(f"Unsupported controller selection: {select}")


def _joint_values(source, env):
    """One value per policy joint, from instance-keyed module exports.

    Each group names a module export mapping instances to joints, and a value: a scalar from a
    package YAML (yaml) or an export with the same instance keys (value_export), optionally
    divided by another such export (divide_by_export), e.g. a wheel-surface speed by its radius.
    """
    joints = _lookup(env["policies"][source["policy"]], source["path"], env["context"])
    values = {}
    for group in source["groups"]:
        exports = env["rendered_modules"][group["module"]]["parameter_exports"]
        if "yaml" in group:
            share = Path(env["package_share"](group["yaml"]["package"]))
            data = yaml.safe_load(resolve_share_path(share, group["yaml"]["file"]).read_text())
            scalar = _lookup(data, group["yaml"]["path"], env["context"])
        for instance, joint in exports[group["joints_export"]].items():
            value = scalar if "yaml" in group else exports[group["value_export"]][instance]
            divisor = (
                exports[group["divide_by_export"]][instance] if "divide_by_export" in group else 1
            )
            if joint in values or not all(
                isinstance(item, (int, float)) and not isinstance(item, bool)
                for item in (value, divisor)
            ):
                raise ValueError(f"Joint value for {joint} is duplicated or not a number")
            values[joint] = float(value) / float(divisor)
    if set(values) != set(joints):
        raise ValueError(f"Joint values do not cover exactly {joints}")
    return [values[joint] for joint in joints]


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
    if kind == "state_ready":
        # Required state sources gate enabling through their "ready" state interfaces.
        return env.get("state_ready", []) or _SKIP
    if kind == "interface":
        # Public ROS names have one owner: config/interface_naming.yaml. A policy may choose the
        # name key (policy/path), e.g. the mode service of the group it configures.
        if "policy" in source:
            policy = env["policies"][source["policy"]]
            if source.get("optional") and not _has(policy, source["path"], {}):
                return _SKIP
            return env["naming"][_lookup(policy, source["path"], {})]
        return env["naming"][source["name"]]
    if kind == "joint_values":
        return _joint_values(source, env)
    if kind == "model_limits":
        # Joint limits have one owner: the robot model's URDF <limit>, in the order of a policy list.
        joints = _lookup(env["policies"][source["policy"]], source["path"], env["context"])
        limits = env["model_limits"]
        missing = [joint for joint in joints if source["attribute"] not in limits.get(joint, {})]
        if missing:
            raise ValueError(f"Robot model has no {source['attribute']} limit for {missing}")
        return [limits[joint][source["attribute"]] for joint in joints]
    if kind == "exports":
        # One list from every selected module that exports it, in module order.
        values = [
            item
            for module in env["rendered_modules"].values()
            for item in module.get("parameter_exports", {}).get(source["name"], [])
        ]
        return values or _SKIP
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
        if value is _SKIP or (kind == "runtime" and value is None):
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
