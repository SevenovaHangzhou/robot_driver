"""Cross-file identity, binding, policy and shared-system checks (X1-X9)."""

from copy import deepcopy

from .primitives import load_yaml, resolve_share_path


def require_unique(values, context):
    if len(values) != len(set(values)):
        raise ValueError(f"Duplicate {context}")


def manifest_links(data, config_root):
    manifest = data["manifest"]
    for name, entry in manifest["modules"].items():
        path = resolve_share_path(config_root, entry["config_file"])
        if not path.is_file():
            raise ValueError(f"Missing module file: {name}")
        envelope = data["modules"][name] if name in data["modules"] else load_yaml(path)
        if envelope["module"] != name or envelope["robot_variant"] != manifest["robot_variant"]:
            raise ValueError(f"Module envelope mismatch: {name}")
    for name, scope in manifest["scopes"].items():
        modules = scope["enabled_modules"] + scope["required_state_modules"]
        if not set(modules).issubset(manifest["modules"]):
            raise ValueError(f"Unknown scope module in {name}")
        for group in scope["actuator_groups"] + scope["jtc_groups"]:
            entry = manifest["modules"][group["module"]]
            if group["mode_group"] not in {mode["name"] for mode in entry["mode_groups"]}:
                raise ValueError(f"Unknown mode group in {name}")
    for name, profile in manifest["profiles"].items():
        if not set(profile["modules"]).issubset(manifest["modules"]) or not set(
            profile["allowed_scopes"]
        ).issubset(manifest["scopes"]):
            raise ValueError(f"Profile reference is not closed: {name}")
    require_unique([entry["name"] for entry in data["controllers"].values()], "controller name")
    for entry in data["controllers"].values():
        mode_names = {mode["name"] for mode in manifest["modules"][entry["module"]]["mode_groups"]}
        if not set(entry["actuator_groups"]).issubset(mode_names):
            raise ValueError("Controller references unknown actuator group")
        if not set(entry["required_state_modules"]).issubset(manifest["modules"]):
            raise ValueError("Controller references unknown state module")


def _runtime_blockers(value, path="manifest"):
    result = []
    if isinstance(value, dict):
        for key, child in value.items():
            if (
                key in {"verified", "certified"}
                and child is False
                or key == "status"
                and child == "draft"
            ):
                result.append(f"{path}.{key}")
            elif key == "pending_facts" and child:
                result.append(f"{path}.{key}")
            else:
                result.extend(_runtime_blockers(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            result.extend(_runtime_blockers(child, f"{path}[{index}]"))
    elif value == "TBD":
        result.append(path)
    return result


def runtime_admission(data, mock, require_manifest_ready=False):
    if not mock and (require_manifest_ready or data["profile"]["status"] == "ready"):
        active_transports = {
            data["manifest"]["modules"][name]["transport"] for name in data["modules"]
        }
        profile = deepcopy(data["profile"])
        if "layout" in profile:
            profile["layout"] = {
                key: value for key, value in profile["layout"].items() if key in active_transports
            }
        readiness = {
            "status": data["manifest"]["status"],
            "profile": profile,
            "modules": {
                name: {"descriptor": data["manifest"]["modules"][name], "facts": facts}
                for name, facts in data["modules"].items()
            },
        }
        blockers = _runtime_blockers(readiness)
        if blockers:
            raise ValueError("Manifest runtime blockers: " + ", ".join(blockers))


def enable_partition(policy, managed):
    names = policy["managed_joints"]
    batches = policy["enable_batches"]
    flattened = [name for batch in batches for name in batch]
    require_unique(names, "policy managed joint")
    require_unique(flattened, "enable batch joint")
    if (
        any(not batch for batch in batches)
        or set(names) != set(managed)
        or set(flattened) != set(names)
    ):
        raise ValueError("Enable policy must partition exactly the current managed joints")


def motion_binding(policy, groups):
    joints = policy["joints"]
    require_unique(joints, "motion policy joint")
    if set(joints) != set(groups):
        raise ValueError("Motion policy joints do not match controller actuator groups")

    def arrays(value):
        if isinstance(value, dict):
            for name, child in value.items():
                if name.endswith(("tolerances", "thresholds")) and len(child) != len(joints):
                    raise ValueError(f"Per-joint array length mismatch: {name}")
                arrays(child)

    arrays(policy)


def merge_systems(rendered):
    """Equal hardware IDs merge without normalizing conflicting owner parameters."""
    systems = {}
    all_names = []
    for owner, result in rendered:
        identifier = result["hardware_system_id"]
        all_names += [
            element.attrib["name"]
            for key in ("joints", "sensors", "gpios")
            for element in result.get(key, [])
        ]
        if identifier not in systems:
            systems[identifier] = {"owner": owner, "result": deepcopy(result)}
            continue
        target = systems[identifier]
        if (
            target["owner"] != owner
            or target["result"]["hardware_plugin"] != result["hardware_plugin"]
            or target["result"]["hardware_params"] != result["hardware_params"]
        ):
            raise ValueError(
                f"Conflicting plugin/master/cycle/timeout for hardware system {identifier}"
            )
        merged = target["result"]
        for key in ("joints", "sensors", "gpios", "managed_joints", "resource_claims"):
            merged.setdefault(key, []).extend(deepcopy(result.get(key, [])))
        for path, text in result["runtime_files"].items():
            if path in merged["runtime_files"]:
                raise ValueError(f"Duplicate runtime file: {path}")
            merged["runtime_files"][path] = text
        for key, value in result["expected_topology"].items():
            previous = merged["expected_topology"][key]
            if isinstance(value, list):
                previous.extend(deepcopy(value))
            elif key == "preflight_required":
                merged["expected_topology"][key] = previous or value
            elif previous != value:
                raise ValueError(f"Conflicting topology metadata for {identifier}")
        for key, names in result["actuator_groups"].items():
            merged["actuator_groups"].setdefault(key, []).extend(names)
    # Health sensors belong to the master and are emitted once by the protocol owner.
    # Across merged modules, exact matching health fragments may be coalesced only there.
    for system in systems.values():
        require_unique(system["result"].get("resource_claims", []), "physical resource claim")
        require_unique(system["result"]["managed_joints"], "managed joint")
    require_unique(all_names, "hardware resource name")
    return tuple(systems.values())
