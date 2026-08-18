#!/usr/bin/env python3
"""Validate source-code projections of the RT-Control driver variant.

Structured YAML/Xacro assets are handled by ``driver_variant_projections``.
This module covers the remaining compile-time/runtime source constants without
executing production Python, compiling C++, or applying an upstream patch.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from collections.abc import Iterator
from pathlib import Path
from typing import Mapping, Sequence

from tools import driver_variant
from tools import driver_variant_projections
from tools.driver_variant_projections import ProjectionView, derive_projection


STATUS_ADAPTER = (
    "src/rt_control/control_api_adapter/control_api_adapter/status_adapter.py"
)
RT_DIAGNOSTICS = "src/rt_control/rt_diagnostics/src/rt_diagnostics_node.cpp"
ENABLE_MANAGER_HEADER = (
    "src/rt_control/enable_manager/include/enable_manager/"
    "enable_manager_controller.hpp"
)
ENABLE_MANAGER_CPP = (
    "src/rt_control/enable_manager/src/enable_manager_controller.cpp"
)
ROS2_CANOPEN_PATCH = (
    "patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch"
)
AXIS_STATE_CHECKER = "tools/rt_control_axis_state_check.py"

_SOURCES = (
    STATUS_ADAPTER,
    RT_DIAGNOSTICS,
    ENABLE_MANAGER_HEADER,
    ENABLE_MANAGER_CPP,
    ROS2_CANOPEN_PATCH,
    AXIS_STATE_CHECKER,
)

_DEFAULT_MANIFEST_RELATIVE = driver_variant.DEFAULT_MANIFEST.relative_to(
    driver_variant.ROOT
)


def load_source_projection_sources(repository_root: Path) -> dict[str, str]:
    """Read all source projections as UTF-8 without importing or compiling them."""
    root = repository_root.resolve()
    sources: dict[str, str] = {}
    findings: list[str] = []
    for relative in _SOURCES:
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError:
            findings.append(f"{relative}: projection path escapes repository root")
            continue
        try:
            sources[relative] = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            findings.append(f"{relative}: cannot read projection: {exc}")
    if findings:
        raise driver_variant.DriverVariantError(findings)
    return sources


def _chunked(items: list[str], size: int) -> list[list[str]]:
    return [items[index : index + size] for index in range(0, len(items), size)]


def _cpp_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def render_enable_manager_topology(document: Mapping[str, object]) -> str:
    """Render the only compile-time topology block used by enable_manager."""
    view = derive_projection(document)
    joint_indices = {
        name: index for index, name in enumerate(view.whole_body_joints)
    }
    batches = document["enable"]["batches"]  # type: ignore[index]
    if not isinstance(batches, list):
        raise driver_variant.DriverVariantError("enable.batches: must be a list")
    indexed_batches: list[list[int]] = []
    for batch in batches:
        members = batch["joints"]
        if len(members) > 3:
            raise driver_variant.DriverVariantError(
                f"enable batch {batch['id']!r}: enable_manager supports at most 3 joints"
            )
        indexed_batches.append([joint_indices[name] for name in members])

    names = [_cpp_string(name) for name in view.whole_body_joints]
    lines = [
        "  // DRIVER_VARIANT_PROJECTION_BEGIN:enable_manager_topology",
        f"  static constexpr std::size_t kAxisCount = {len(names)}U;",
        f"  static constexpr std::size_t kBatchCount = {len(indexed_batches)}U;",
        "  inline static constexpr std::array<const char *, kAxisCount> kJointNames = {",
    ]
    name_chunks = _chunked(names, 5)
    for index, chunk in enumerate(name_chunks):
        suffix = "," if index + 1 < len(name_chunks) else "};"
        lines.append("    " + ", ".join(chunk) + suffix)
    padded_batches = [batch + [-1] * (3 - len(batch)) for batch in indexed_batches]
    batch_text = ", ".join(
        "{" + ", ".join(str(item) for item in batch) + "}"
        for batch in padded_batches
    )
    sizes = ", ".join(f"{len(batch)}U" for batch in indexed_batches)
    lines.extend(
        [
            "  inline static constexpr "
            "std::array<std::array<std::int8_t, 3>, kBatchCount>",
            f"    kEnableBatches = {{{{{batch_text}}}}};",
            "  inline static constexpr std::array<std::uint8_t, kBatchCount> "
            f"kBatchSizes = {{{sizes}}};",
            "  // DRIVER_VARIANT_PROJECTION_END:enable_manager_topology",
        ]
    )
    return "\n".join(lines)


def render_rt_diagnostics_topology(document: Mapping[str, object]) -> str:
    """Render diagnostics joint/ring/node topology and full responder count."""
    view = derive_projection(document)
    ring_positions = ", ".join(str(axis.ring_position) for axis in view.ethercat_axes)
    joint_names = [_cpp_string(axis.joint) for axis in view.ethercat_axes]
    node_ids = ", ".join(f"{axis.node_id}U" for axis in view.canopen_axes)
    lines = [
        "// DRIVER_VARIANT_PROJECTION_BEGIN:rt_diagnostics_topology",
        f"constexpr std::array<int, {len(view.ethercat_axes)}> kRingPositions = "
        f"{{{ring_positions}}};",
        f"constexpr std::array<const char *, {len(view.ethercat_axes)}> kJointNames = {{",
    ]
    chunks = _chunked(joint_names, 5)
    for index, chunk in enumerate(chunks):
        suffix = "," if index + 1 < len(chunks) else "};"
        lines.append("  " + ", ".join(chunk) + suffix)
    lines.extend(
        [
            f"constexpr std::array<std::uint8_t, {len(view.canopen_axes)}> "
            f"kCanopenNodeIds = {{{node_ids}}};",
            "constexpr std::size_t kExpectedEthercatResponders = "
            f"{len(view.ring_positions)}U;",
            "// DRIVER_VARIANT_PROJECTION_END:rt_diagnostics_topology",
        ]
    )
    return "\n".join(lines)


def render_ros2_canopen_track_nodes(document: Mapping[str, object]) -> str:
    """Render the CANopen safety node/mode table embedded in the upstream patch."""
    view = derive_projection(document)
    entries = ", ".join(
        f"{{{axis.node_id}U, {axis.operation_mode}U}}"
        for axis in sorted(view.canopen_axes, key=lambda item: item.node_id)
    )
    return "\n".join(
        [
            "// DRIVER_VARIANT_PROJECTION_BEGIN:ros2_canopen_track_nodes",
            "struct TrackNodeConfiguration",
            "{",
            "  uint8_t node_id;",
            "  uint16_t operation_mode;",
            "};",
            f"constexpr std::array<TrackNodeConfiguration, {len(view.canopen_axes)}> "
            "kTrackNodeConfigurations = {{",
            f"  {entries}}}}};",
            "// DRIVER_VARIANT_PROJECTION_END:ros2_canopen_track_nodes",
        ]
    )


def _extract_marker_block(
    text: str,
    identifier: str,
    path: str,
    findings: list[str],
    *,
    patch_added_lines: bool = False,
) -> str | None:
    plain_begin = f"// DRIVER_VARIANT_PROJECTION_BEGIN:{identifier}"
    plain_end = f"// DRIVER_VARIANT_PROJECTION_END:{identifier}"
    prefix = "+" if patch_added_lines else ""
    begin = prefix + plain_begin
    end = prefix + plain_end
    lines = text.splitlines()
    begin_indexes = [index for index, line in enumerate(lines) if line.strip() == begin]
    end_indexes = [index for index, line in enumerate(lines) if line.strip() == end]
    if len(begin_indexes) != 1 or len(end_indexes) != 1:
        if patch_added_lines and (
            any(line.strip() == plain_begin for line in lines)
            or any(line.strip() == plain_end for line in lines)
        ):
            findings.append(
                f"{path}: {identifier} marker block must use only added patch lines"
            )
        else:
            findings.append(
                f"{path}: {identifier} begin/end marker must occur exactly once"
            )
        return None
    start = begin_indexes[0]
    finish = end_indexes[0]
    if finish < start:
        findings.append(f"{path}: {identifier} end marker precedes begin marker")
        return None
    block_lines = lines[start : finish + 1]
    if patch_added_lines:
        if any(not line.startswith("+") or line.startswith("++") for line in block_lines):
            findings.append(
                f"{path}: {identifier} marker block must use only added patch lines"
            )
            return None
        block_lines = [line[1:] for line in block_lines]
    return "\n".join(block_lines)


def _check_marker(
    text: str,
    path: str,
    identifier: str,
    expected: str,
    findings: list[str],
    *,
    patch_added_lines: bool = False,
) -> None:
    actual = _extract_marker_block(
        text,
        identifier,
        path,
        findings,
        patch_added_lines=patch_added_lines,
    )
    if actual is not None and actual != expected:
        findings.append(f"{path}: {identifier} projection drift")


def _literal_assignment(
    tree: ast.Module,
    name: str,
    path: str,
    findings: list[str],
) -> tuple[int, ...] | None:
    matches: list[ast.AST] = []
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            matches.append(statement.value)
        elif isinstance(statement, ast.AnnAssign) and isinstance(
            statement.target, ast.Name
        ) and statement.target.id == name:
            matches.append(statement.value)
    if (
        len(matches) != 1
        or len(_runtime_name_stores(tree.body, name)) != 1
        or _has_indirect_runtime_name_store(tree.body, name)
        or _has_global_declaration(tree, name)
    ):
        findings.append(f"{path}: {name} must be assigned at top level exactly once")
        return None
    try:
        value = ast.literal_eval(matches[0])
    except (ValueError, TypeError, SyntaxError):
        findings.append(f"{path}: {name} must be a literal tuple of integer IDs")
        return None
    if (
        not isinstance(value, tuple)
        or any(not isinstance(item, int) or isinstance(item, bool) for item in value)
    ):
        findings.append(f"{path}: {name} must be a literal tuple of integer IDs")
        return None
    return value


def _top_level_assignment_value(tree: ast.Module, name: str) -> ast.AST | None:
    values: list[ast.AST] = []
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            values.append(statement.value)
        elif isinstance(statement, ast.AnnAssign) and isinstance(
            statement.target, ast.Name
        ) and statement.target.id == name:
            values.append(statement.value)
    return (
        values[0]
        if len(values) == 1
        and len(_runtime_name_stores(tree.body, name)) == 1
        and not _has_indirect_runtime_name_store(tree.body, name)
        and not _has_global_declaration(tree, name)
        else None
    )


def _runtime_name_stores(nodes: Sequence[ast.AST], name: str) -> list[ast.Name]:
    """Collect stores in one runtime scope, excluding nested definitions."""
    return [
        node
        for node in _walk_runtime_nodes(nodes)
        if isinstance(node, ast.Name)
        and isinstance(node.ctx, ast.Store)
        and node.id == name
    ]


def _runtime_self_attribute_stores(
    nodes: Sequence[ast.AST], name: str
) -> list[ast.Attribute]:
    return [
        node
        for node in _walk_runtime_nodes(nodes)
        if isinstance(node, ast.Attribute)
        and isinstance(node.ctx, ast.Store)
        and _is_self_attribute(node, name)
    ]


def _has_indirect_runtime_name_store(
    nodes: Sequence[ast.AST], name: str
) -> bool:
    for node in _walk_runtime_nodes(nodes):
        if (
            isinstance(node, ast.Subscript)
            and isinstance(node.ctx, ast.Store)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id in {"globals", "locals", "vars"}
            and not node.value.args
            and not node.value.keywords
            and isinstance(node.slice, ast.Constant)
            and node.slice.value == name
        ):
            return True
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id in {"setattr", "delattr"}
            and len(node.args) >= 2
            and isinstance(node.args[1], ast.Constant)
            and node.args[1].value == name
        ):
            return True
    return False


def _has_global_declaration(tree: ast.AST, name: str) -> bool:
    return any(
        isinstance(node, ast.Global) and name in node.names for node in ast.walk(tree)
    )


def _all_name_stores(tree: ast.AST, name: str) -> list[ast.Name]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Name)
        and isinstance(node.ctx, ast.Store)
        and node.id == name
    ]


def _has_indirect_self_attribute_store(
    nodes: Sequence[ast.AST], name: str
) -> bool:
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in {"setattr", "delattr"}
        and len(node.args) >= 2
        and isinstance(node.args[0], ast.Name)
        and node.args[0].id == "self"
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value == name
        for node in _walk_runtime_nodes(nodes)
    )


def _has_exact_import(tree: ast.Module, module: str, name: str) -> bool:
    matches = [
        alias
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module == module
        for alias in node.names
        if alias.name == name
    ]
    return len(matches) == 1 and matches[0].asname is None


def _references_name(node: ast.AST | None, name: str) -> bool:
    return node is not None and any(
        isinstance(item, ast.Name)
        and isinstance(item.ctx, ast.Load)
        and item.id == name
        for item in ast.walk(node)
    )


def _direct_tuple_generator(value: ast.AST | None, source: str) -> bool:
    """Return whether ``tuple(...)`` directly projects every item in source."""
    if (
        not isinstance(value, ast.Call)
        or not isinstance(value.func, ast.Name)
        or value.func.id != "tuple"
        or len(value.args) != 1
        or value.keywords
        or not isinstance(value.args[0], ast.GeneratorExp)
    ):
        return False
    generator = value.args[0]
    if len(generator.generators) != 1:
        return False
    clause = generator.generators[0]
    return (
        isinstance(clause.target, ast.Name)
        and isinstance(clause.iter, ast.Name)
        and clause.iter.id == source
        and not clause.ifs
        and not clause.is_async
        and _references_name(generator.elt, clause.target.id)
    )


def _top_level_function(
    tree: ast.Module, name: str
) -> ast.FunctionDef | ast.AsyncFunctionDef | None:
    matches = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    ]
    return matches[0] if len(matches) == 1 else None


def _class_method(
    tree: ast.Module, class_name: str, method_name: str
) -> ast.FunctionDef | ast.AsyncFunctionDef | None:
    classes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef) and node.name == class_name
    ]
    if len(classes) != 1:
        return None
    methods = [
        node
        for node in classes[0].body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == method_name
    ]
    return methods[0] if len(methods) == 1 else None


def _is_self_attribute(node: ast.AST, name: str) -> bool:
    return (
        isinstance(node, ast.Attribute)
        and node.attr == name
        and isinstance(node.value, ast.Name)
        and node.value.id == "self"
    )


def _status_callback_tracks_topology(
    method: ast.FunctionDef | ast.AsyncFunctionDef | None,
) -> bool:
    if method is None or len(method.body) != 1:
        return False
    assignment = method.body[0]
    if not (
        isinstance(assignment, ast.Assign)
        and len(assignment.targets) == 1
        and _is_self_attribute(assignment.targets[0], "_diagnostic_state")
        and len(_runtime_self_attribute_stores(method.body, "_diagnostic_state")) == 1
    ):
        return False
    value = assignment.value
    return (
        isinstance(value, ast.Call)
        and isinstance(value.func, ast.Name)
        and value.func.id == "update_diagnostic_state"
        and len(value.args) == 3
        and not value.keywords
        and _is_self_attribute(value.args[0], "_diagnostic_state")
        and isinstance(value.args[1], ast.Attribute)
        and value.args[1].attr == "status"
        and isinstance(value.args[1].value, ast.Name)
        and value.args[1].value.id == "message"
        and isinstance(value.args[2], ast.Call)
        and isinstance(value.args[2].func, ast.Attribute)
        and value.args[2].func.attr == "monotonic"
        and isinstance(value.args[2].func.value, ast.Name)
        and value.args[2].func.value.id == "time"
        and not value.args[2].args
        and not value.args[2].keywords
    )


def _snapshot_components(node: ast.AST, snapshot: str) -> bool:
    return (
        isinstance(node, ast.Attribute)
        and node.attr == "components"
        and isinstance(node.value, ast.Name)
        and node.value.id == snapshot
    )


def _component_call_uses_snapshot(
    value: ast.AST, snapshot: str, first_argument: str | None = None
) -> bool:
    if not (
        isinstance(value, ast.Call)
        and _is_self_attribute(value.func, "_component")
        and len(value.args) == 2
        and _snapshot_components(value.args[1], snapshot)
    ):
        return False
    return first_argument is None or (
        isinstance(value.args[0], ast.Name) and value.args[0].id == first_argument
    )


def _projects_topology_group(value: ast.AST, group: str, snapshot: str) -> bool:
    if not isinstance(value, ast.ListComp) or len(value.generators) != 1:
        return False
    generator = value.generators[0]
    if not isinstance(generator.target, ast.Name):
        return False
    iterator = generator.iter
    if not (
        isinstance(iterator, ast.Attribute)
        and iterator.attr == group
        and isinstance(iterator.value, ast.Attribute)
        and iterator.value.attr == "topology"
        and isinstance(iterator.value.value, ast.Name)
        and iterator.value.value.id == snapshot
    ):
        return False
    return (
        _component_call_uses_snapshot(value.elt, snapshot, generator.target.id)
        and not generator.ifs
        and not generator.is_async
    )


def _summary_projects_diagnostic_topology(
    method: ast.FunctionDef | ast.AsyncFunctionDef | None,
) -> bool:
    if method is None or len(method.body) != 2:
        return False
    snapshot = "diagnostic_state"
    assignments = [
        node
        for node in _walk_runtime_nodes(method.body)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        and any(
            isinstance(target, ast.Name) and target.id == snapshot
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
        )
    ]
    if len(assignments) != 1 or not _is_self_attribute(
        assignments[0].value, "_diagnostic_state"
    ):
        return False
    returns = [
        node.value
        for node in _walk_runtime_nodes(method.body)
        if isinstance(node, ast.Return) and isinstance(node.value, ast.Call)
    ]
    if len(returns) != 1:
        return False
    call = returns[0]
    if not isinstance(call.func, ast.Name) or call.func.id != "build_safety_summary":
        return False
    keywords = {keyword.arg: keyword.value for keyword in call.keywords if keyword.arg}
    return (
        _component_call_uses_snapshot(
            keywords.get("enable_manager", ast.Constant(None)), snapshot
        )
        and _component_call_uses_snapshot(
            keywords.get("ethercat_master", ast.Constant(None)), snapshot
        )
        and _projects_topology_group(
            keywords.get("ethercat_slaves", ast.Constant(None)),
            "ethercat_slaves",
            snapshot,
        )
        and _projects_topology_group(
            keywords.get("canopen_nodes", ast.Constant(None)),
            "canopen_nodes",
            snapshot,
        )
    )


def _status_callbacks_are_serialized(
    method: ast.FunctionDef | ast.AsyncFunctionDef | None,
) -> bool:
    if method is None:
        return False
    assignments = [
        node.value
        for node in _walk_runtime_nodes(method.body)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        and any(
            _is_self_attribute(target, "_callback_group")
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
        )
    ]
    if not (
        len(assignments) == 1
        and isinstance(assignments[0], ast.Call)
        and isinstance(assignments[0].func, ast.Name)
        and assignments[0].func.id == "MutuallyExclusiveCallbackGroup"
        and not assignments[0].args
        and not assignments[0].keywords
        and not _has_indirect_self_attribute_store(method.body, "_callback_group")
    ):
        return False
    callback_calls = [
        node
        for node in _walk_runtime_nodes(method.body)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and _is_self_attribute(node.func, node.func.attr)
        and node.func.attr in {"create_subscription", "create_timer"}
    ]
    return bool(callback_calls) and all(
        any(
            keyword.arg == "callback_group"
            and _is_self_attribute(keyword.value, "_callback_group")
            for keyword in call.keywords
        )
        for call in callback_calls
    )


def _walk_runtime_nodes(nodes: Sequence[ast.AST]) -> Iterator[ast.AST]:
    """Walk executable nodes without accepting references in nested definitions."""
    pending = list(reversed(nodes))
    while pending:
        node = pending.pop()
        if isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)
        ):
            continue
        yield node
        children = list(ast.iter_child_nodes(node))
        pending.extend(reversed(children))


def _writes_observed_joint(node: ast.AST) -> bool:
    if not isinstance(node, (ast.Assign, ast.AnnAssign)):
        return False
    targets = node.targets if isinstance(node, ast.Assign) else [node.target]
    return any(
        isinstance(target, ast.Subscript)
        and isinstance(target.value, ast.Name)
        and target.value.id == "observed"
        and isinstance(target.slice, ast.Name)
        and target.slice.id == "joint"
        for target in targets
    )


def _statement_shape(source: str) -> str:
    statements = ast.parse(source).body
    if len(statements) != 1:
        raise AssertionError("canonical source must contain exactly one statement")
    return ast.dump(statements[0], include_attributes=False)


_AXIS_EXTRACTION_LOOP_SHAPE = _statement_shape(
    """
for joint in AXES:
    interfaces = records.get(joint)
    if not isinstance(interfaces, Mapping):
        raise AxisStateError(f"{joint}: missing joint interface record")
    names = interfaces.get("interface_names")
    values = interfaces.get("values")
    if not isinstance(names, list) or not isinstance(values, list):
        raise AxisStateError(f"{joint}: invalid interface_names/values")
    try:
        status_index = names.index("status_word")
    except ValueError as error:
        raise AxisStateError(f"{joint}: missing status_word") from error
    if status_index >= len(values):
        raise AxisStateError(f"{joint}: missing status_word value")

    value = values[status_index]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AxisStateError(f"{joint}: status_word is not numeric")
    numeric = float(value)
    if not math.isfinite(numeric) or not numeric.is_integer():
        raise AxisStateError(f"{joint}: status_word is not an integer: {value!r}")
    status_word = int(numeric)
    if not 0 <= status_word <= 0xFFFF:
        raise AxisStateError(f"{joint}: status_word is outside uint16: {status_word}")
    observed[joint] = status_word
"""
)

_AXIS_POLICY_LOOP_SHAPE = _statement_shape(
    """
for joint, status_word in observed.items():
    if expected == "enabled":
        if status_word & 0x006F != 0x0027:
            raise AxisStateError(
                f"{joint}: raw=0x{status_word:04X}, expected=OperationEnabled(0x0027)"
            )
        continue

    switch_on_disabled = status_word & 0x004F == 0x0040
    ready_to_switch_on_terminal = (
        joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES
        and status_word & 0x006F == 0x0021
    )
    if not switch_on_disabled and not ready_to_switch_on_terminal:
        expected_state = "SwitchOnDisabled(0x0040)"
        if joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES:
            expected_state += " or configured ReadyToSwitchOn(0x0021)"
        raise AxisStateError(
            f"{joint}: raw=0x{status_word:04X}, expected={expected_state}"
        )
"""
)


def _axes_drive_runtime_extraction(
    function: ast.FunctionDef | ast.AsyncFunctionDef | None,
) -> bool:
    if function is None or len(function.body) < 2:
        return False
    loop = function.body[-2]
    result = function.body[-1]
    returns = [
        node
        for node in _walk_runtime_nodes(function.body)
        if isinstance(node, ast.Return)
    ]
    return (
        len(returns) == 1
        and returns[0] is result
        and ast.dump(loop, include_attributes=False) == _AXIS_EXTRACTION_LOOP_SHAPE
        and isinstance(result, ast.Return)
        and isinstance(result.value, ast.Name)
        and result.value.id == "observed"
    )


def _is_policy_membership(node: ast.AST, policy_name: str) -> bool:
    return (
        isinstance(node, ast.Compare)
        and len(node.ops) == 1
        and isinstance(node.ops[0], ast.In)
        and isinstance(node.left, ast.Name)
        and node.left.id == "joint"
        and len(node.comparators) == 1
        and isinstance(node.comparators[0], ast.Name)
        and node.comparators[0].id == policy_name
    )


def _is_ready_to_switch_on_status(node: ast.AST) -> bool:
    return (
        isinstance(node, ast.Compare)
        and len(node.ops) == 1
        and isinstance(node.ops[0], ast.Eq)
        and len(node.comparators) == 1
        and isinstance(node.comparators[0], ast.Constant)
        and node.comparators[0].value == 0x0021
        and isinstance(node.left, ast.BinOp)
        and isinstance(node.left.op, ast.BitAnd)
        and isinstance(node.left.left, ast.Name)
        and node.left.left.id == "status_word"
        and isinstance(node.left.right, ast.Constant)
        and node.left.right.value == 0x006F
    )


def _is_disable_terminal_guard(node: ast.AST) -> bool:
    if not (
        isinstance(node, ast.BoolOp)
        and isinstance(node.op, ast.And)
        and len(node.values) == 2
    ):
        return False
    expected = {"switch_on_disabled", "ready_to_switch_on_terminal"}
    actual = {
        value.operand.id
        for value in node.values
        if isinstance(value, ast.UnaryOp)
        and isinstance(value.op, ast.Not)
        and isinstance(value.operand, ast.Name)
    }
    return actual == expected


def _policy_drives_runtime_check(
    function: ast.FunctionDef | ast.AsyncFunctionDef | None,
    policy_name: str,
) -> bool:
    if function is None:
        return False
    if len(function.body) < 2:
        return False
    loop = function.body[-2]
    result = function.body[-1]
    returns = [
        node
        for node in _walk_runtime_nodes(function.body)
        if isinstance(node, ast.Return)
    ]
    if not (
        ast.dump(loop, include_attributes=False) == _AXIS_POLICY_LOOP_SHAPE
        and isinstance(result, ast.Return)
        and isinstance(result.value, ast.Name)
        and result.value.id == "observed"
        and len(returns) == 1
        and returns[0] is result
    ):
        return False
    terminal_assignment = loop.body[2]
    if not (
        isinstance(terminal_assignment, ast.Assign)
        and len(terminal_assignment.targets) == 1
        and isinstance(terminal_assignment.targets[0], ast.Name)
        and terminal_assignment.targets[0].id == "ready_to_switch_on_terminal"
    ):
        return False
    policy_value = terminal_assignment.value
    if not (
        isinstance(policy_value, ast.BoolOp)
        and isinstance(policy_value.op, ast.And)
        and len(policy_value.values) == 2
        and _is_policy_membership(policy_value.values[0], policy_name)
        and _is_ready_to_switch_on_status(policy_value.values[1])
    ):
        return False
    guard = loop.body[3]
    return (
        isinstance(guard, ast.If)
        and _is_disable_terminal_guard(guard.test)
        and any(
            isinstance(item, ast.Raise) for item in _walk_runtime_nodes(guard.body)
        )
    )


def _strip_cpp_comments(text: str) -> str:
    """Mask C++ comments while retaining literals and line structure."""
    output: list[str] = []
    index = 0
    state = "code"
    while index < len(text):
        char = text[index]
        following = text[index + 1] if index + 1 < len(text) else ""
        if state == "code":
            if char == "/" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if char == "/" and following == "*":
                output.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            output.append(char)
            if char == '"':
                state = "string"
            elif char == "'":
                state = "character"
            index += 1
            continue
        if state == "line_comment":
            output.append("\n" if char == "\n" else " ")
            index += 1
            if char == "\n":
                state = "code"
            continue
        if state == "block_comment":
            if char == "*" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "code"
            else:
                output.append("\n" if char == "\n" else " ")
                index += 1
            continue

        output.append(char)
        index += 1
        if char == "\\" and index < len(text):
            output.append(text[index])
            index += 1
        elif (state == "string" and char == '"') or (
            state == "character" and char == "'"
        ):
            state = "code"
    return "".join(output)


def _strip_cpp_literals(text: str) -> str:
    """Mask string and character literals before matching C++ identifiers."""
    output: list[str] = []
    index = 0
    quote = ""
    while index < len(text):
        char = text[index]
        if not quote:
            if char in {'"', "'"}:
                quote = char
                output.append(" ")
            else:
                output.append(char)
            index += 1
            continue
        output.append("\n" if char == "\n" else " ")
        index += 1
        if char == "\\" and index < len(text):
            escaped = text[index]
            output.append("\n" if escaped == "\n" else " ")
            index += 1
        elif char == quote:
            quote = ""
    return "".join(output)


def _has_cpp_patterns(text: str, patterns: Sequence[str]) -> bool:
    return all(re.search(pattern, text) is not None for pattern in patterns)


def _literal_string_tuple_assignment(
    tree: ast.Module,
    name: str,
    path: str,
    findings: list[str],
) -> tuple[str, ...] | None:
    matches: list[ast.AST] = []
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            matches.append(statement.value)
        elif isinstance(statement, ast.AnnAssign) and isinstance(
            statement.target, ast.Name
        ) and statement.target.id == name:
            matches.append(statement.value)
    if (
        len(matches) != 1
        or len(_runtime_name_stores(tree.body, name)) != 1
        or _has_indirect_runtime_name_store(tree.body, name)
        or _has_global_declaration(tree, name)
    ):
        findings.append(f"{path}: {name} must be assigned at top level exactly once")
        return None
    try:
        value = ast.literal_eval(matches[0])
    except (ValueError, TypeError, SyntaxError):
        findings.append(f"{path}: {name} must be a literal tuple of strings")
        return None
    if not isinstance(value, tuple) or any(not isinstance(item, str) for item in value):
        findings.append(f"{path}: {name} must be a literal tuple of strings")
        return None
    return value


def _check_status_adapter(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    try:
        tree = ast.parse(text, filename=STATUS_ADAPTER)
    except SyntaxError as exc:
        findings.append(f"{STATUS_ADAPTER}: invalid Python syntax: {exc}")
        return
    if not _has_exact_import(
        tree, "rclpy.callback_groups", "MutuallyExclusiveCallbackGroup"
    ):
        findings.append(
            f"{STATUS_ADAPTER}: MutuallyExclusiveCallbackGroup must be imported "
            "exactly once without alias"
        )
    elif _all_name_stores(tree, "MutuallyExclusiveCallbackGroup") or (
        _has_indirect_runtime_name_store(
            tree.body, "MutuallyExclusiveCallbackGroup"
        )
    ):
        findings.append(
            f"{STATUS_ADAPTER}: MutuallyExclusiveCallbackGroup must not be rebound"
        )
    for helper in ("update_diagnostic_state", "build_safety_summary"):
        if _top_level_function(tree, helper) is None:
            findings.append(
                f"{STATUS_ADAPTER}: {helper} must be defined at top level exactly once"
            )
        elif (
            _all_name_stores(tree, helper)
            or _has_indirect_runtime_name_store(tree.body, helper)
            or _has_global_declaration(tree, helper)
        ):
            findings.append(f"{STATUS_ADAPTER}: {helper} must not be rebound")
    expected_ecat = tuple(axis.ring_position for axis in view.ethercat_axes)
    expected_can = tuple(axis.node_id for axis in view.canopen_axes)
    for name, expected in (
        ("ETHERCAT_DRIVE_POSITIONS", expected_ecat),
        ("CANOPEN_NODE_IDS", expected_can),
    ):
        actual = _literal_assignment(tree, name, STATUS_ADAPTER, findings)
        if actual is not None and actual != expected:
            findings.append(
                f"{STATUS_ADAPTER}: {name} {actual!r} does not match manifest "
                f"{expected!r}"
            )
    for target, source in (
        ("ETHERCAT_SLAVE_NAMES", "ETHERCAT_DRIVE_POSITIONS"),
        ("CANOPEN_NODE_NAMES", "CANOPEN_NODE_IDS"),
    ):
        if not _direct_tuple_generator(
            _top_level_assignment_value(tree, target), source
        ):
            findings.append(
                f"{STATUS_ADAPTER}: {source} must directly drive the {target} "
                "runtime name projection"
            )
    status_class = "RtStatusAdapterNode"
    if not _status_callback_tracks_topology(
        _class_method(tree, status_class, "_on_diagnostics")
    ):
        findings.append(
            f"{STATUS_ADAPTER}: managed IDs must feed the runtime diagnostic callback"
        )
    if not _summary_projects_diagnostic_topology(
        _class_method(tree, status_class, "_build_summary")
    ):
        findings.append(
            f"{STATUS_ADAPTER}: observed managed IDs must feed the runtime safety summary"
        )
    if not _status_callbacks_are_serialized(
        _class_method(tree, status_class, "__init__")
    ):
        findings.append(
            f"{STATUS_ADAPTER}: callback group must serialize diagnostic snapshots"
        )


def _check_axis_state_checker(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    try:
        tree = ast.parse(text, filename=AXIS_STATE_CHECKER)
    except SyntaxError as exc:
        findings.append(f"{AXIS_STATE_CHECKER}: invalid Python syntax: {exc}")
        return
    for name, expected in (
        ("AXES", view.whole_body_joints),
        (
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES",
            view.ready_to_switch_on_disable_terminal_joints,
        ),
    ):
        actual = _literal_string_tuple_assignment(
            tree, name, AXIS_STATE_CHECKER, findings
        )
        if actual is not None and actual != expected:
            findings.append(
                f"{AXIS_STATE_CHECKER}: {name} {actual!r} does not match "
                f"manifest {expected!r}"
            )
    extract_function = _top_level_function(tree, "_extract_status_words")
    if not _axes_drive_runtime_extraction(extract_function):
        findings.append(
            f"{AXIS_STATE_CHECKER}: AXES must directly drive the runtime extraction"
        )

    policy_name = "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES"
    check_function = _top_level_function(tree, "check_axis_states")
    if not _policy_drives_runtime_check(check_function, policy_name):
        findings.append(
            f"{AXIS_STATE_CHECKER}: {policy_name} must directly drive the "
            "runtime policy"
        )


def _check_rt_diagnostics_consumers(text: str, findings: list[str]) -> None:
    block = _extract_marker_block(
        text,
        "rt_diagnostics_topology",
        RT_DIAGNOSTICS,
        findings,
    )
    outside = text.replace(block, "", 1) if block is not None else text
    uncommented = _strip_cpp_comments(outside)
    code = _strip_cpp_literals(uncommented)
    forbidden = (
        "std::stoi(status.hardware_id)",
        "node + 2U",
        "std::array<CanopenSnapshot, 2>",
        'status.hardware_id != "2"',
    )
    if any(pattern in uncommented for pattern in forbidden):
        findings.append(
            f"{RT_DIAGNOSTICS}: legacy hard-coded consumer remains outside "
            "rt_diagnostics_topology"
        )
    required_consumers = {
        "kRingPositions": (
            r"for\s*\(\s*std::size_t\s+axis\s*=\s*0\s*;\s*axis\s*<\s*"
            r"kRingPositions\s*\.\s*size\s*\(\s*\)\s*;\s*\+\+axis\s*\)",
            r"\bposition\s*=\s*kRingPositions\s*\[\s*axis\s*\]\s*;",
        ),
        "kJointNames": (
            r"std::string\s*\(\s*kJointNames\s*\[\s*axis\s*\]\s*\)\s*"
            r"\+\s*[^;]*status_word",
        ),
        "kCanopenNodeIds": (
            r"std::find_if\s*\(\s*kCanopenNodeIds\s*\.\s*begin\s*\(\s*\)\s*,\s*"
            r"kCanopenNodeIds\s*\.\s*end\s*\(\s*\)",
            r"\bnode_id\s*=\s*kCanopenNodeIds\s*\[\s*node\s*\]\s*;",
        ),
        "kExpectedEthercatResponders": (
            r"static_cast\s*<\s*int\s*>\s*\(\s*slaves_responding\s*\)\s*!=\s*"
            r"static_cast\s*<\s*int\s*>\s*\(\s*kExpectedEthercatResponders\s*\)",
        ),
    }
    for name, patterns in required_consumers.items():
        if not _has_cpp_patterns(code, patterns):
            findings.append(
                f"{RT_DIAGNOSTICS}: {name} is missing a required runtime consumer "
                "outside its projection block"
            )


def _check_enable_manager_consumers(text: str, findings: list[str]) -> None:
    uncommented = _strip_cpp_comments(text)
    code = _strip_cpp_literals(uncommented)
    if "is_ti5_axis" in code or (
        "axis == 1U" in code and "axis == 2U" in code
    ):
        findings.append(
            f"{ENABLE_MANAGER_CPP}: legacy hard-coded disable-terminal consumer remains"
        )
    required = (
        '"ready_to_switch_on_disable_terminal_joints"',
        "ready_to_switch_on_disable_terminal_[axis]",
    )
    for token in required:
        if token not in uncommented:
            findings.append(
                f"{ENABLE_MANAGER_CPP}: configured disable-terminal policy consumer "
                f"{token!r} is missing"
            )
    required_consumers = {
        "kJointNames": (
            r"for\s*\(\s*const\s+char\s*\*\s*joint\s*:\s*kJointNames\s*\)",
            r"std::find_if\s*\(\s*kJointNames\s*\.\s*begin\s*\(\s*\)\s*,\s*"
            r"kJointNames\s*\.\s*end\s*\(\s*\)",
            r"std::distance\s*\(\s*kJointNames\s*\.\s*begin\s*\(\s*\)\s*,\s*joint\s*\)",
        ),
        "kEnableBatches": (
            r"kEnableBatches\s*\[\s*batch\s*\]\s*\[\s*item\s*\]",
            r"kEnableBatches\s*\[\s*current_batch_\s*\]\s*\[\s*item\s*\]",
            r"kEnableBatches\s*\[\s*current_batch_\s*\]\s*\[\s*0\s*\]",
        ),
        "kBatchSizes": (
            r"kBatchSizes\s*\[\s*batch\s*\]",
            r"kBatchSizes\s*\[\s*current_batch_\s*\]",
        ),
    }
    for name, patterns in required_consumers.items():
        if not _has_cpp_patterns(code, patterns):
            findings.append(
                f"{ENABLE_MANAGER_CPP}: {name} is missing a required runtime consumer"
            )


def _ros2_canopen_patch_section(text: str, findings: list[str]) -> str | None:
    header = (
        "diff --git a/canopen_ros2_control/src/cia402_system.cpp "
        "b/canopen_ros2_control/src/cia402_system.cpp"
    )
    starts = [
        index for index, line in enumerate(text.splitlines()) if line == header
    ]
    if len(starts) != 1:
        findings.append(
            f"{ROS2_CANOPEN_PATCH}: cia402_system.cpp diff section must occur exactly once"
        )
        return None
    lines = text.splitlines()
    start = starts[0]
    finish = next(
        (
            index
            for index in range(start + 1, len(lines))
            if lines[index].startswith("diff --git ")
        ),
        len(lines),
    )
    return "\n".join(lines[start:finish])


def _check_ros2_canopen_patch(
    text: str, document: Mapping[str, object], findings: list[str]
) -> None:
    section = _ros2_canopen_patch_section(text, findings)
    if section is None:
        return
    expected = render_ros2_canopen_track_nodes(document)
    _check_marker(
        section,
        ROS2_CANOPEN_PATCH,
        "ros2_canopen_track_nodes",
        expected,
        findings,
        patch_added_lines=True,
    )
    required = (
        "!isConfiguredTrackNode(id)",
        "configurations.size() != kTrackNodeConfigurations.size()",
    )
    if any(token not in section for token in required) or section.count(
        "for (const auto & configuration : kTrackNodeConfigurations)"
    ) < 2:
        findings.append(
            f"{ROS2_CANOPEN_PATCH}: legacy hard-coded consumer or disconnected "
            "track-node table remains"
        )
    actual_block = _extract_marker_block(
        section,
        "ros2_canopen_track_nodes",
        ROS2_CANOPEN_PATCH,
        [],
        patch_added_lines=True,
    )
    outside = section
    if actual_block is not None:
        added_block = "\n".join("+" + line for line in actual_block.splitlines())
        outside = section.replace(added_block, "", 1)
    forbidden = (
        "id != 2U",
        "id != 3U",
        "node_id != 2U",
        "node_id != 3U",
        "{2U, 3U}",
    )
    if any(token in outside for token in forbidden):
        findings.append(
            f"{ROS2_CANOPEN_PATCH}: legacy hard-coded consumer remains outside "
            "ros2_canopen_track_nodes"
        )
    added_code = "\n".join(
        line[1:]
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    runtime_code = _strip_cpp_comments(added_code)
    runtime_patterns = (
        r"bool\s+isConfiguredTrackNode\s*\(\s*const\s+uint8_t\s+node_id\s*\)"
        r"\s*\{\s*return\s+std::any_of\s*\(",
        r"if\s*\(\s*emcy\.eec\s*==\s*0U\s*\|\|\s*"
        r"!isConfiguredTrackNode\s*\(\s*id\s*\)\s*\)\s*\{\s*return\s*;\s*\}",
        r"ros2_canopen::COData\s+mode_command\s*=\s*\{\s*0x6060U\s*,\s*"
        r"0x00U\s*,\s*static_cast\s*<\s*uint32_t\s*>\s*\(\s*"
        r"configuration\.operation_mode\s*\)\s*\}\s*;",
        r"driver\s*->\s*get_mode\s*\(\s*\)\s*!=\s*"
        r"configuration\.operation_mode\s*&&\s*!\s*driver\s*->\s*"
        r"set_operation_mode\s*\(\s*configuration\.operation_mode\s*\)",
    )
    if not _has_cpp_patterns(runtime_code, runtime_patterns):
        findings.append(
            f"{ROS2_CANOPEN_PATCH}: track-node table must directly control runtime "
            "mode and EMCY safety"
        )


def validate_source_projections(
    document: Mapping[str, object], sources: Mapping[str, str]
) -> None:
    """Aggregate source-projection drift into one fail-closed error."""
    view = derive_projection(document)
    findings: list[str] = []
    for path in _SOURCES:
        if path not in sources:
            findings.append(f"{path}: missing source projection")

    if STATUS_ADAPTER in sources:
        _check_status_adapter(sources[STATUS_ADAPTER], view, findings)
    if AXIS_STATE_CHECKER in sources:
        _check_axis_state_checker(sources[AXIS_STATE_CHECKER], view, findings)
    if ENABLE_MANAGER_HEADER in sources:
        _check_marker(
            sources[ENABLE_MANAGER_HEADER],
            ENABLE_MANAGER_HEADER,
            "enable_manager_topology",
            render_enable_manager_topology(document),
            findings,
        )
    if ENABLE_MANAGER_CPP in sources:
        _check_enable_manager_consumers(sources[ENABLE_MANAGER_CPP], findings)
    if RT_DIAGNOSTICS in sources:
        _check_marker(
            sources[RT_DIAGNOSTICS],
            RT_DIAGNOSTICS,
            "rt_diagnostics_topology",
            render_rt_diagnostics_topology(document),
            findings,
        )
        _check_rt_diagnostics_consumers(sources[RT_DIAGNOSTICS], findings)
    if ROS2_CANOPEN_PATCH in sources:
        _check_ros2_canopen_patch(
            sources[ROS2_CANOPEN_PATCH], document, findings
        )
    if findings:
        raise driver_variant.DriverVariantError(findings)


def validate_repository_projections(
    repository_root: Path,
    manifest_path: Path | None = None,
) -> dict[str, object]:
    """Validate the manifest and every structured/source projection offline."""
    root = repository_root.resolve()
    path = manifest_path or _DEFAULT_MANIFEST_RELATIVE
    if not path.is_absolute():
        path = root / path
    document = driver_variant.load_manifest(path, repository_root=root)

    findings: list[str] = []
    try:
        structured_sources = driver_variant_projections.load_structured_sources(
            document, root
        )
        driver_variant_projections.validate_structured_projections(
            document, structured_sources
        )
    except driver_variant.DriverVariantError as exc:
        findings.extend(exc.findings)
    try:
        source_projections = load_source_projection_sources(root)
        validate_source_projections(document, source_projections)
    except driver_variant.DriverVariantError as exc:
        findings.extend(exc.findings)
    if findings:
        raise driver_variant.DriverVariantError(findings)
    return document


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=driver_variant.ROOT,
        help="repository root (defaults to the parent of tools/)",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        help="manifest path, relative to --repository-root unless absolute",
    )
    args = parser.parse_args(argv)
    try:
        document = validate_repository_projections(
            args.repository_root, args.manifest
        )
    except driver_variant.DriverVariantError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"PASS: driver variant projections for {document['variant_id']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
