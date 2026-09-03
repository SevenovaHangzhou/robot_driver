from __future__ import annotations

from dataclasses import replace
from typing import Any, Mapping

from .fault_catalog import FaultCatalog
from .fault_log import FaultEvent


_TOPOLOGY = {
    1: ("right_joint1", "ZeroErr"),
    2: ("right_joint2", "TI5"),
    3: ("right_joint3", "TI5"),
    4: ("right_joint4", "ZeroErr"),
    5: ("right_joint5", "ZeroErr"),
    6: ("right_joint6", "ZeroErr"),
    7: ("left_joint1", "ZeroErr"),
    8: ("left_joint2", "TI5"),
    9: ("left_joint3", "TI5"),
    10: ("left_joint4", "ZeroErr"),
    11: ("left_joint5", "ZeroErr"),
    12: ("left_joint6", "ZeroErr"),
    14: ("right_force_sensor", "X503"),
    15: ("left_force_sensor", "X503"),
    16: ("turn", "ZeroErr"),
    17: ("updown", "XMC"),
}


def _diagnostic_level(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int(value[0]) if len(value) == 1 else 3
    return int(value)


def _values(status: Any) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in getattr(status, "values", ())}


def _number(values: Mapping[str, str], *keys: str) -> int:
    for key in keys:
        raw = values.get(key)
        if raw is None or not str(raw).strip():
            continue
        try:
            return int(str(raw).strip(), 0)
        except ValueError:
            try:
                return int(float(str(raw).strip()))
            except ValueError:
                continue
    return 0


def _address(source: str, values: Mapping[str, str]) -> int:
    explicit = _number(values, "position", "ring_position", "address")
    if explicit:
        return explicit
    marker = "/slave_"
    if marker in source:
        try:
            return int(source.rsplit(marker, 1)[1])
        except ValueError:
            return -1
    return -1


class DiagnosticFaultProjector:
    """Project standard diagnostics into the UI's private incident model."""

    def __init__(self, catalog: FaultCatalog) -> None:
        self._catalog = catalog
        self._active_by_source: dict[str, FaultEvent] = {}

    def project(self, status: Any, timestamp_s: float) -> FaultEvent | None:
        source = str(status.name)
        level = _diagnostic_level(status.level)
        if level == 0:
            active = self._active_by_source.pop(source, None)
            return None if active is None else replace(
                active, timestamp_s=float(timestamp_s), active=False
            )

        values = _values(status)
        address = _address(source, values)
        topology_joint, topology_vendor = _TOPOLOGY.get(address, ("", ""))
        joint = (
            values.get("joint")
            or values.get("sensor")
            or values.get("failed_joint")
            or topology_joint
        )
        vendor = values.get("vendor") or topology_vendor or "Unknown"
        status_word = _number(
            values, "status_word_hex", "status_word_raw", "failed_status_word"
        )
        error_code = _number(
            values, "error_code_hex", "error_code_raw", "error_code", "last_emcy_code"
        )
        definition = self._catalog.lookup(vendor, error_code)
        known_code = error_code != 0 and definition.known
        message = definition.title_zh if known_code else str(status.message or "未知故障")
        event = FaultEvent(
            timestamp_s=float(timestamp_s),
            source=source,
            joint=joint,
            address=address,
            vendor=vendor,
            status_word=status_word,
            error_code=error_code,
            message=message,
            severity=definition.severity if known_code else ("warning" if level == 1 else "error"),
            reset_policy=definition.reset_policy if known_code else "do_not_reset",
            active=True,
            possible_causes=definition.possible_causes,
            operator_actions=definition.operator_actions,
            manual_title=definition.manual_title,
            manual_version=definition.manual_version,
            manual_pages=definition.manual_pages,
            source_reference=definition.source_reference,
        )
        self._active_by_source[source] = event
        return event
