from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml


ALLOWED_SEVERITIES = frozenset({"info", "warning", "error", "critical"})
ALLOWED_RESET_POLICIES = frozenset(
    {
        "do_not_reset",
        "reset_once_after_link_recovered",
        "reset_once_after_condition_resolved",
        "restart_required",
    }
)


class CatalogValidationError(ValueError):
    """Raised when a fault catalog entry lacks authoritative metadata."""


@dataclass(frozen=True)
class FaultDefinition:
    vendor: str
    code: int
    title_zh: str
    severity: str
    reset_policy: str
    possible_causes: tuple[str, ...]
    operator_actions: tuple[str, ...]
    manual_title: str
    manual_version: str
    manual_pages: str
    source_reference: str
    known: bool = True


def _required_text(entry: Mapping[str, Any], key: str, source: Path) -> str:
    value = entry.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CatalogValidationError(f"{source}: {key} must be non-empty text")
    return value.strip()


def _required_text_list(
    entry: Mapping[str, Any], key: str, source: Path
) -> tuple[str, ...]:
    value = entry.get(key)
    if not isinstance(value, list) or not value:
        raise CatalogValidationError(f"{source}: {key} must be a non-empty list")
    items = tuple(str(item).strip() for item in value)
    if any(not item for item in items):
        raise CatalogValidationError(f"{source}: {key} contains empty text")
    return items


def _parse_code(value: Any, source: Path) -> int:
    if isinstance(value, bool):
        raise CatalogValidationError(f"{source}: code must be an integer or hex text")
    if isinstance(value, int):
        code = value
    elif isinstance(value, str):
        try:
            code = int(value.strip(), 0)
        except ValueError as exc:
            raise CatalogValidationError(f"{source}: invalid code {value!r}") from exc
    else:
        raise CatalogValidationError(f"{source}: code must be an integer or hex text")
    if not 0 <= code <= 0xFFFFFFFF:
        raise CatalogValidationError(f"{source}: code is outside uint32 range")
    return code


def _parse_entry(entry: Any, source: Path) -> FaultDefinition:
    if not isinstance(entry, Mapping):
        raise CatalogValidationError(f"{source}: each fault must be a mapping")
    severity = _required_text(entry, "severity", source)
    if severity not in ALLOWED_SEVERITIES:
        raise CatalogValidationError(f"{source}: unknown severity {severity!r}")
    reset_policy = _required_text(entry, "reset_policy", source)
    if reset_policy not in ALLOWED_RESET_POLICIES:
        raise CatalogValidationError(
            f"{source}: unknown reset_policy {reset_policy!r}"
        )
    return FaultDefinition(
        vendor=_required_text(entry, "vendor", source),
        code=_parse_code(entry.get("code"), source),
        title_zh=_required_text(entry, "title_zh", source),
        severity=severity,
        reset_policy=reset_policy,
        possible_causes=_required_text_list(entry, "possible_causes", source),
        operator_actions=_required_text_list(entry, "operator_actions", source),
        manual_title=_required_text(entry, "manual_title", source),
        manual_version=_required_text(entry, "manual_version", source),
        manual_pages=_required_text(entry, "manual_pages", source),
        source_reference=_required_text(entry, "source_reference", source),
    )


class FaultCatalog:
    def __init__(self, definitions: Iterable[FaultDefinition]) -> None:
        self._definitions: dict[tuple[str, int], FaultDefinition] = {}
        for definition in definitions:
            key = (definition.vendor.casefold(), definition.code)
            if key in self._definitions:
                raise CatalogValidationError(
                    f"duplicate fault entry: {definition.vendor} 0x{definition.code:04X}"
                )
            self._definitions[key] = definition

    @classmethod
    def load_directory(cls, directory: Path | str) -> "FaultCatalog":
        root = Path(directory)
        if not root.is_dir():
            raise CatalogValidationError(f"catalog directory does not exist: {root}")
        definitions: list[FaultDefinition] = []
        files = sorted(root.glob("*.yaml"))
        if not files:
            raise CatalogValidationError(f"catalog directory has no YAML files: {root}")
        for path in files:
            try:
                document = yaml.safe_load(path.read_text(encoding="utf-8"))
            except (OSError, yaml.YAMLError) as exc:
                raise CatalogValidationError(f"{path}: cannot load YAML: {exc}") from exc
            if not isinstance(document, Mapping) or not isinstance(
                document.get("faults"), list
            ):
                raise CatalogValidationError(f"{path}: root faults list is required")
            definitions.extend(
                _parse_entry(entry, path) for entry in document["faults"]
            )
        return cls(definitions)

    def lookup(self, vendor: str, code: int) -> FaultDefinition:
        key = (str(vendor).casefold(), int(code))
        definition = self._definitions.get(key)
        if definition is not None:
            return definition
        return FaultDefinition(
            vendor=str(vendor),
            code=int(code),
            title_zh="未知厂商错误",
            severity="critical",
            reset_policy="do_not_reset",
            possible_causes=("当前错误码目录没有经过手册验证的解释",),
            operator_actions=("停止自动恢复并查阅对应厂商手册",),
            manual_title="未收录",
            manual_version="未收录",
            manual_pages="未收录",
            source_reference="unavailable",
            known=False,
        )
