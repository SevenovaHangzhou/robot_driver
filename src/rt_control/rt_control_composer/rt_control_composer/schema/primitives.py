#!/usr/bin/env python3
"""Offline validation for machine and policy YAML, using installed Draft 7 schemas."""

import json
import math
from pathlib import Path, PurePosixPath
from typing import Any, Iterator
from urllib.parse import urlsplit

import yaml
from jsonschema import Draft7Validator, RefResolver, ValidationError, validators
from yaml.tokens import AliasToken, AnchorToken


class ConfigError(ValueError):
    """Invalid configuration or an unsafe schema/reference boundary."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate/merge keys before JSON Schema validation."""

    def construct_mapping(self, node: yaml.MappingNode, deep: bool = False) -> dict:
        result = {}
        for key_node, value_node in node.value:
            if key_node.value == "<<":
                raise ConfigError("YAML merge keys are forbidden")
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, str):
                raise ConfigError("YAML mapping keys must be strings")
            if key in result:
                raise ConfigError(f"Duplicate YAML key: {key}")
            result[key] = self.construct_object(value_node, deep=deep)
        return result


def _check_json_values(value: Any) -> None:
    if isinstance(value, dict):
        for child in value.values():
            _check_json_values(child)
    elif isinstance(value, list):
        for child in value:
            _check_json_values(child)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ConfigError("Non-finite YAML numbers are forbidden")
    elif value is not None and not isinstance(value, (str, int, float, bool)):
        raise ConfigError("YAML values must be JSON-compatible; quote dates and timestamps")


def load_yaml(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    try:
        if any(isinstance(token, (AnchorToken, AliasToken)) for token in yaml.scan(text)):
            raise ConfigError("YAML anchors and aliases are forbidden")
        data = yaml.load(text, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as error:
        raise ConfigError(f"Invalid single-document YAML: {error}") from error
    _check_json_values(data)
    return data


def resolve_share_path(root: Path, reference: str) -> Path:
    """Resolve a POSIX package-share reference without traversal or symlink escape."""
    path = PurePosixPath(reference)
    if (
        not reference
        or path.is_absolute()
        or ".." in path.parts
        or "\\" in reference
        or ":" in reference
        or (path.parts and path.parts[0] == "evidence")
        or any(character in reference for character in ("\n", "\r", "\x00"))
    ):
        raise ConfigError(f"Invalid package-share relative path: {reference!r}")
    resolved = (root / reference).resolve()
    if not resolved.is_relative_to(root.resolve()):
        raise ConfigError(f"Reference escapes package share: {reference!r}")
    return resolved


def _schema_nodes(value: Any) -> Iterator[dict]:
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from _schema_nodes(child)
    elif isinstance(value, list):
        for child in value:
            yield from _schema_nodes(child)


class _InstalledResolver(RefResolver):
    def resolve_remote(self, uri: str) -> Any:
        raise ConfigError(f"Schema reference is not in the installed store: {uri}")


def _schema_store(schema_dir: Path) -> dict:
    root = schema_dir.resolve()
    store = {}
    for path in sorted(root.glob("*.schema.json")):
        if not path.resolve().is_relative_to(root):
            raise ConfigError("Schema symlink escapes installed directory")
        data = json.loads(path.read_text(encoding="utf-8"))
        Draft7Validator.check_schema(data)
        for node in _schema_nodes(data):
            if "$id" in node or "id" in node:
                raise ConfigError("Schema scope identifiers are not supported")
            if "$ref" in node:
                reference = node["$ref"]
                parsed = urlsplit(reference)
                if parsed.scheme or parsed.netloc or parsed.query:
                    raise ConfigError(f"Non-local schema reference: {reference}")
                if parsed.path and (
                    "/" in parsed.path or "\\" in parsed.path or "%" in parsed.path
                ):
                    raise ConfigError(f"Schema references must name installed files: {reference}")
                if parsed.path:
                    resolve_share_path(root, parsed.path)
        store[path.as_uri()] = data
    for data in store.values():
        for node in _schema_nodes(data):
            if "$ref" not in node:
                continue
            filename = urlsplit(node["$ref"]).path
            if filename and (root / filename).as_uri() not in store:
                raise ConfigError(f"Missing installed schema: {filename}")
    return store


def validate_data(data: Any, schema_id: str, schema_dir: Path, share_root: Path) -> None:
    """Validate a Mapping/projection; does not establish runtime readiness."""
    _check_json_values(data)
    store = _schema_store(schema_dir)
    path = resolve_share_path(schema_dir, schema_id)
    uri = path.as_uri()
    if uri not in store:
        raise ConfigError(f"Unknown installed schema: {schema_id}")

    def relative_path(
        validator: Any, enabled: bool, instance: Any, schema: dict
    ) -> Iterator[ValidationError]:
        if enabled and isinstance(instance, str):
            try:
                resolve_share_path(share_root, instance)
            except ConfigError as error:
                yield ValidationError(str(error))

    validator_type = validators.extend(Draft7Validator, {"x-share-relative-path": relative_path})
    resolver = _InstalledResolver(base_uri=uri, referrer=store[uri], store=store)
    validator_type(store[uri], resolver=resolver).validate(data)


def load_and_validate(path: Path, schema_id: str, schema_dir: Path, share_root: Path) -> Any:
    data = load_yaml(path)
    validate_data(data, schema_id, schema_dir, share_root)
    return data
