"""Only the approved representation differences are canonicalized for equivalence."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _rename(value, runtime_dir, rename_map):
    if isinstance(value, str):
        value = value.replace(str(runtime_dir), "<RUNTIME_DIR>")
        return rename_map.get(value, value)
    if isinstance(value, dict):
        return {key: _rename(child, runtime_dir, rename_map) for key, child in value.items()}
    if isinstance(value, list):
        return [_rename(child, runtime_dir, rename_map) for child in value]
    return value


def canonical_xml(text, runtime_dir, rename_map):
    # Parse placeholders after the old raw output's XML escaping, then map complete
    # attribute/text values. Child order remains untouched.
    root = ET.fromstring(text)
    for element in root.iter():
        element.attrib.update(
            {key: _rename(value, runtime_dir, rename_map) for key, value in element.attrib.items()}
        )
        if element.text is not None:
            element.text = _rename(element.text, runtime_dir, rename_map)
    return ET.canonicalize(ET.tostring(root, encoding="unicode"))


def canonical_yaml(text, runtime_dir, rename_map):
    return yaml.safe_dump(_rename(yaml.safe_load(text), runtime_dir, rename_map), sort_keys=True)


def canonical_files(directory, runtime_dir, rename_map):
    result = {}
    for path in sorted(Path(directory).rglob("*")):
        if not path.is_file() or path.suffix not in (".yaml", ".xml"):
            continue
        name = path.relative_to(directory).as_posix()
        name = rename_map.get(name, name)
        text = path.read_text()
        result[name] = (
            canonical_xml(text, runtime_dir, rename_map)
            if path.suffix == ".xml"
            else canonical_yaml(text, runtime_dir, rename_map)
        )
    return result
