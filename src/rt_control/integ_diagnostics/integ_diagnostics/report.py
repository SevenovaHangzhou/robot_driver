"""Self-check report file: stage A (launch, before any node) then stage B (diagnostics node)."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile


def state_directory():
    return Path(os.environ.get("RT_CONTROL_STATE_DIR", Path(tempfile.gettempdir()) / "rt-control"))


def write_self_check(control_scope, generated_dir, entries, directory=None, stage="before_bus"):
    """Latest self-check of a scope (stage A, then B); the operator web reads it."""
    root = (Path(directory) if directory is not None else state_directory()) / "self_check"
    root.mkdir(parents=True, exist_ok=True)
    report = {
        "schema_version": 1,
        "stage": stage,
        "control_scope": control_scope,
        "generated_dir": str(generated_dir),
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "passed": not any(entry["blocking"] for entry in entries),
        "entries": entries,
    }
    path = root / f"{control_scope}.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=1))
    temporary.replace(path)
    return path
