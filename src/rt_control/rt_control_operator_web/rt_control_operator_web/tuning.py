"""Field tuning of the chassis steering motion (ELECTRI-127). No ROS imports.

The repository artifact holds the defaults. The console stores operator changes in the
tuning file that ``scope_operator_tuning`` applies at every scope start, and appends
every change to ``history.jsonl`` next to it.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Mapping, Optional

import yaml

# Same location and whitelist as rt_control_bringup.operator_tuning.
TUNING_ROOT = Path("/var/lib/rt-control/tuning")
STEERING_PARAMETERS = (
    "max_steering_slew",
    "max_steering_acceleration",
    "max_steering_jerk",
    "alignment_threshold",
    "align_first_threshold",
    "align_first_release",
)


class TuningError(ValueError):
    pass


class FieldTuning:
    """Tuning file and defaults of one controller in one deployment artifact."""

    def __init__(self, generated_dir: Path, controller: str, root: Path = TUNING_ROOT) -> None:
        self.generated_dir = Path(generated_dir)
        self.controller = controller
        artifact = yaml.safe_load((self.generated_dir / "artifact.yaml").read_text("utf-8"))
        unit = artifact["selection"]["machine_unit"]
        self.path = Path(root) / unit / f"{controller}.yaml"
        self.history = self.path.parent / "history.jsonl"

    def source(self) -> dict:
        try:
            document = yaml.safe_load(self.path.read_text("utf-8")) or {}
            return {"source": "field", "updated_at": str(document.get("updated_at", ""))}
        except FileNotFoundError:
            return {"source": "default", "updated_at": ""}
        except (OSError, yaml.YAMLError) as exc:
            return {"source": "invalid", "updated_at": str(exc)}

    def defaults(self) -> dict:
        try:
            document = yaml.safe_load((self.generated_dir / "controllers.yaml").read_text("utf-8"))
            parameters = document[self.controller]["ros__parameters"]
            return {name: float(parameters[name]) for name in STEERING_PARAMETERS}
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise TuningError(f"读取默认值失败：{exc}") from exc

    def store(self, values_rad: Optional[Mapping[str, float]], action: str) -> None:
        """Write (or remove, when ``values_rad`` is None) the file and log the change."""
        if values_rad is not None and set(values_rad) - set(STEERING_PARAMETERS):
            raise TuningError("only the steering motion parameters can be tuned")
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        previous = None
        if self.path.exists():
            previous = (yaml.safe_load(self.path.read_text("utf-8")) or {}).get("parameters")
        if values_rad is None:
            self.path.unlink(missing_ok=True)
        else:
            document = {
                "schema_version": 1,
                "controller": self.controller,
                "updated_at": stamp,
                "units": "rad, rad/s, rad/s^2, rad/s^3",
                "parameters": {name: float(value) for name, value in values_rad.items()},
            }
            temporary = self.path.with_suffix(".tmp")
            temporary.write_text(yaml.safe_dump(document, sort_keys=False), "utf-8")
            os.replace(temporary, self.path)
        record = {
            "time": stamp,
            "action": action,
            "before": previous,
            "after": dict(values_rad) if values_rad is not None else None,
        }
        with self.history.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
