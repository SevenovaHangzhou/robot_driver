"""Apply persisted operator tuning to loaded controllers at scope start (ELECTRI-127).

Repository data stays the source of defaults. The operator console stores field tuning in
/var/lib/rt-control/tuning/<machine_unit>/<controller>.yaml; this step writes those values
through the controller's own parameter service, so its limit checks apply. A rejected or
malformed file leaves the defaults in place and is reported; it never blocks the start.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import yaml

TUNING_ROOT = Path("/var/lib/rt-control/tuning")
# Only these parameters may be tuned in the field; everything else stays repository data.
TUNABLE = {
    "swerve_controller": (
        "max_steering_slew",
        "max_steering_acceleration",
        "max_steering_jerk",
        "alignment_threshold",
    ),
}


def load_tuning(path: Path, controller: str) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if document.get("schema_version") != 1 or document.get("controller") != controller:
        raise ValueError("unexpected tuning file header")
    values = document.get("parameters") or {}
    allowed = TUNABLE[controller]
    unknown = sorted(set(values) - set(allowed))
    if unknown:
        raise ValueError(f"parameters not tunable in the field: {unknown}")
    result = {}
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a number")
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        result[name] = float(value)
    return result


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    runtime = Path(argv[0])
    artifact = yaml.safe_load((runtime / "artifact.yaml").read_text(encoding="utf-8"))
    unit = artifact["selection"]["machine_unit"]
    pending = {}
    for controller in TUNABLE:
        path = TUNING_ROOT / unit / f"{controller}.yaml"
        if not path.is_file():
            continue
        try:
            pending[controller] = load_tuning(path, controller)
        except (OSError, ValueError, yaml.YAMLError) as error:
            print(
                f"RT_SCOPE_TUNING controller={controller} rejected_file={path} reason={error}",
                flush=True,
            )
    if not pending:
        print(f"RT_SCOPE_TUNING none unit={unit}", flush=True)
        return 0

    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    rclpy.init()
    node = rclpy.create_node("scope_operator_tuning")
    try:
        for controller, values in pending.items():
            client = node.create_client(SetParameters, f"/{controller}/set_parameters")
            if not client.wait_for_service(timeout_sec=10.0):
                print(f"RT_SCOPE_TUNING controller={controller} unavailable", flush=True)
                return 1
            request = SetParameters.Request(
                parameters=[
                    Parameter(
                        name=name,
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_DOUBLE, double_value=value
                        ),
                    )
                    for name, value in values.items()
                ]
            )
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            response = future.result()
            if response is None:
                print(f"RT_SCOPE_TUNING controller={controller} timeout", flush=True)
                return 1
            for (name, value), result in zip(values.items(), response.results):
                state = "applied" if result.successful else f"rejected({result.reason})"
                print(
                    f"RT_SCOPE_TUNING controller={controller} {name}={value:.6g} {state}",
                    flush=True,
                )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
