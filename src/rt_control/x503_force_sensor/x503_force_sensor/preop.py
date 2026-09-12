"""One PREOP-only X503 unit readback, completed before hardware activation."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import time
from typing import Callable, Sequence

import yaml


class SnapshotError(ValueError):
    """A device parameter snapshot is unavailable or invalid."""


class PreopRequired(SnapshotError):
    """Reading is forbidden unless the master is idle and the sensor is PREOP."""


@dataclass(frozen=True)
class SensorReadSpec:
    sensor_name: str
    slave_position: int
    vendor_id: int
    product_code: int
    revision: int


def load_sensor_spec(name: str, position: int, profile: Path) -> SensorReadSpec:
    document = yaml.safe_load(profile.read_text(encoding="utf-8"))
    try:
        return SensorReadSpec(name, position, document["vendor_id"],
                              document["product_id"], document["metadata"]["revision_id"])
    except (KeyError, TypeError) as error:
        raise SnapshotError(f"Incomplete X503 identity in {profile}") from error


def _run(argv: list[str]) -> str:
    try:
        result = subprocess.run(argv, stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=2, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise SnapshotError(f"Cannot execute EtherCAT read: {error}") from error
    if result.returncode or result.stderr.strip():
        raise SnapshotError(result.stderr.strip()[:400] or "EtherCAT read failed")
    return result.stdout


def _policy(config: dict) -> dict[str, str]:
    try:
        readback = config["read_only_sdo"]
        if (config["schema_version"] != 1 or readback["index"] != 0x8005
                or readback["decimal_subindices"] != list(range(6, 12))
                or readback["unit_subindices"] != list(range(12, 18))
                or readback["expected_unit_codes"] != [5, 5, 5, 7, 7, 7]
                or config["engineering_unit_contract"] != "force_N_torque_Nm"
                or config["validity_policy"] != "sample_codes_in_range"
                or config["valid_sample_codes"] != []):
            raise SnapshotError("Unsupported X503 readback contract or SDO whitelist")
        lower, upper = config["sample_code_min"], config["sample_code_max"]
        if (type(lower) is not int or type(upper) is not int
                or not -2147483648 <= lower <= upper <= 2147483647):
            raise SnapshotError("Invalid sample-code limits")
    except (KeyError, TypeError) as error:
        raise SnapshotError("Incomplete X503 readback configuration") from error
    return {"engineering_unit_contract": "force_N_torque_Nm",
            "validity_policy": "sample_codes_in_range",
            "sample_code_min": str(lower), "sample_code_max": str(upper)}


def _field(text: str, name: str) -> str:
    matches = re.findall(r"^\s*" + re.escape(name) + r":\s*(.*?)\s*$", text, re.MULTILINE)
    if len(matches) != 1:
        raise PreopRequired(f"Missing or ambiguous EtherCAT {name}")
    return matches[0]


def _guard(spec: SensorReadSpec, responders: int, master_index: int,
           run: Callable[[list[str]], str]) -> dict[str, int]:
    prefix = ["ethercat", "-m", str(master_index)]
    try:
        master = run([*prefix, "master"])
        if (_field(master, "Phase") != "Idle" or _field(master, "Active") != "no"
                or _field(master, "Slaves") != str(responders)
                or not re.search(r"^\s*Link:\s*UP\s*$", master, re.MULTILINE)):
            raise PreopRequired("X503 SDO access requires Idle/Inactive master and complete link")
        slave = run([*prefix, "slaves", "-p", str(spec.slave_position), "-v"])
        if _field(slave, "State") != "PREOP" or _field(slave, "Flag") != "+":
            raise PreopRequired(f"X503 {spec.sensor_name} is not error-free PREOP")
        identity = {key: int(_field(slave, label), 16) for key, label in (
            ("vendor_id", "Vendor Id"), ("product_code", "Product code"),
            ("revision", "Revision number"), ("serial", "Serial number"))}
        if (identity["vendor_id"], identity["product_code"], identity["revision"]) != (
                spec.vendor_id, spec.product_code, spec.revision):
            raise PreopRequired(f"X503 {spec.sensor_name} identity does not match its profile")
        return identity
    except PreopRequired:
        raise
    except (SnapshotError, ValueError) as error:
        raise PreopRequired(f"Cannot verify PREOP before X503 SDO access: {error}") from error


def _uint32(text: str) -> int:
    match = re.fullmatch(r"0x([0-9a-fA-F]{8})\s+([0-9]+)\s*", text)
    if match is None or int(match[1], 16) != int(match[2]):
        raise SnapshotError("X503 uint32 response is incomplete or inconsistent")
    return int(match[2])


def read_preop_snapshot(
    specs: Sequence[SensorReadSpec], config: dict, *, startup_id: str,
    expected_responders: int, run: Callable[[list[str]], str] = _run,
    master_index: int = 0, mock: bool = False,
) -> dict:
    """No retries or persistent fallback; one arm's read error stays invalid.

    State/identity uncertainty aborts initialization. Each read is guarded and
    all reads finish synchronously before the launch creates control processes.
    """
    policy = _policy(config)
    if (not isinstance(startup_id, str) or not startup_id.strip()
            or type(expected_responders) is not int or expected_responders < 1
            or type(master_index) is not int or not 0 <= master_index <= 65535):
        raise SnapshotError("Invalid startup snapshot identity")
    names, positions = set(), set()
    for spec in specs:
        if (not spec.sensor_name or spec.sensor_name != spec.sensor_name.strip()
                or spec.sensor_name in names or spec.slave_position in positions
                or type(spec.slave_position) is not int or not 0 <= spec.slave_position < expected_responders
                or any(type(v) is not int or not 0 <= v <= 0xFFFFFFFF
                       for v in (spec.vendor_id, spec.product_code, spec.revision))):
            raise SnapshotError("Invalid or duplicate X503 sensor specification")
        names.add(spec.sensor_name)
        positions.add(spec.slave_position)
    identities = {} if mock else {
        spec.sensor_name: _guard(spec, expected_responders, master_index, run) for spec in specs}
    snapshot = {"schema_version": 1, "startup_id": startup_id,
                "source": "mock" if mock else "preop_sdo", "captured_at_ns": time.time_ns(),
                "sensors": []}
    for spec in specs:
        values = {**policy, "slave_position": str(spec.slave_position), "snapshot_valid": "false"}
        record = {"sensor_name": spec.sensor_name, "slave_position": spec.slave_position,
                  "identity": identities.get(spec.sensor_name), "values": values, "error": ""}
        snapshot["sensors"].append(record)
        if mock:
            record["error"] = "Mock hardware has no verified device readback"
            continue
        decimals, units = [], []
        try:
            for channel in range(6):
                results = []
                for subindex in (6 + channel, 12 + channel):
                    if _guard(spec, expected_responders, master_index, run) != identities[spec.sensor_name]:
                        raise PreopRequired("X503 identity changed during initialization")
                    results.append(_uint32(run([
                        "ethercat", "-m", str(master_index), "upload", "-p", str(spec.slave_position),
                        "--type", "uint32", "0x8005", str(subindex)])))
                decimal, unit = results
                if decimal > 10 or unit != (5 if channel < 3 else 7):
                    raise SnapshotError(f"Unexpected unit or decimal on channel {channel + 1}")
                decimals.append(decimal)
                units.append(unit)
            values.update(snapshot_valid="true")
            values.update({f"decimal_{i + 1}": str(v) for i, v in enumerate(decimals)})
            values.update({f"unit_{i + 1}": str(v) for i, v in enumerate(units)})
        except PreopRequired:
            raise
        except SnapshotError as error:
            record["error"] = str(error)[:400]
    if not mock:
        for spec in specs:
            if _guard(spec, expected_responders, master_index, run) != identities[spec.sensor_name]:
                raise PreopRequired("X503 identity changed before snapshot completion")
    return snapshot
