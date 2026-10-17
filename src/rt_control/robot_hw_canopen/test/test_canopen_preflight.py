"""CAN owner preflight and bundle generation are offline; no CAN frame is sent."""

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from robot_hw_canopen.bundle import generate
from robot_hw_canopen.rt_control_renderer import preflight

SOURCE = "options: {dcf_path: BUNDLE}\nmaster: {node_id: 127}\nsteering_encoder_1: {node_id: 11, dcf: steering_encoder_1.eds}\n"


def link(flags, state):
    return json.dumps([{"flags": flags, "linkinfo": {"info_data": {"state": state}}}])


def topology(bundle, source=SOURCE):
    return {
        "node_ids": [11],
        "can_interface": "can0",
        "can_usb_serial": None,
        "bundle_dir": str(bundle),
        "bus_sha256": hashlib.sha256(source.encode()).hexdigest(),
        "preflight_required": True,
    }


def bundle(tmp_path, source=SOURCE):
    path = tmp_path / "bundle"
    path.mkdir()
    for name in ("bus.yml", "master.dcf", "master.bin"):
        (path / name).write_text("x")
    (path / "source.sha256").write_text(hashlib.sha256(source.encode()).hexdigest() + "\n")
    return path


def test_up_error_active_interface_with_current_bundle_is_admitted(tmp_path):
    calls = []
    preflight(
        topology(bundle(tmp_path)),
        run=lambda argv, **_: calls.append(argv) or link(["UP"], "ERROR-ACTIVE"),
    )
    assert calls == [["ip", "-details", "-json", "link", "show", "can0"]]


@pytest.mark.parametrize(
    "flags,state,message",
    [(["NO-CARRIER"], "ERROR-ACTIVE", "not up"), (["UP"], "BUS-OFF", "not error-active")],
)
def test_down_or_bus_off_interface_is_rejected(tmp_path, flags, state, message):
    with pytest.raises(RuntimeError, match=message):
        preflight(topology(bundle(tmp_path)), run=lambda argv, **_: link(flags, state))


def test_stale_or_missing_bundle_is_rejected_before_any_command(tmp_path):
    path = bundle(tmp_path, source="older source")
    with pytest.raises(RuntimeError, match="stale"):
        preflight(topology(path), run=lambda *_: pytest.fail("ran"))
    (path / "master.dcf").unlink()
    with pytest.raises(RuntimeError, match="incomplete"):
        preflight(topology(path), run=lambda *_: pytest.fail("ran"))


def test_mock_topology_runs_no_command(tmp_path):
    preflight(
        {**topology(tmp_path), "preflight_required": False}, run=lambda *_: pytest.fail("ran")
    )


def test_bundle_generation_runs_dcfgen_at_the_final_path(tmp_path):
    output = tmp_path / "deploy/bundle"
    artifact = tmp_path / "artifact"
    artifact.mkdir()
    eds = tmp_path / "encoder.eds"
    eds.write_text("[FileInfo]\n")
    source = SOURCE.replace("BUNDLE", str(output))
    (artifact / "canopen_bus.yaml").write_text(source)
    (artifact / "canopen_sources.yaml").write_text(yaml.safe_dump({"steering_encoder_1": str(eds)}))
    fake = tmp_path / "dcfgen"
    fake.write_text(
        "#!/usr/bin/env python3\nimport sys, pathlib\n"
        "out = pathlib.Path(sys.argv[sys.argv.index('-d') + 1])\n"
        "assert pathlib.Path(sys.argv[-1]).read_text().count(str(out)) >= 1\n"
        "(out / 'master.dcf').write_text('[DeviceInfo]\\n')\n"
    )
    fake.chmod(0o755)
    assert generate(artifact, dcfgen=str(fake)) == output
    assert (output / "bus.yml").read_text() == source
    assert (output / "steering_encoder_1.eds").read_text() == "[FileInfo]\n"
    assert (output / "master.bin").read_bytes() == b"\0\0\0\0"
    assert (output / "source.sha256").read_text().strip() == hashlib.sha256(source.encode()).hexdigest()
    assert Path(output).is_dir()
