"""Deploy-time artifacts, declarative parameter sources and the EtherCAT family registry."""

from pathlib import Path
import shutil
import sys
import types

import pytest
import yaml

from rt_control_composer.artifact import load_artifact, runtime_arguments, write_artifact
from rt_control_composer.model import ARTIFACT
from rt_control_composer.parameter_sources import controller_parameters

RT = Path(__file__).resolve().parents[2]
for owner in ("robot_hw_ethercat", "robot_hw_canopen", "robot_hw_can"):
    sys.path.insert(0, str(RT / owner))
from robot_hw_ethercat import rt_control_renderer as ETHERCAT  # noqa: E402

ARMS = dict(
    robot_variant="Alfa_V3",
    machine_unit="Alfa_V3-01",
    physical_profile="arms_only",
    control_scope="arms_only",
    use_mock_hardware=True,
)
FORCE = dict(ARMS, physical_profile="force_bench", control_scope="force_bench")


@pytest.fixture
def shares(tmp_path, monkeypatch):
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["robot_description"] = RT.parent / "description/robot_description"
    for name in ("rt_control_machine_config", "enable_manager", "robot_hw_ethercat"):
        target = tmp_path / "share" / name
        shutil.copytree(RT / name, target, ignore=shutil.ignore_patterns("test", "src"))
        lookup[name] = target
    monkeypatch.setattr(ETHERCAT, "_share", lambda: lookup["robot_hw_ethercat"])
    return lookup


def share_of(shares):
    return lambda name: str(shares[name])


def test_artifact_round_trip_reads_without_writing(shares, tmp_path, monkeypatch):
    output = tmp_path / "generated" / "arms"
    written = write_artifact(output, package_share=share_of(shares), **ARMS)
    on_disk = {
        path.relative_to(output).as_posix(): path.read_text()
        for path in output.rglob("*")
        if path.is_file() and path.name != ARTIFACT
    }
    assert on_disk == written.files
    assert not list(output.parent.glob(".arms.*")), "staging/previous directories must not remain"

    def denied(*args, **kwargs):
        pytest.fail("Launch-time load attempted a write")

    monkeypatch.setattr(Path, "write_text", denied)
    monkeypatch.setattr(Path, "mkdir", denied)
    loaded = load_artifact(output, **ARMS)
    assert loaded.files == written.files
    assert Path(loaded.runtime_dir) == output
    assert loaded.robot_description == written.robot_description
    assert loaded.controllers == written.controllers
    assert loaded.controller_order == written.controller_order
    assert loaded.runtime_parameters == written.runtime_parameters
    assert loaded.lifecycle == written.lifecycle
    assert loaded.hardware_activation_order == written.hardware_activation_order
    assert loaded.staged_hardware_activation == written.staged_hardware_activation
    assert [s["result"]["expected_topology"] for s in loaded.systems] == [
        s["result"]["expected_topology"] for s in written.systems
    ]


@pytest.mark.parametrize("damage", ["modified", "extra", "missing_manifest"])
def test_damaged_artifact_is_rejected(shares, tmp_path, damage):
    output = tmp_path / "arms"
    write_artifact(output, package_share=share_of(shares), **ARMS)
    if damage == "modified":
        (output / "controllers.yaml").write_text("controller_manager: {}\n")
        match = "modified"
    elif damage == "extra":
        (output / "unexpected.yaml").write_text("{}\n")
        match = "file set differs"
    else:
        (output / ARTIFACT).unlink()
        match = "compose_artifact"
    with pytest.raises(ValueError, match=match):
        load_artifact(output, **ARMS)


def test_selection_mismatch_and_older_schema_are_rejected(shares, tmp_path):
    output = tmp_path / "arms"
    write_artifact(output, package_share=share_of(shares), **ARMS)
    with pytest.raises(ValueError, match="selection differs in \\['jtc_only'\\]"):
        load_artifact(output, **ARMS, jtc_only=True)
    manifest = yaml.safe_load((output / ARTIFACT).read_text())
    manifest["schema_version"] = 1
    (output / ARTIFACT).write_text(yaml.safe_dump(manifest))
    with pytest.raises(ValueError, match="Unsupported composition artifact.*compose_artifact"):
        load_artifact(output, **ARMS)


def test_load_uses_the_artifact_as_written_without_recomposing(shares, tmp_path):
    output = tmp_path / "arms"
    written = write_artifact(output, package_share=share_of(shares), **ARMS)
    # Installed configuration changes take effect only after the artifact is regenerated.
    policy = shares["rt_control_machine_config"] / "machines/Alfa_V3/policies/Alfa_V3_enable_policy.yaml"
    policy.write_text(policy.read_text().replace("batch_timeout: 4.0", "batch_timeout: 5.0"))
    assert load_artifact(output, **ARMS).files == written.files


def test_artifact_paths_and_replacement_rules(shares, tmp_path):
    with pytest.raises(ValueError, match="absolute"):
        write_artifact("relative/arms", package_share=share_of(shares), **ARMS)
    occupied = tmp_path / "occupied"
    occupied.mkdir()
    (occupied / "keep.txt").write_text("user data")
    with pytest.raises(ValueError, match="not an artifact"):
        write_artifact(occupied, package_share=share_of(shares), **ARMS)
    assert (occupied / "keep.txt").read_text() == "user data"
    output = tmp_path / "arms"
    write_artifact(output, package_share=share_of(shares), **ARMS)
    write_artifact(output, package_share=share_of(shares), **ARMS, jtc_only=True)
    assert yaml.safe_load((output / ARTIFACT).read_text())["selection"]["jtc_only"] is True


def test_per_start_values_stay_out_of_the_artifact(shares, tmp_path):
    output = tmp_path / "force"
    composed = write_artifact(output, package_share=share_of(shares), **FORCE)
    stored = yaml.safe_load((output / "controllers.yaml").read_text())
    names = sorted(composed.runtime_parameters)
    assert len(names) == 2
    assert all("startup_id" not in stored[name]["ros__parameters"] for name in names)
    arguments = runtime_arguments(composed, {"startup_id": "bench-1"})
    assert arguments == ["--ros-args"] + [
        item for name in names for item in ("-p", f"{name}:startup_id:=bench-1")
    ]
    with pytest.raises(ValueError, match="Missing runtime value startup_id"):
        runtime_arguments(composed, {})


def environment(**overrides):
    base = {
        "context": {"mode": "normal", "controller": "new_controller"},
        "policies": {"tuning": {"gain": 2.0, "views": {"normal": {"limit": 1.5}}}},
        "policy_refs": {},
        "package_share": None,
        "rendered_modules": {},
        "selected": ["new_controller", "helper"],
        "tags": {"new_controller": ["x"], "helper": ["peer"]},
        "naming": {},
        "runtime_values": {},
        "runtime_parameters": {},
        "runtime_dir": "/synthetic",
        "runtime_files": {},
        "managed_joints": [],
    }
    return {**base, **overrides}


def test_new_controller_type_is_data_only():
    binding = {
        "module": "m",
        "parameters": [
            {"policy": {"name": "tuning", "path": "gain", "as": "gain"}},
            {"policy": {"name": "tuning", "path": "views.{mode}"}},
            {"controllers": {"tag": "peer", "select": "first", "as": "peer"}},
            {"when_policy": "absent", "policy": {"name": "absent", "path": "x", "as": "x"}},
        ],
    }
    result = controller_parameters("new_controller", binding, environment())
    assert result == {"gain": 2.0, "limit": 1.5, "peer": "helper"}


@pytest.mark.parametrize(
    "source, match",
    [
        ({"policy": {"name": "absent", "path": "x", "as": "x"}}, "unselected policy"),
        ({"policy": {"name": "tuning", "path": "missing", "as": "x"}}, "Missing parameter"),
        ({"controllers": {"tag": "none", "select": "first", "as": "x"}}, "No selected"),
        ({"policy": {"name": "tuning", "path": "gain"}}, "needs 'as'"),
        ({"policy": {"name": "tuning"}, "yaml": {}}, "exactly one kind"),
    ],
)
def test_parameter_source_rejections(source, match):
    with pytest.raises(ValueError, match=match):
        controller_parameters(
            "new_controller", {"module": "m", "parameters": [source]}, environment()
        )


def test_two_sources_for_one_parameter_are_rejected():
    duplicate = {"policy": {"name": "tuning", "path": "gain", "as": "limit"}}
    binding = {
        "module": "m",
        "parameters": [{"policy": {"name": "tuning", "path": "views.{mode}"}}, duplicate],
    }
    with pytest.raises(ValueError, match="Duplicate parameter source"):
        controller_parameters("new_controller", binding, environment())


def test_ethercat_family_is_selected_by_registry_data(shares, monkeypatch):
    calls = []
    module = types.ModuleType("robot_hw_ethercat.synthetic_renderer")
    module.render = lambda request, *args: calls.append(request["machine"]["spec"]["family"]) or {}
    module.preflight = lambda topology, run=None: calls.append(topology["kind"])
    monkeypatch.setitem(sys.modules, module.__name__, module)
    path = shares["robot_hw_ethercat"] / "config/profiles/renderer_families.yaml"
    registry = yaml.safe_load(path.read_text())
    request = {"machine": {"spec": {"family": "synthetic"}}}
    with pytest.raises(ValueError, match="Unregistered EtherCAT renderer family"):
        ETHERCAT.render(request)
    registry["families"]["synthetic"] = {"module": "synthetic_renderer", "module_schema": "x"}
    registry["preflight"]["synthetic_kind"] = "synthetic_renderer"
    path.write_text(yaml.safe_dump(registry))
    ETHERCAT.render(request)
    ETHERCAT.preflight({"kind": "synthetic_kind"})
    assert calls == ["synthetic", "synthetic_kind"]
    with pytest.raises(ValueError, match="Unregistered EtherCAT preflight kind"):
        ETHERCAT.preflight({"kind": "unknown"})


def test_master_timeouts_come_from_the_protocol_profile(shares, tmp_path):
    profile = shares["robot_hw_ethercat"] / "config/profiles/ethercat_master.yaml"
    values = yaml.safe_load(profile.read_text())
    values["preload_timeout_ms"] = 6000
    profile.write_text(yaml.safe_dump(values))
    composed = write_artifact(tmp_path / "arms", package_share=share_of(shares), **ARMS)
    assert '<param name="preload_timeout_ms">6000</param>' in composed.robot_description
    values["preload_timeout_ms"] = 0
    profile.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="positive integer"):
        write_artifact(tmp_path / "arms", package_share=share_of(shares), **ARMS)
