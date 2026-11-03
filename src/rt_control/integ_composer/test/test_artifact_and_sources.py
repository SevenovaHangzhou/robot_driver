"""Deploy-time artifacts, declarative parameter sources and the EtherCAT family registry."""

from pathlib import Path
import runpy
import shutil
import sys
import types

import pytest
import yaml

from integ_composer.artifact import load_artifact, runtime_arguments, write_artifact
from integ_composer.compose import compose
from integ_composer.model import ARTIFACT
from integ_composer.parameter_sources import controller_parameters

RT = Path(__file__).resolve().parents[2]
for owner in ("hw_ethercat", "hw_canopen", "hw_can_damiao"):
    sys.path.insert(0, str(RT / owner))
from hw_ethercat import rt_control_renderer as ETHERCAT  # noqa: E402

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
    for name in ("hwcfg_machines", "ctrl_enable_manager", "hw_ethercat"):
        target = tmp_path / "share" / name
        shutil.copytree(RT / name, target, ignore=shutil.ignore_patterns("test", "src"))
        lookup[name] = target
    monkeypatch.setattr(ETHERCAT, "_share", lambda: lookup["hw_ethercat"])
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
    policy = shares["hwcfg_machines"] / "machines/Alfa_V3/policies/Alfa_V3_enable_policy.yaml"
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


@pytest.mark.parametrize("jtc_only", [False, True])
def test_gravity_off_preserves_outputs_and_accepts_old_artifacts(shares, tmp_path, jtc_only):
    output = tmp_path / "arms"
    options = dict(ARMS, jtc_only=jtc_only, package_share=share_of(shares))
    implicit = write_artifact(output, **options)
    explicit = compose(runtime_dir=output, gravity_feedforward=False, **options)
    assert explicit.files == implicit.files
    assert explicit.controller_order == implicit.controller_order
    path = output / ARTIFACT
    manifest = yaml.safe_load(path.read_text())
    assert manifest["selection"].pop("gravity_feedforward") is False
    path.write_text(yaml.safe_dump(manifest))
    assert load_artifact(output, gravity_feedforward=False, **ARMS, jtc_only=jtc_only).files == implicit.files
    manifest["selection"]["gravity_feedforward"] = True
    path.write_text(yaml.safe_dump(manifest))
    with pytest.raises(ValueError, match="selection differs.*gravity_feedforward"):
        load_artifact(output, **{k: v for k, v in options.items() if k != "package_share"})


@pytest.mark.parametrize("use_mock_hardware", [False, True])
@pytest.mark.parametrize("jtc_only", [False, True])
def test_gravity_on_rejects_before_render_or_write(tmp_path, use_mock_hardware, jtc_only):
    output = tmp_path / "not_created" / "arms"

    def no_share(_):
        pytest.fail("Unqualified gravity request reached an owner")

    options = dict(
        ARMS, use_mock_hardware=use_mock_hardware, jtc_only=jtc_only,
        gravity_feedforward=True, package_share=no_share,
    )
    with pytest.raises(ValueError, match="D-004"):
        compose(runtime_dir=output, **options)
    with pytest.raises(ValueError, match="D-004"):
        write_artifact(output, **options)
    with pytest.raises(ValueError, match="D-004"):
        load_artifact(output, **{k: v for k, v in options.items() if k != "package_share"})
    assert not output.parent.exists()


def test_rejected_gravity_selection_preserves_existing_artifact(shares, tmp_path):
    output = tmp_path / "arms"
    options = dict(ARMS, package_share=share_of(shares))
    write_artifact(output, **options)
    before = {p.relative_to(output): p.read_bytes() for p in output.rglob("*") if p.is_file()}
    with pytest.raises(ValueError, match="D-004"):
        write_artifact(output, gravity_feedforward=True, **options)
    assert {p.relative_to(output): p.read_bytes() for p in output.rglob("*") if p.is_file()} == before


@pytest.mark.parametrize("value", ["false", "true", 0, 1, None])
def test_gravity_selection_requires_boolean(tmp_path, value):
    with pytest.raises(ValueError, match="gravity_feedforward must be a boolean"):
        compose(runtime_dir=tmp_path, gravity_feedforward=value, **ARMS)


def test_gravity_cli_reports_admission_failure(tmp_path, capsys):
    main = runpy.run_path(str(RT / "integ_composer/scripts/compose_artifact"))["main"]
    output = tmp_path / "arms"
    assert main([
        "--output", str(output), "--robot-variant", "Alfa_V3",
        "--machine-unit", "Alfa_V3-01", "--physical-profile", "arms_only",
        "--control-scope", "arms_only", "--mock", "--gravity-feedforward",
    ]) == 1
    assert "D-004" in capsys.readouterr().err
    assert not output.exists()


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
    module = types.ModuleType("hw_ethercat.synthetic_renderer")
    module.render = lambda request, *args: calls.append(request["machine"]["spec"]["family"]) or {}
    module.preflight = lambda topology, run=None: calls.append(topology["kind"])
    monkeypatch.setitem(sys.modules, module.__name__, module)
    path = shares["hw_ethercat"] / "config/profiles/renderer_families.yaml"
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
    profile = shares["hw_ethercat"] / "config/profiles/ethercat_master.yaml"
    values = yaml.safe_load(profile.read_text())
    values["preload_timeout_ms"] = 6000
    profile.write_text(yaml.safe_dump(values))
    composed = write_artifact(tmp_path / "arms", package_share=share_of(shares), **ARMS)
    assert '<param name="preload_timeout_ms">6000</param>' in composed.robot_description
    values["preload_timeout_ms"] = 0
    profile.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="positive integer"):
        write_artifact(tmp_path / "arms", package_share=share_of(shares), **ARMS)
