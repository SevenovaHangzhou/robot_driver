"""Head and passive sensor equivalence; real golden facts are test-only."""

from hashlib import sha256
from pathlib import Path
import shutil
import sys

import pytest
import yaml

from rt_control_composer.canonical import canonical_files
from rt_control_composer.compose import compose
from rt_control_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
GOLDEN = PACKAGE / "test/golden"
MANIFEST = load_yaml(GOLDEN / "manifest.yaml")
for owner in ("robot_hw_can", "robot_hw_ethercat"):
    sys.path.insert(0, str(RT / owner))
from robot_hw_can import rt_control_renderer as HEAD
from robot_hw_ethercat import rt_control_renderer as FORCE


@pytest.fixture
def shares(tmp_path, monkeypatch):
    data = tmp_path / "data"
    shutil.copytree(RT / "rt_control_machine_config", data)
    monkeypatch.setattr(HEAD, "_share", lambda: RT / "robot_hw_can")
    monkeypatch.setattr(FORCE, "_share", lambda: RT / "robot_hw_ethercat")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["rt_control_machine_config"] = data
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def materialize(result):
    """Compose is pure; the tests write its files to compare with frozen output trees."""
    for name, text in result.files.items():
        path = Path(result.runtime_dir) / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return result


def arguments(shares, family):
    deployment = next((shares["rt_control_machine_config"] / "units").glob("*/deployment.yaml"))
    variant = load_yaml(deployment)["robot_variant"]
    scope = "head_only" if family == "head" else "force_bench"
    return dict(
        robot_variant=variant,
        machine_unit=deployment.parent.name,
        physical_profile=scope,
        control_scope=scope,
        package_share=lambda name: str(shares[name]),
    )


@pytest.mark.parametrize(
    "case", [name for name in MANIFEST["cases"] if name.startswith(("head-", "force-"))]
)
def test_four_frozen_head_force_cases(shares, tmp_path, case):
    family = case.split("-")[0]
    facts = MANIFEST["cases"][case]
    target = GOLDEN / case
    assert {
        path.relative_to(target).as_posix(): sha256(path.read_bytes()).hexdigest()
        for path in target.rglob("*")
        if path.is_file()
    } == facts["outputs"]
    args = arguments(shares, family)
    if family == "head":
        config = load_yaml(GOLDEN / "head_config.yaml")
        data = shares["rt_control_machine_config"]
        path = data / "machines" / args["robot_variant"] / "head_gimbal.yaml"
        module = load_yaml(path)
        module["verified"] = True
        for joint, entry in zip(module["spec"]["joints"], config["joints"]):
            for key in entry:
                if key not in ("name", "min", "max"):
                    joint[key] = entry[key]
        path.write_text(yaml.safe_dump(module, sort_keys=False))
        # Baseline head-real uses test limits -1/+1 for pitch. This override is
        # isolated in the test model share and never changes production model facts.
        model = tmp_path / "model"
        shutil.copytree(shares["robot_description"], model)
        limits = model / "config/joint_limits_gripper.yaml"
        values = yaml.safe_load(limits.read_text())
        root = values["joints"]
        for joint in config["joints"]:
            root[joint["name"]]["lower_position_rad"] = joint["min"]
            root[joint["name"]]["upper_position_rad"] = joint["max"]
        limits.write_text(yaml.safe_dump(values))
        shares["robot_description"] = model
    runtime = tmp_path / "output"
    result = materialize(compose(
        **args,
        runtime_dir=runtime,
        use_mock_hardware=facts["use_mock_hardware"],
        sync_mode=facts["sync_mode"],
        startup_id=facts["startup_id"],
    ))
    snapshot = tmp_path / "snapshot"
    for path in runtime.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(runtime)
        destination = (
            snapshot / relative
            if relative.as_posix()
            in {"robot_description.xml", "controllers.yaml", "controller_order.yaml"}
            else snapshot / "runtime" / relative
        )
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, destination)
    expected = canonical_files(target, "<RUNTIME_DIR>", MANIFEST["rename_map"])
    actual = canonical_files(snapshot, runtime, MANIFEST["rename_map"])
    assert set(expected) == set(actual)
    assert actual == expected, [name for name in expected if actual[name] != expected[name]]
    assert load_yaml(runtime / "controllers.yaml") == result.controllers


def test_head_confirmation_is_recorded_and_unconfirmed_data_is_rejected(shares, tmp_path):
    path = (
        shares["rt_control_machine_config"]
        / "machines"
        / arguments(shares, "head")["robot_variant"]
        / "head_gimbal.yaml"
    )
    values = load_yaml(path)
    assert values["verified"] is True and values["authorities"]["confirmation_person"] == "user"
    values["verified"] = False
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="D-b"):
        compose(**arguments(shares, "head"), runtime_dir=tmp_path / "real", use_mock_hardware=False)
    assert not (tmp_path / "real").exists()
    compose(**arguments(shares, "head"), runtime_dir=tmp_path / "mock")


def test_force_compose_never_runs_read_only_cli(shares, tmp_path, monkeypatch):
    def denied(*args, **kwargs):
        pytest.fail("Offline compose accessed ethercat CLI")

    from robot_hw_ethercat import force_renderer

    monkeypatch.setattr(force_renderer.subprocess, "run", denied)
    compose(
        **arguments(shares, "force"),
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
        sync_mode="dc",
    )


def test_force_preflight_uses_recorded_cli_and_rejects_mismatch(shares, tmp_path):
    from rt_control_composer.compose import preflight

    result = compose(
        **arguments(shares, "force"),
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
        sync_mode="dc",
    )
    cli = load_yaml(GOLDEN / "force_cli.yaml")
    calls = []

    def run(command):
        calls.append(command)
        return cli[" ".join(command)]

    preflight(result, run=run)
    assert len(calls) == 5 and all(command[1] in {"master", "slaves", "pdos"} for command in calls)
    cli["ethercat master -m 0"] = "Phase: Operation\nActive: yes\nLink: UP\nSlaves: 6\n"
    with pytest.raises(ValueError, match="idle master"):
        preflight(result, run=run)


def test_head_sample_c_adapter_is_offline(shares, tmp_path, monkeypatch):
    from rt_control_composer.renderer_registry import render_checked

    data = shares["rt_control_machine_config"]
    args = arguments(shares, "head")
    manifest = load_yaml(data / "machines" / args["robot_variant"] / "manifest.yaml")
    descriptor = manifest["modules"]["head_gimbal"]
    request = {
        "module": descriptor,
        "machine": load_yaml(data / descriptor["config_file"]),
        "devices": {path: load_yaml(data / path) for path in descriptor["device_refs"]},
        "unit": load_yaml(next((data / "units").glob("*/head_zero.yaml"))),
        "deployment": load_yaml(next((data / "units").glob("*/deployment.yaml"))),
        "robot_model": {
            "joint_limits": yaml.safe_load(
                (shares["robot_description"] / "config/joint_limits_gripper.yaml").read_text()
            )
        },
        "use_mock_hardware": True,
        "bus_cycle_hz": 1000,
        "runtime_dir": str(tmp_path),
        "render_mode": "normal",
    }
    sample = load_yaml(PACKAGE / "test/fixtures/sample_c/machines/SyntheticC/can_fixture.yaml")
    request["machine"]["robot_variant"] = sample["robot_variant"]

    def denied(*args, **kwargs):
        pytest.fail("Protocol renderer attempted filesystem write")

    monkeypatch.setattr(Path, "write_text", denied)
    result = render_checked(HEAD, request, PACKAGE / "schema", data)
    assert len(result["joints"]) == 2 and len(result["gpios"]) == 1
    assert not result["runtime_files"]
    request["machine"]["spec"]["joints"][1]["can_id"] = request["machine"]["spec"]["joints"][0][
        "can_id"
    ]
    with pytest.raises(ValueError, match="unique"):
        HEAD.render(request)
