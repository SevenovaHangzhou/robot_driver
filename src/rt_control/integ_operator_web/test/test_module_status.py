"""Startup selection and RT health stay distinct; display code sends no commands."""

from types import SimpleNamespace as NS
import yaml
from integ_operator_web.module_status import ModuleStatus


def status(tmp_path):
    rows = [
        dict(
            module="arm",
            component="left",
            label="左臂",
            policy="required",
            present=True,
            selected=True,
            reason="present",
            domain=1,
            joints=["j1"],
            health_interfaces=["domain/data_valid", "domain/process_data_age_ms"],
        ),
        dict(
            module="force",
            component="right",
            label="右腕力",
            policy="optional",
            present=False,
            selected=False,
            reason="absent",
            domain=4,
            joints=[],
            health_interfaces=[],
        ),
    ]
    (tmp_path / "module_selection.yaml").write_text(yaml.safe_dump({"modules": rows}))
    return ModuleStatus(tmp_path, 0.2)


def message(valid=1.0, age=0.01):
    return NS(
        joint_names=["domain"],
        interface_values=[
            NS(interface_names=["data_valid", "process_data_age_ms"], values=[valid, age])
        ],
    )


def test_missing_legacy_metadata_has_no_invented_modules(tmp_path):
    assert ModuleStatus(tmp_path, 0.2).snapshot() == []


def test_absent_stays_skipped_and_selected_waits_for_feedback(tmp_path):
    rows = status(tmp_path).snapshot()
    assert rows[0]["communication"] == "waiting"
    assert rows[1]["reason"] == "absent"
    assert rows[1]["communication"] == "not_loaded"


def test_valid_invalid_and_stale_feedback_are_distinct(tmp_path):
    view = status(tmp_path)
    view.observe(message(), now=1.0)
    assert view.snapshot(now=1.01)[0]["communication"] == "reported_valid"
    assert view.snapshot(now=1.01)[0]["age_ms"] == 0.01
    view.observe(message(valid=0.0), now=1.1)
    assert view.snapshot(now=1.11)[0]["communication"] == "reported_invalid"
    assert view.snapshot(now=1.5)[0]["communication"] == "waiting"
    assert view.snapshot(now=1.5)[1]["reason"] == "absent"


def test_nonfinite_and_partial_samples_are_not_shown_as_valid(tmp_path):
    view = status(tmp_path)
    view.observe(message(age=float("nan")), now=1)
    assert view.snapshot(now=1.01)[0]["communication"] == "waiting"
    view.observe(
        NS(
            joint_names=["domain"],
            interface_values=[NS(interface_names=["data_valid"], values=[1.0])],
        ),
        now=2,
    )
    assert view.snapshot(now=2.01)[0]["communication"] == "waiting"


def test_current_publication_with_old_device_feedback_is_invalid(tmp_path):
    view = status(tmp_path)
    view.observe(message(age=500), now=1)
    assert view.snapshot(now=1.01)[0]["communication"] == "reported_invalid"
