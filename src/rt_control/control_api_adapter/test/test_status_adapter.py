import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.status_adapter import (  # noqa: E402
    BatteryHealthSnapshot,
    CANOPEN_NODE_NAMES,
    ComponentSnapshot,
    DiagnosticState,
    DiagnosticTopology,
    ETHERCAT_MASTER_NAME,
    ETHERCAT_SLAVE_NAMES,
    PlcHealthSnapshot,
    _diagnostic_level,
    build_safety_summary,
    populate_domain_readiness,
    update_diagnostic_state,
    update_diagnostic_topology,
)


def ok_component(name: str, **values: str) -> ComponentSnapshot:
    return ComponentSnapshot(
        name=name,
        level=0,
        fresh=True,
        message="healthy",
        values=values,
    )


def test_safety_summary_requires_enabled_control_and_all_approved_sources() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat_master=ok_component("/robot/rt_control/ethercat/master"),
        ethercat_slaves=[ok_component(name) for name in ETHERCAT_SLAVE_NAMES],
        canopen_nodes=[ok_component(name) for name in CANOPEN_NODE_NAMES],
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert summary.safe_to_start_motion
    assert summary.control_enabled
    assert summary.state == "READY"


def test_safety_summary_reports_canopen_fault() -> None:
    node_2 = ok_component("/robot/rt_control/canopen/node_2")
    node_3 = ComponentSnapshot(
        name="/robot/rt_control/canopen/node_3",
        level=2,
        fresh=True,
        message="Fault",
        values={},
    )
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat_master=ok_component("/robot/rt_control/ethercat/master"),
        ethercat_slaves=[ok_component(name) for name in ETHERCAT_SLAVE_NAMES],
        canopen_nodes=[node_2, node_3],
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert not summary.safe_to_start_motion
    assert not summary.canopen_ok
    assert "/robot/rt_control/canopen/node_3: Fault" in summary.active_faults


@pytest.mark.parametrize(
    ("ethercat_names", "canopen_names", "unhealthy_field"),
    [
        (
            (*ETHERCAT_SLAVE_NAMES[:-1], ETHERCAT_SLAVE_NAMES[0]),
            CANOPEN_NODE_NAMES,
            "ethercat_ok",
        ),
        (
            (*ETHERCAT_SLAVE_NAMES[:-1], "/robot/rt_control/ethercat/slave_99"),
            CANOPEN_NODE_NAMES,
            "ethercat_ok",
        ),
        (
            ETHERCAT_SLAVE_NAMES,
            (CANOPEN_NODE_NAMES[0], CANOPEN_NODE_NAMES[0]),
            "canopen_ok",
        ),
        (
            ETHERCAT_SLAVE_NAMES,
            (CANOPEN_NODE_NAMES[0], "/robot/rt_control/canopen/node_99"),
            "canopen_ok",
        ),
    ],
)
def test_safety_summary_rejects_duplicate_or_unexpected_diagnostic_names(
    ethercat_names: tuple[str, ...],
    canopen_names: tuple[str, ...],
    unhealthy_field: str,
) -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager", state="ENABLED", stage="success"
        ),
        ethercat_master=ok_component("/robot/rt_control/ethercat/master"),
        ethercat_slaves=[ok_component(name) for name in ethercat_names],
        canopen_nodes=[ok_component(name) for name in canopen_names],
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert not getattr(summary, unhealthy_field)
    assert not summary.safe_to_start_motion
    expected_fault = (
        "ethercat_slaves: incomplete diagnostic set"
        if unhealthy_field == "ethercat_ok"
        else "canopen_nodes: incomplete diagnostic set"
    )
    assert expected_fault in summary.active_faults


def test_diagnostic_topology_tracks_authoritative_batches_and_recovers() -> None:
    expected = DiagnosticTopology(ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES)
    observed = update_diagnostic_topology(
        DiagnosticTopology(),
        (ETHERCAT_MASTER_NAME, *ETHERCAT_SLAVE_NAMES, *CANOPEN_NODE_NAMES),
    )
    assert observed == expected

    invalid_groups = (
        (
            ETHERCAT_MASTER_NAME,
            *ETHERCAT_SLAVE_NAMES[:-1],
            ETHERCAT_SLAVE_NAMES[0],
            *CANOPEN_NODE_NAMES,
        ),
        (
            ETHERCAT_MASTER_NAME,
            *ETHERCAT_SLAVE_NAMES[:-1],
            "/robot/rt_control/ethercat/slave_99",
            *CANOPEN_NODE_NAMES,
        ),
        (ETHERCAT_MASTER_NAME, *ETHERCAT_SLAVE_NAMES[:-1], *CANOPEN_NODE_NAMES),
        (
            ETHERCAT_MASTER_NAME,
            ETHERCAT_SLAVE_NAMES[1],
            ETHERCAT_SLAVE_NAMES[0],
            *ETHERCAT_SLAVE_NAMES[2:],
            *CANOPEN_NODE_NAMES,
        ),
        (
            ETHERCAT_MASTER_NAME,
            *ETHERCAT_SLAVE_NAMES,
            CANOPEN_NODE_NAMES[0],
            CANOPEN_NODE_NAMES[0],
        ),
        (
            ETHERCAT_MASTER_NAME,
            *ETHERCAT_SLAVE_NAMES,
            CANOPEN_NODE_NAMES[0],
            "/robot/rt_control/canopen/node_99",
        ),
    )
    for names in invalid_groups:
        invalid = update_diagnostic_topology(observed, names)
        assert invalid != expected, names
        assert update_diagnostic_topology(
            invalid,
            (
                ETHERCAT_MASTER_NAME,
                *ETHERCAT_SLAVE_NAMES,
                *CANOPEN_NODE_NAMES,
            ),
        ) == expected


def test_unrelated_diagnostics_do_not_replace_managed_topology() -> None:
    expected = DiagnosticTopology(ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES)
    assert update_diagnostic_topology(
        expected,
        ("/robot/rt_control/plc", "/some/other/diagnostic"),
    ) == expected


def test_diagnostic_batch_update_replaces_one_immutable_snapshot() -> None:
    old_status = SimpleNamespace(name=ETHERCAT_SLAVE_NAMES[0])
    old = DiagnosticState(
        topology=DiagnosticTopology(ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES),
        components={old_status.name: (old_status, 1.0)},
    )
    new_statuses = tuple(
        SimpleNamespace(name=name)
        for name in (
            ETHERCAT_MASTER_NAME,
            *ETHERCAT_SLAVE_NAMES,
            *CANOPEN_NODE_NAMES,
        )
    )

    updated = update_diagnostic_state(old, new_statuses, received=2.0)

    assert old.components == {old_status.name: (old_status, 1.0)}
    assert updated is not old
    assert updated.topology == DiagnosticTopology(
        ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES
    )
    assert all(updated.components[status.name] == (status, 2.0) for status in new_statuses)
    with pytest.raises(TypeError):
        updated.components["unexpected"] = (SimpleNamespace(name="unexpected"), 2.0)


def test_diagnostic_level_accepts_ros_python_byte_field() -> None:
    assert _diagnostic_level(0) == 0
    assert _diagnostic_level(b"\x00") == 0
    assert _diagnostic_level(b"\x02") == 2
    assert _diagnostic_level(b"\x03") == 3
    assert _diagnostic_level(b"") == 2


def test_domain_readiness_uses_vendored_schema() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat_master=ok_component("/robot/rt_control/ethercat/master"),
        ethercat_slaves=[ok_component(name) for name in ETHERCAT_SLAVE_NAMES],
        canopen_nodes=[
            ok_component("/robot/rt_control/canopen/node_2"),
            ComponentSnapshot(
                "/robot/rt_control/canopen/node_3", 2, True, "Fault", {}
            ),
        ],
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )
    message = SimpleNamespace(
        STATUS_HEALTHY="HEALTHY",
        STATUS_DEGRADED="DEGRADED",
        STATUS_UNAVAILABLE="UNAVAILABLE",
        header=None,
        domain="",
        readiness_name="",
        ready=True,
        status="",
        operational_state="",
        map_version="unexpected",
        producer_instance_id="",
        blockers=[],
        errors=["unexpected"],
    )

    populate_domain_readiness(
        message,
        header="header",
        summary=summary,
        producer_instance_id="instance-1",
    )

    assert message.header == "header"
    assert message.domain == "rt_control"
    assert message.readiness_name == "rt_control"
    assert not message.ready
    assert message.status == message.STATUS_UNAVAILABLE
    assert message.operational_state == "NOT_READY"
    assert message.map_version == ""
    assert message.producer_instance_id == "instance-1"
    assert message.blockers == ["canopen_unavailable"]
    assert message.errors == []


def test_domain_readiness_healthy_state_has_no_blockers_or_errors() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat_master=ok_component("/robot/rt_control/ethercat/master"),
        ethercat_slaves=[ok_component(name) for name in ETHERCAT_SLAVE_NAMES],
        canopen_nodes=[ok_component(name) for name in CANOPEN_NODE_NAMES],
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )
    message = SimpleNamespace(
        STATUS_HEALTHY="HEALTHY",
        STATUS_DEGRADED="DEGRADED",
        STATUS_UNAVAILABLE="UNAVAILABLE",
        blockers=["unexpected"],
        errors=["unexpected"],
    )

    populate_domain_readiness(
        message,
        header="header",
        summary=summary,
        producer_instance_id="instance-1",
    )

    assert message.ready
    assert message.status == message.STATUS_HEALTHY
    assert message.map_version == ""
    assert message.blockers == []
    assert message.errors == []
