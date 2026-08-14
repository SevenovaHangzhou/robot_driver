import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.status_adapter import (  # noqa: E402
    BatteryHealthSnapshot,
    CANOPEN_NODE_NAMES,
    ComponentSnapshot,
    ETHERCAT_SLAVE_NAMES,
    PlcHealthSnapshot,
    _diagnostic_level,
    build_safety_summary,
    populate_domain_readiness,
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
    assert message.blockers == ["/robot/rt_control/canopen/node_3: Fault"]
    assert message.errors == []
