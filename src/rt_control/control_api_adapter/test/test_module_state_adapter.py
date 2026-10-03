import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.module_state_adapter import (  # noqa: E402
    IMPLEMENTATION_NOT_ADMITTED,
    IMPLEMENTATION_NOT_IMPLEMENTED,
    INSTALLATION_NOT_INSTALLED,
    ProtectiveStopCore,
    require_edge_preview_only,
    static_deferred_modules,
)


def test_draft_cannot_enable_unwired_edge_protection() -> None:
    require_edge_preview_only(False)
    with pytest.raises(ValueError, match="not implemented"):
        require_edge_preview_only(True)


def test_deferred_and_uninstalled_capabilities_are_explicit() -> None:
    modules = {item.module_id: item for item in static_deferred_modules()}

    edge = modules["edge_protection"]
    assert edge.installation_state == INSTALLATION_NOT_INSTALLED
    assert edge.implementation_state == IMPLEMENTATION_NOT_ADMITTED
    assert not edge.enabled
    assert not edge.configuration_valid
    assert not edge.data_valid

    force = modules["force_control"]
    exchange = modules["battery_exchange"]
    assert force.implementation_state == IMPLEMENTATION_NOT_IMPLEMENTED
    assert exchange.implementation_state == IMPLEMENTATION_NOT_IMPLEMENTED
    assert not force.enabled and not exchange.enabled


def test_enabled_edge_protection_latches_on_trigger_or_invalid_input() -> None:
    core = ProtectiveStopCore(installed=True, enabled=True)

    triggered = core.update_inputs((("front", True, True),))
    assert triggered.latched
    assert triggered.stop_required
    assert triggered.active_inputs == ("front",)

    recovered = core.update_inputs((("front", False, True),))
    assert recovered.latched
    assert not recovered.reset_conditions_met

    core.set_motion_stopped(True)
    cleared, state = core.reset("operator-confirmation-1")
    assert cleared
    assert not state.latched
    assert not state.stop_required

    invalid = core.update_inputs((("front", False, False),))
    assert invalid.latched
    assert invalid.stop_required


def test_protective_stop_cannot_enable_or_reset_without_installed_hardware() -> None:
    with pytest.raises(ValueError, match="not installed"):
        ProtectiveStopCore(installed=False, enabled=True)

    core = ProtectiveStopCore(installed=False, enabled=False)
    cleared, state = core.reset("operator-confirmation-1")
    assert not cleared
    assert not state.reset_conditions_met
