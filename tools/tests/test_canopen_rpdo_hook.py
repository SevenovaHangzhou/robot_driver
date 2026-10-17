from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/ros2_canopen/0006-expose-rpdo-receive-hook.patch"
LIFECYCLE_PATCH = ROOT / "patches/ros2_canopen/0001-shared-canopen-lifecycle.patch"


def test_rpdo_hook_patch_exposes_receive_time_without_changing_default_storage():
    text = PATCH.read_text(encoding="utf-8")

    assert "virtual void on_rpdo_received(" in text
    assert "std::chrono::steady_clock::time_point received_at" in text
    assert "canopen_data_[id].set_rpdo_data(data);" in text
    assert "on_rpdo_received(data, id, std::chrono::steady_clock::now());" in text
    assert "auto rpdo_cb = [this]" in text


def test_native_and_manual_docker_use_the_shared_patch_without_motor_overlay():
    for relative_path in ("tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"):
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        shared = text.index("0001-shared-canopen-lifecycle.patch")
        txqlen = text.index("0002-lely-preconfigured-txqlen.patch", shared)
        hook = text.index("0006-expose-rpdo-receive-hook.patch", txqlen)
        assert shared < txqlen < hook
        assert "0005-derive-motor-topology-from-hardware-info.patch" not in text

    lifecycle = LIFECYCLE_PATCH.read_text(encoding="utf-8")
    patch = PATCH.read_text(encoding="utf-8")
    for text in (lifecycle, patch):
        assert "canopen_402_driver/" not in text
        assert "cia402_system.cpp" not in text
        assert "Track node" not in text


NMT_PATCH = ROOT / "patches/ros2_canopen/0007-expose-nmt-state-hook.patch"


def test_nmt_hook_patch_keeps_default_storage_and_follows_rpdo_hook():
    text = NMT_PATCH.read_text(encoding="utf-8")
    assert "virtual void on_nmt_state_changed(canopen::NmtState nmt_state, uint8_t id);" in text
    assert "canopen_data_[id].nmt_state.set_state(nmt_state);" in text
    assert "auto nmt_state_cb = [this]" in text
    for relative_path in ("tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        rpdo = source.index("0006-expose-rpdo-receive-hook.patch")
        assert source.index("0007-expose-nmt-state-hook.patch", rpdo) > rpdo
