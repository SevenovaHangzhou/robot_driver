from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/ros2_canopen/0006-expose-rpdo-receive-hook.patch"


def test_rpdo_hook_patch_exposes_receive_time_without_changing_default_storage():
    text = PATCH.read_text(encoding="utf-8")

    assert "virtual void on_rpdo_received(" in text
    assert "std::chrono::steady_clock::time_point received_at" in text
    assert "canopen_data_[id].set_rpdo_data(data);" in text
    assert "on_rpdo_received(data, id, std::chrono::steady_clock::now());" in text
    assert "auto rpdo_cb = [this]" in text


def test_native_and_manual_docker_patch_chains_apply_0006_after_0005():
    for relative_path in ("tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"):
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        previous = text.index("0005-derive-motor-topology-from-hardware-info.patch")
        hook = text.index("0006-expose-rpdo-receive-hook.patch", previous)
        assert previous < hook
