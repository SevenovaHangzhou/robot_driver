"""Regression coverage for concurrent archives and IgH snapshot underflow."""

import json
from concurrent.futures import ThreadPoolExecutor
from threading import Barrier

import pytest

from runtime_log import blackbox, ethercat
from test_blackbox import FIELDS, PyWriter, make_box, run_cycles
from test_recording import artifact_dir  # noqa: F401 - fixture shared with recording tests


def test_simultaneous_segments_keep_independent_valid_windows(tmp_path, monkeypatch):
    from mcap.reader import make_reader

    boxes = []
    for i in range(2):
        writer = PyWriter(tmp_path / f"segment-{i}", FIELDS + [f"identity_{i}"])
        box, _ = make_box(tmp_path, writer.path, post_s=0)
        for seq in range(10):
            writer.write([seq, 0x1637, 8, 0, 0, i])
        box._step(final=False)
        box.trigger("cycle_overrun")
        boxes.append(box)
    barrier = Barrier(2)
    original = blackbox.write_mcap

    def overlap(*args):
        barrier.wait(timeout=5)
        return original(*args)

    monkeypatch.setattr(blackbox, "write_mcap", overlap)
    with ThreadPoolExecutor(2) as pool:
        list(pool.map(lambda box: box._step(final=False), boxes))
    assert len({box.saved[0] for box in boxes}) == 2
    for i, box in enumerate(boxes):
        directory = box.out_dir / box.saved[0]
        meta = json.loads((directory / "trigger.json").read_text())
        assert meta["segment"] == f"segment-{i}"
        with (directory / "window.mcap").open("rb") as stream:
            messages = [json.loads(m.data) for _, _, m in make_reader(stream).iter_messages()]
        assert len(messages) == 10
        assert all(m[f"identity_{i}"] == i for m in messages)


def test_archive_io_failure_reports_error_and_keeps_sampling(tmp_path, monkeypatch):
    writer = PyWriter(tmp_path / "segment", FIELDS)
    box, events = make_box(tmp_path, writer.path, post_s=0)
    run_cycles(writer, 10)
    box._step(final=False)
    box.trigger("manual")
    original = blackbox.write_mcap

    def fail(*args):
        raise OSError("injected archive failure")

    monkeypatch.setattr(blackbox, "write_mcap", fail)
    box._step(final=False)
    assert not box.saved
    assert events[-1]["kind"] == "blackbox_error"
    assert "injected archive failure" in events[-1]["detail"]
    monkeypatch.setattr(blackbox, "write_mcap", original)
    run_cycles(writer, 10)
    box.trigger("manual")
    box._step(final=False)
    assert box.describe()["cycle_stats"]["cycles"] == 20
    assert len(box.saved) == 1


@pytest.mark.parametrize("bits", [32, 64])
def test_underflow_does_not_poison_loss_high_water_mark(bits):
    tracker = ethercat.CounterTracker()

    def sample(tx, rx, lost):
        return ethercat.parse_master(
            f"Main:\n Tx frames: {tx}\n Rx frames: {rx}\nCommon:\n Lost frames: {lost}\n"
        )["counters"]

    assert tracker.update(sample(90000, 90000, 0)) == {}
    dip = sample(90677, 90678, (1 << bits) - 1)
    assert dip["lost_frames"] == 0
    assert tracker.update(dip)["lost_frames"] == 0
    assert tracker.update(sample(91720, 91720, 0))["lost_frames"] == 0
    assert tracker.update(sample(92720, 92717, 3))["lost_frames"] == 3
    # A large counter without corroborating rx > tx must remain visible.
    assert sample(1 << 33, 1 << 32, (1 << 32) - 1)["lost_frames"] == (1 << 32) - 1


def test_old_unsigned_snapshots_replay_without_false_losses():
    from runtime_log.recorder import counter_events

    rows = [
        {"ts": i, "counters": {"tx_frames": tx, "rx_frames": rx, "lost_frames": lost}}
        for i, (tx, rx, lost) in enumerate(
            [(100, 100, 0), (200, 201, 4294967295), (300, 300, 0), (400, 398, 2)]
        )
    ]
    events = counter_events(rows)
    assert len(events) == 1 and events[0].data["lost_frames"] == 2


def test_manual_experiments_from_two_segments_do_not_overwrite(tmp_path):
    from mcap.reader import make_reader

    boxes = []
    for i in range(2):
        fields = [f"joint_{i}/position", f"joint_{i}/target_position"]
        writer = PyWriter(tmp_path / f"segment-{i}", fields)
        box, _ = make_box(tmp_path, writer.path)
        for seq in range(10):
            writer.write([i, i])
        box._step(final=False)
        start = 1_800_000_000.0
        box.start_experiment("goal-1", "manual", start)
        box._step(final=False)
        boxes.append(box)
    assert len({box.experiment.directory for box in boxes}) == 2
    for i, box in enumerate(boxes):
        directory = box.experiment.directory
        assert directory.name.endswith("-goal-1")  # restart goal numbering remains compatible
        box._step(final=True)
        with (directory / "data.mcap").open("rb") as stream:
            rows = [json.loads(m.data) for _, _, m in make_reader(stream).iter_messages()]
        assert len(rows) == 10
        assert all(row[f"joint_{i}/position"] == i for row in rows)


def test_archive_error_is_visible_in_run_timeline(tmp_path, profile, artifact_dir):
    from runtime_log.archive import Archive
    from runtime_log.recorder import RunRecorder
    from test_recording import LAUNCH_PID, FIXTURES, FakeProc

    log_dir = tmp_path / f"x-{LAUNCH_PID}"
    log_dir.mkdir()
    (log_dir / "launch.log").write_text((FIXTURES / "launch_normal.log").read_text())
    recorder = RunRecorder(
        LAUNCH_PID, log_dir, profile, Archive(tmp_path / "runs"), FakeProc(artifact_dir), None
    )
    recorder.blackbox_events.put(
        {
            "kind": "blackbox_error",
            "ts": recorder.start + 1,
            "segment": "segment",
            "reasons": ["manual"],
            "detail": "disk full",
        }
    )
    recorder.drain_blackbox_events()
    assert recorder.events[-1].rule == "blackbox_error"
    assert recorder.events[-1].level == "error"
    assert recorder.events[-1].message == "disk full"
    recorder.finish(recorder.start + 2)
