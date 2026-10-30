"""Console admission checks, poses and Robot Model handling with a fake driver. No ROS."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rt_control_arm_console.console import ArmConsole, ConsoleError, config_from_mapping  # noqa: E402
from rt_control_arm_console.model import ModelError, load_arm_model, resolve_mesh  # noqa: E402
from rt_control_arm_console.poses import PoseStore  # noqa: E402
from rt_control_arm_console.trajectory import plan_move  # noqa: E402

CONFIG = {
    "gears": {
        "leisure": {
            "label": "休闲",
            "velocity_deg_s": 10,
            "acceleration_deg_s2": 20,
            "jerk_deg_s3": 80,
        },
        "sport": {
            "label": "运动",
            "velocity_deg_s": 30,
            "acceleration_deg_s2": 60,
            "jerk_deg_s3": 240,
        },
    },
    "limit_margin_deg": 2.0,
    "max_feedback_age_s": 0.2,
    "start_tolerance_deg": 0.5,
    "sample_period_s": 0.01,
    "stop_lead_s": 0.02,
    "start_delay_s": 0.05,
}
JOINTS = ["left_joint1", "left_joint2"]
URDF = """<robot name="r">
  <link name="world"/>
  <link name="mount"><visual><geometry><mesh filename="package://pkg/meshes/mount.stl"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="l1"/><link name="other"/>
  <link name="l2"><visual><geometry><mesh filename="package://pkg/meshes/l2.stl"/></geometry></visual></link>
  <joint name="fix" type="fixed"><origin xyz="0 0 1" rpy="3.14159265 0 0"/>
    <parent link="world"/><child link="mount"/></joint>
  <joint name="left_joint1" type="revolute"><parent link="mount"/><child link="l1"/>
    <limit lower="-1.0" upper="1.0" velocity="1.3" effort="1"/></joint>
  <joint name="left_joint2" type="revolute"><parent link="l1"/><child link="l2"/>
    <limit lower="-0.5" upper="2.0" velocity="0.1" effort="1"/></joint>
  <joint name="right" type="revolute"><parent link="world"/><child link="other"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
  <ros2_control name="x" type="system"/>
</robot>"""


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Driver:
    def __init__(self):
        self.sent = []
        self.starts = []
        self.cancelled = 0

    def send(self, joints, points, done, start):
        self.sent.append((tuple(joints), list(points), done))
        self.starts.append(start)

    def cancel(self):
        self.cancelled += 1


@pytest.fixture
def setup(tmp_path):
    clock, driver = Clock(), Driver()
    console = ArmConsole(
        config_from_mapping(CONFIG), "left_arm_jtc", driver, tmp_path / "poses.json", clock
    )
    console.set_model(load_arm_model(URDF, JOINTS))
    console.update(
        positions={"left_joint1": 0.0, "left_joint2": 0.2},
        feedback_at=clock.now,
        enable_state="ENABLED",
        enable_at=clock.now,
        controller_active=True,
        controller_at=clock.now,
        capability={"topic": "/x"},
        mock=True,
    )
    console.acquire("s1")
    return console, driver, clock


def test_vacuum_pressure_is_independent_and_expires_without_motion_effects(setup):
    console, driver, clock = setup
    assert all(not sample["valid"] for sample in console.status("s1")["vacuum_pressure"].values())
    console.update_vacuum_pressure("left", -42.3, True)
    console.update_vacuum_pressure("right", -0.8, True)
    readings = console.status("s1")["vacuum_pressure"]
    assert readings["left"]["pressure_kpa"] == -42.3
    assert readings["right"]["pressure_kpa"] == -0.8
    clock.now += 0.8
    console.update_vacuum_pressure("right", -10.0, True)
    clock.now += 0.3
    readings = console.status("s1")["vacuum_pressure"]
    assert readings["left"] == {
        "pressure_kpa": None, "valid": False, "error": "stale", "unit": "kPa"
    }
    assert readings["right"]["pressure_kpa"] == -10.0
    console.update_vacuum_pressure("left", 0.0, True)
    assert console.status("s1")["vacuum_pressure"]["left"]["valid"]
    assert not driver.sent
    assert driver.cancelled == 0


@pytest.mark.parametrize("pressure,valid,error,unit,expected", [
    (-1.0, False, "modbus_timeout", "kPa", "modbus_timeout"),
    (float("nan"), True, "", "kPa", "non_finite"),
    (float("inf"), True, "", "kPa", "non_finite"),
    (-1.0, True, "", "MPa", "invalid_unit"),
])
def test_vacuum_pressure_invalid_samples_hide_previous_number(setup, pressure, valid, error, unit, expected):
    console, _, _ = setup
    console.update_vacuum_pressure("left", -50.0, True)
    console.update_vacuum_pressure("left", pressure, valid, error, unit)
    sample = console.status("s1")["vacuum_pressure"]["left"]
    assert sample["pressure_kpa"] is None
    assert not sample["valid"]
    assert sample["error"] == expected


def test_browser_view_shows_the_whole_robot_and_names_the_arm_links():
    model = load_arm_model(URDF, JOINTS)
    robot = ET.fromstring(model.browser_urdf)
    assert model.root_link == "mount"
    assert model.arm_links == ("l1", "l2")
    assert {j.get("name") for j in robot.findall("joint")} == {"fix", *JOINTS, "right"}
    assert {link.get("name") for link in robot.findall("link")} == {
        "world",
        "mount",
        "l1",
        "other",
        "l2",
    }
    assert sorted(m.get("filename") for m in robot.iter("mesh")) == [
        "/mesh/pkg/meshes/l2.stl",
        "/mesh/pkg/meshes/mount.stl",
    ]
    assert robot.find(".//collision") is None and robot.find("ros2_control") is None
    with pytest.raises(ModelError, match="missing"):
        load_arm_model(URDF, ["left_joint9"])


def test_mesh_resolution_stays_inside_the_package_share(tmp_path):
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes/a.stl").write_bytes(b"solid")
    (tmp_path / "secret.txt").write_text("x")
    lookup = {"pkg": str(tmp_path)}.__getitem__
    assert resolve_mesh("pkg/meshes/a.stl", lookup) == (tmp_path / "meshes/a.stl").resolve()
    for bad in ("pkg/../secret.txt", "pkg/secret.txt", "nopkg/meshes/a.stl", "pkg"):
        assert resolve_mesh(bad, lookup) is None


def test_execute_sends_a_synchronised_trajectory_from_the_measured_pose(setup):
    console, driver, _ = setup
    result = console.execute("s1", {"left_joint1": 0.5, "left_joint2": 0.2}, "sport")
    joints, points, _ = driver.sent[0]
    assert joints == tuple(JOINTS)
    assert points[0].positions == (0.0, 0.2) and points[0].time == 0.0
    assert points[-1].positions == pytest.approx((0.5, 0.2))
    assert result["duration_s"] == pytest.approx(points[-1].time)
    with pytest.raises(ConsoleError, match="尚未结束"):
        console.execute("s1", {"left_joint1": 0.1, "left_joint2": 0.2}, "sport")


def test_model_velocity_caps_the_gear_for_moving_joints(setup):
    console, driver, _ = setup
    console.execute("s1", {"left_joint1": 0.0, "left_joint2": 1.2}, "sport")
    points = driver.sent[0][1]
    assert max(abs(p.velocities[1]) for p in points) <= 0.1 + 1e-9  # model limit < 30 deg/s


@pytest.mark.parametrize(
    "change,message",
    [
        ({"enable_state": "IDLE"}, "未使能"),
        ({"controller_active": False}, "未激活"),
        ({"capability": None}, "能力"),
    ],
)
def test_execute_requires_enable_controller_and_capability(setup, change, message):
    console, driver, _ = setup
    console.update(**change)
    with pytest.raises(ConsoleError, match=message):
        console.execute("s1", {"left_joint1": 0.5, "left_joint2": 0.2}, "sport")
    assert not driver.sent


def test_execute_rejects_stale_feedback_lease_and_out_of_range_targets(setup):
    console, driver, clock = setup
    with pytest.raises(ConsoleError, match="超出允许范围"):
        console.execute(
            "s1", {"left_joint1": 1.0, "left_joint2": 0.2}, "sport"
        )  # within the model-limit margin
    with pytest.raises(ConsoleError, match="全部受控关节"):
        console.execute("s1", {"left_joint1": 0.5}, "sport")
    with pytest.raises(ConsoleError, match="控制权"):
        console.execute("s2", {"left_joint1": 0.5, "left_joint2": 0.2}, "sport")
    clock.now += 0.5
    console.touch("s1")
    with pytest.raises(ConsoleError, match="反馈超时"):
        console.execute("s1", {"left_joint1": 0.5, "left_joint2": 0.2}, "sport")
    assert not driver.sent


def test_lease_expires_without_status_stream(setup):
    console, _, clock = setup
    with pytest.raises(ConsoleError, match="另一个终端"):
        console.acquire("s2")
    clock.now += 5.0
    console.acquire("s2")
    assert console.status("s2")["lease"] == "you"


def test_stop_joins_the_running_motion_and_reports_the_result(setup):
    console, driver, clock = setup
    console.update(enable_at=clock.now + 10, controller_at=clock.now + 10)
    console.execute("s1", {"left_joint1": 0.8, "left_joint2": 0.2}, "sport")
    move_done = driver.sent[0][2]
    clock.now += 1.0
    console.update(feedback_at=clock.now)
    assert console.stop()["stopping"] is True
    _, stop_points, stop_done = driver.sent[1]
    # Same timeline as the move (start delay 0.05 s). The stop starts one sample in
    # the past on the move's own points; braking starts 0.02 s ahead of now.
    assert driver.starts == [pytest.approx(100.05)] * 2
    now, cut = 1.0 - 0.05, 1.0 - 0.05 + 0.02
    assert stop_points[0].time <= now - 0.01 + 1e-9
    times = [point.time for point in stop_points]
    assert all(b > a for a, b in zip(times, times[1:]))
    console_plan = console._execution  # noqa: SLF001 - white-box check of the joined line
    for point in stop_points:
        if point.time < cut - 1e-9:
            reference = plan_move((0.0, 0.2), (0.8, 0.2), console.config.gear("sport").limits)
            assert point.positions[0] == pytest.approx(reference.state(point.time).positions[0])
    assert console_plan.kind == "stop"
    assert any(abs(point.time - cut) < 1e-9 for point in stop_points)
    assert stop_points[-1].velocities == (0.0, 0.0)
    move_done(False, "preempted")  # superseded: ignored
    assert console.status("s1")["moving"]["kind"] == "stop"
    stop_done(True, "")
    assert console.status("s1")["moving"] is None
    assert console.status("s1")["last_result"] == "已停止"


def test_failed_stop_falls_back_to_controller_hold(setup):
    console, driver, clock = setup
    console.execute("s1", {"left_joint1": 0.8, "left_joint2": 0.2}, "sport")
    clock.now += 0.5
    console.stop()
    driver.sent[1][2](False, "rejected")
    assert driver.cancelled == 1


def test_poses_save_list_and_goto(setup, tmp_path):
    console, driver, _ = setup
    console.save_pose("s1", " 抓取位 ", overwrite=False)
    with pytest.raises(ConsoleError, match="已存在"):
        console.save_pose("s1", "抓取位", overwrite=False)
    console.update(positions={"left_joint1": 0.3, "left_joint2": 0.2})
    console.goto_pose("s1", "抓取位", "leisure")
    assert driver.sent[0][1][-1].positions == pytest.approx((0.0, 0.2))
    reloaded = PoseStore(tmp_path / "poses.json", JOINTS)
    assert [pose["name"] for pose in reloaded.list()] == ["抓取位"]
    assert PoseStore(tmp_path / "poses.json", ["other"]).list() == []


def test_mesh_lookup_accepts_symlink_installed_meshes(tmp_path):
    source = tmp_path / "src/meshes"
    source.mkdir(parents=True)
    (source / "a.stl").write_bytes(b"solid a")
    share = tmp_path / "install/share/pkg/meshes"
    share.mkdir(parents=True)
    (share / "a.stl").symlink_to(source / "a.stl")  # what colcon --symlink-install produces

    def lookup(_package):
        return tmp_path / "install/share/pkg"

    found = resolve_mesh("pkg/meshes/a.stl", lookup)
    assert found is not None and found.read_bytes() == b"solid a"
    assert resolve_mesh("pkg/../src/meshes/a.stl", lookup) is None


def _sequence_console(setup):
    console, driver, clock = setup
    console._spawn = lambda job: job()  # noqa: SLF001 - run the next step synchronously
    console.update(enable_at=clock.now + 100, controller_at=clock.now + 100)
    console.save_pose("s1", "A", overwrite=False)  # (0.0, 0.2)
    console.update(positions={"left_joint1": 0.5, "left_joint2": 0.2})
    console.save_pose("s1", "B", overwrite=False)  # (0.5, 0.2)
    return console, driver, clock


def _arrive(console, driver, clock, index, success=True):
    """Report the trajectory ``index`` as finished at its last point."""
    _, points, done = driver.sent[index]
    clock.now += 0.1
    console.update(positions=dict(zip(JOINTS, points[-1].positions)), feedback_at=clock.now)
    done(success, "" if success else "aborted")


def test_sequence_visits_poses_in_order_with_repeats(setup):
    console, driver, clock = _sequence_console(setup)
    assert console.run_sequence("s1", ["A", "B", "A"], "leisure") == {"total": 3}
    status = console.status("s1")
    assert status["sequence"] == {"names": ["A", "B", "A"], "index": 0, "total": 3,
                                   "dwell_s": 0.0, "dwelling": False, "dwell_remaining_s": 0.0,
                                   "steps": [{"name": name, "gear": "leisure", "dwell_s": 0.0}
                                             for name in ["A", "B", "A"]]}
    assert status["moving"]["label"] == "1/3 A"
    for index, goal in enumerate([(0.0, 0.2), (0.5, 0.2), (0.0, 0.2)]):
        assert driver.sent[index][1][-1].positions == pytest.approx(goal)
        _arrive(console, driver, clock, index)
    assert len(driver.sent) == 3
    status = console.status("s1")
    assert status["sequence"] is None and status["moving"] is None
    assert status["last_result"] == "序列完成（3 个位姿）"


def test_sequence_skips_a_pose_that_is_already_reached(setup):
    console, driver, clock = _sequence_console(setup)  # currently at B
    console.run_sequence("s1", ["B", "A"], "leisure")
    assert len(driver.sent) == 1
    assert driver.sent[0][1][-1].positions == pytest.approx((0.0, 0.2))
    assert console.status("s1")["moving"]["label"] == "2/2 A"


def test_stop_cancels_the_rest_of_the_sequence(setup):
    console, driver, clock = _sequence_console(setup)
    console.run_sequence("s1", ["A", "B"], "leisure")
    clock.now += 0.5
    console.update(feedback_at=clock.now)
    assert console.stop()["stopping"] is True
    assert console.status("s1")["sequence"] is None
    driver.sent[1][2](True, "")  # stop finished
    assert len(driver.sent) == 2
    assert console.status("s1")["last_result"] == "已停止"


def test_failed_step_aborts_the_sequence(setup):
    console, driver, clock = _sequence_console(setup)
    console.run_sequence("s1", ["A", "B"], "leisure")
    _arrive(console, driver, clock, 0, success=False)
    status = console.status("s1")
    assert status["sequence"] is None and len(driver.sent) == 1
    assert status["last_result"].startswith("序列中止（1/2 A）")


def test_sequence_checks_every_pose_before_moving(setup):
    console, driver, _ = _sequence_console(setup)
    with pytest.raises(ConsoleError, match="第 2 个"):
        console.run_sequence("s1", ["A", "缺失"], "leisure")
    with pytest.raises(ConsoleError, match="至少一个"):
        console.run_sequence("s1", [], "leisure")
    with pytest.raises(ConsoleError, match="控制权"):
        console.run_sequence("s2", ["A"], "leisure")
    console.update(enable_state="IDLE")
    with pytest.raises(ConsoleError, match="未使能"):
        console.run_sequence("s1", ["A"], "leisure")
    assert not driver.sent and console.status("s1")["sequence"] is None


def _dwell_console(setup):
    console, driver, clock = _sequence_console(setup)
    jobs = []
    console._spawn = jobs.append

    def wait(seconds):
        clock.now += seconds
        console.touch('s1')
        console.update(feedback_at=clock.now, enable_at=clock.now, controller_at=clock.now)

    console._wait = wait
    return console, driver, clock, jobs


@pytest.mark.parametrize('dwell', [2, 8])
def test_dwell_waits_full_two_seconds_and_includes_final_pose(setup, dwell):
    console, driver, clock, jobs = _dwell_console(setup)
    console.run_sequence('s1', ['A', 'B'], 'leisure', dwell)
    _arrive(console, driver, clock, 0)
    arrived = clock.now
    state = console.status('s1')['sequence']
    assert state['dwelling'] and state['dwell_remaining_s'] == dwell
    assert len(driver.sent) == 1
    times = []
    wait = console._wait

    def inspect(seconds):
        assert len(driver.sent) == 1
        times.append(clock.now)
        wait(seconds)

    console._wait = inspect
    jobs.pop(0)()
    assert times and clock.now - arrived >= dwell
    assert len(driver.sent) == 2
    console._wait = wait
    _arrive(console, driver, clock, 1)
    assert console.status('s1')['sequence']['dwelling']
    jobs.pop(0)()
    assert console.status('s1')['sequence'] is None


def test_already_reached_pose_still_dwells(setup):
    console, driver, clock, jobs = _dwell_console(setup)
    console.run_sequence('s1', ['B', 'A'], 'leisure', 2)
    assert not driver.sent
    assert console.status('s1')['sequence']['dwelling']
    jobs.pop(0)()
    assert len(driver.sent) == 1 and clock.now >= 102


def test_stop_during_dwell_and_old_worker_cannot_advance_new_sequence(setup):
    console, driver, clock, jobs = _dwell_console(setup)
    console.run_sequence('s1', ['A', 'B'], 'leisure', 2)
    _arrive(console, driver, clock, 0)
    old_job = jobs.pop(0)
    assert console.stop() == {'stopping': False}
    console.run_sequence('s1', ['B'], 'leisure', 2)
    old_job()
    assert len(driver.sent) == 2
    assert console.status('s1')['sequence']['index'] == 0


@pytest.mark.parametrize('failure', ['stop', 'lease', 'feedback', 'enable', 'controller'])
def test_dwell_interruption_sends_no_next_motion(setup, failure):
    console, driver, clock, jobs = _dwell_console(setup)
    console.run_sequence('s1', ['A', 'B'], 'leisure', 2)
    _arrive(console, driver, clock, 0)

    def fail(seconds):
        clock.now += seconds
        if failure == 'stop':
            console.stop()
        elif failure == 'lease':
            console.release('s1')
        elif failure == 'feedback':
            console.update(feedback_at=clock.now - 1)
        elif failure == 'enable':
            console.update(enable_state='IDLE')
        else:
            console.update(controller_active=False)

    console._wait = fail
    jobs.pop(0)()
    assert len(driver.sent) == 1
    assert console.status('s1')['sequence'] is None


@pytest.mark.parametrize('value', [True, -2, '2', None, float('nan'), float('inf')])
def test_invalid_dwell_rejected_before_motion(setup, value):
    console, driver, _, _ = _dwell_console(setup)
    with pytest.raises(ConsoleError, match='停留时间'):
        console.run_sequence('s1', ['A'], 'leisure', value)
    assert not driver.sent


def test_dwell_blocks_manual_move_and_enable_change(setup):
    console, driver, clock, jobs = _dwell_console(setup)
    console.run_sequence('s1', ['A', 'B'], 'leisure', 2)
    _arrive(console, driver, clock, 0)
    with pytest.raises(ConsoleError, match='序列'):
        console.goto_pose('s1', 'B', 'leisure')
    with pytest.raises(ConsoleError, match='序列'):
        console.begin_enable('s1')
    console.stop()
    jobs.pop(0)()
    assert len(driver.sent) == 1


def test_low_speed_sequence_advances_within_controller_point_budget(setup):
    from copy import deepcopy
    console, driver, clock, jobs = _dwell_console(setup)
    cfg = deepcopy(CONFIG)
    cfg['gears']['leisure'].update(velocity_deg_s=0.2, acceleration_deg_s2=0.4, jerk_deg_s3=1.6)
    console.config = config_from_mapping(cfg)
    original_send = driver.send

    def bounded_send(joints, points, done, start):
        assert len(points) <= 2000  # Same admission limit as deployed JTC.
        original_send(joints, points, done, start)

    driver.send = bounded_send
    console.run_sequence('s1', ['A', 'B', 'A'], 'leisure', 2)
    for i in range(3):
        assert len(driver.sent) == i + 1
        _arrive(console, driver, clock, i)
        jobs.pop(0)()
    assert console.status('s1')['sequence'] is None
    assert console.status('s1')['last_result'] == '序列完成（3 个位姿）'


def test_sequence_uses_each_steps_speed_and_dwell_including_final(setup):
    console, driver, clock, jobs = _dwell_console(setup)
    steps = [{"name": "A", "gear": "leisure", "dwell_s": 0.3},
             {"name": "B", "gear": "sport", "dwell_s": 1.7}]
    console.run_sequence("s1", [], "", steps=steps)
    assert console._execution.gear.name == "leisure"
    steps[1]["gear"] = "leisure"  # Request mutation cannot change the running sequence.
    _arrive(console, driver, clock, 0)
    assert console.status("s1")["sequence"]["dwell_remaining_s"] == pytest.approx(0.3)
    arrived = clock.now
    jobs.pop(0)()
    assert clock.now - arrived == pytest.approx(0.3)
    assert console._execution.gear.name == "sport"
    _arrive(console, driver, clock, 1)
    assert console.status("s1")["sequence"]["dwell_remaining_s"] == pytest.approx(1.7)
    arrived = clock.now
    jobs.pop(0)()
    assert clock.now - arrived == pytest.approx(1.7)
    assert console.status("s1")["sequence"] is None
    assert len(driver.sent) == 2


@pytest.mark.parametrize("bad_step", [
    {"name": "B", "gear": "missing", "dwell_s": 0},
    {"name": "B", "gear": "sport", "dwell_s": -0.1},
    {"name": "missing", "gear": "sport", "dwell_s": 0},
    {"name": "B", "gear": "sport", "dwell_s": float("nan")},
    {"name": "B", "gear": "sport", "dwell_s": True},
    {"name": "B"}, None,
])
def test_invalid_later_step_rejects_entire_sequence_before_motion(setup, bad_step):
    console, driver, _, _ = _dwell_console(setup)
    with pytest.raises(ConsoleError):
        console.run_sequence("s1", [], "", steps=[
            {"name": "A", "gear": "leisure", "dwell_s": 0}, bad_step])
    assert not driver.sent
    assert console.status("s1")["sequence"] is None
