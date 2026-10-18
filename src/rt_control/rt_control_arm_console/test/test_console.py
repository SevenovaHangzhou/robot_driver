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
    assert status["sequence"] == {"names": ["A", "B", "A"], "index": 0, "total": 3}
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
