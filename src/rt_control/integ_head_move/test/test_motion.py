import pytest

from integ_head_move.motion import arrived, HeadJoint, load_head, target_error, time_limit


def test_head_is_read_from_the_artifact_in_controller_order(generated_dir):
    topic, joints = load_head(generated_dir)
    assert topic == "/head_position_controller/commands"
    assert joints == [
        HeadJoint("H1_joint", -1.57, 1.57, 0.2),
        HeadJoint("H2_joint", -1.0, 1.0, 0.4),
    ]


@pytest.mark.parametrize(
    ("target", "error"),
    [([0.5, -0.5], None), ([1.6, 0.0], "H1_joint"), ([0.0, float("nan")], "H2_joint")],
)
def test_targets_outside_the_command_range_are_named(generated_dir, target, error):
    found = target_error(load_head(generated_dir)[1], target)
    assert found is None if error is None else error in found


def test_time_limit_is_the_slowest_joint_plus_two_seconds(generated_dir):
    joints = load_head(generated_dir)[1]
    # H1: 0.4 rad at 0.2 rad/s = 2 s; H2: 0.4 rad at 0.4 rad/s = 1 s.
    assert time_limit(joints, [0.0, 0.0], [0.4, -0.4]) == pytest.approx(4.0)


def test_arrival_needs_every_joint_within_a_hundredth_of_a_radian():
    assert arrived([0.5, 0.2], [0.509, 0.191])
    assert not arrived([0.5, 0.2], [0.5, 0.215])
