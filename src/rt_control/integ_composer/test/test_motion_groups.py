"""enable_manager motion groups (ELECTRI-236): one nested parameter group per group with an
activatable member, and one exclusive mode-controller set per switching group."""

from integ_composer.compose import _mode_controller_groups
from integ_composer.parameter_sources import controller_parameters

TEMPLATES = [
    {
        "name": "arms",
        "parameters": [
            {"controllers": {"tag": "enable_target", "select": "first", "as": "motion_controller"}},
            {
                "controllers": {
                    "tag": "rolling",
                    "select": "first",
                    "optional": True,
                    "as": "rolling_controller",
                }
            },
        ],
    },
    {
        "name": "base",
        "parameters": [
            {"controllers": {"tag": "enable_target", "select": "first", "as": "motion_controller"}},
            {
                "controllers": {
                    "tag": "base_trajectory",
                    "select": "first",
                    "as": "trajectory_controller",
                }
            },
        ],
    },
    {
        "name": "lift",
        "parameters": [
            {"controllers": {"tag": "enable_target", "select": "first", "as": "motion_controller"}}
        ],
    },
]
BINDINGS = {
    "arm_jtc": {"motion_group": "arms", "tags": ["trajectory", "enable_target"]},
    "arm_rolling": {"motion_group": "arms", "tags": ["rolling"]},
    "swerve": {"motion_group": "base", "tags": ["enable_target", "fault_reporting"]},
    "wheel_jtc": {"motion_group": "base", "tags": ["base_trajectory"]},
    "head": {"tags": ["enable_companion"]},
}


def environment(selected):
    return {
        "selected": selected,
        "tags": {name: BINDINGS[name]["tags"] for name in selected},
        "motion_groups": {name: BINDINGS[name].get("motion_group") for name in selected},
        "policies": {},
        "rendered_modules": {},
    }


def manager_parameters(selected):
    binding = {
        "module": "arms",
        "motion_groups": TEMPLATES,
        "parameters": [
            {
                "controllers": {
                    "tag": "enable_companion",
                    "select": "all",
                    "optional": True,
                    "as": "companion_controllers",
                }
            }
        ],
    }
    return controller_parameters("enable_manager", binding, environment(selected))


def test_each_group_with_an_activatable_member_gets_its_own_parameters():
    parameters = manager_parameters(["arm_jtc", "arm_rolling", "swerve", "wheel_jtc", "head"])
    assert parameters == {
        "motion_groups": ["arms", "base"],
        "arms": {"motion_controller": "arm_jtc", "rolling_controller": "arm_rolling"},
        "base": {"motion_controller": "swerve", "trajectory_controller": "wheel_jtc"},
        "companion_controllers": ["head"],
    }


def test_groups_without_an_activatable_member_are_omitted():
    # The rolling controller alone cannot be enabled; the lift has no member in this scope.
    assert manager_parameters(["arm_rolling", "head"]) == {"companion_controllers": ["head"]}


def test_every_switching_group_has_one_exclusive_controller_set():
    order = {"inactive": ["arm_jtc", "arm_rolling", "swerve", "wheel_jtc", "head"], "active": []}
    assert _mode_controller_groups(order, BINDINGS) == [
        ["arm_jtc", "arm_rolling"],
        ["swerve", "wheel_jtc"],
    ]
    # Without its alternate a group has nothing to switch between.
    jtc_only = {"inactive": ["arm_jtc", "swerve", "wheel_jtc", "head"], "active": []}
    assert _mode_controller_groups(jtc_only, BINDINGS) == [["swerve", "wheel_jtc"]]
