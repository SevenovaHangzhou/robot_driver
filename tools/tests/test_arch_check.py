"""arch_check: every rule accepts valid input and reports a new violation with its rule ID."""

import subprocess
from pathlib import Path

import pytest
import yaml

from tools import arch_check
from tools.archcheck import layers, repo

PACKAGES = (
    "infra_ring",
    "hwcfg_machines",
    "hw_bus",
    "hw_semantic_components",
    "ctrl_motor",
    "periph_bms",
    "integ_composer",
)


def tree(**extra):
    """A minimal compliant source tree as path -> text."""
    texts = {
        repo.RULES_FILE: yaml.safe_dump(
            {
                "exceptions": [],
                "listen_only_peripherals": ["periph_bms"],
                "vendor_names": ["AcmeMotors"],
                "duplicates": [
                    {"name": "raw CAN", "owner": "infra_ring", "pattern": r"\bAF_CAN\b"},
                    {"name": "old codec", "owner": "", "pattern": r"\bencode_double\b"},
                ],
            }
        ),
        # Machine and joint names are read from the data package itself.
        "src/rt_control/hwcfg_machines/machines/Demo_V9/arms.yaml": yaml.safe_dump(
            {"spec": {"axes": [{"robot_model_joint": "arm_joint1"}]}}
        ),
    }
    for package in PACKAGES:
        texts[f"src/rt_control/{package}/package.xml"] = (
            f"<package><name>{package}</name></package>"
        )
    texts.update({"src/rt_control/" + path: text for path, text in extra.items()})
    return texts


def findings(texts, group="layers"):
    rules, config = repo.read_rules(texts)
    family, check = arch_check.GROUPS[group]
    return repo.apply_exceptions(config + check(texts, rules), rules, {family})


@pytest.mark.parametrize(
    "rule,path,accepted,rejected",
    [
        (
            "LAYER-1",
            "hw_other/package.xml",
            "<package><name>hw_other</name></package>",
            "<package><name>other</name></package>",
        ),
        ("LAYER-1", "motor/package.xml", "", "<package><name>motor</name></package>"),
        (
            "LAYER-2",
            "ctrl_motor/src/node.cpp",
            '#include "hw_semantic_components/a.hpp"',
            '#include "hw_bus/a.hpp"',
        ),
        (
            "LAYER-2",
            "ctrl_motor/tool.py",
            "from hw_semantic_components import status",
            "import hw_bus",
        ),
        (
            "LAYER-2",
            "ctrl_motor/CMakeLists.txt",
            "find_package(hw_semantic_components REQUIRED)",
            "find_package(hw_bus REQUIRED)",
        ),
        (
            "LAYER-2",
            "hw_bus/package.xml",
            "<package><name>hw_bus</name></package>",
            "<package><name>hw_bus</name><test_depend>ctrl_motor</test_depend></package>",
        ),
        (
            "LAYER-2",
            "hw_bus/tool.py",
            "import hw_semantic_components",
            "from integ_composer import x",
        ),
        (
            "LAYER-2",
            "infra_ring/include/infra_ring/ring.hpp",
            "#include <cstdint>",
            '#include "hw_semantic_components/a.hpp"',
        ),
        (
            "LAYER-2",
            "periph_bms/package.xml",
            "<package><name>periph_bms</name><depend>infra_ring</depend></package>",
            "<package><name>periph_bms</name><depend>ctrl_motor</depend></package>",
        ),
        (
            "LAYER-2",
            "ctrl_motor/src/node.cpp",
            '#include "hw_semantic_components/a.hpp"',
            '#include "periph_bms/a.hpp"',
        ),
        (
            "LAYER-3",
            "ctrl_motor/package.xml",
            "<package><name>ctrl_motor</name><exec_depend>hwcfg_machines</exec_depend></package>",
            "<package><name>ctrl_motor</name><depend>hwcfg_machines</depend></package>",
        ),
        ("LAYER-3", "hwcfg_machines/machines/x.py", "", "print(1)"),
        (
            "LAYER-4",
            "periph_bms/include/periph_bms/protocol/decoder.hpp",
            "#include <cstdint>",
            "#include <std_msgs/msg/header.hpp>",
        ),
        (
            "LAYER-5",
            "periph_bms/src/node.cpp",
            "const auto bytes = ::read(fd, &frame, sizeof(frame));",
            "const auto bytes = ::write(fd, &frame, sizeof(frame));",
        ),
    ],
)
def test_layer_rules(rule, path, accepted, rejected):
    texts = tree()
    if accepted:
        texts["src/rt_control/" + path] = accepted
    assert findings(texts) == []
    texts["src/rt_control/" + path] = rejected
    reported = findings(texts)
    assert any(f.rule == rule and f.path == "src/rt_control/" + path for f in reported), reported


@pytest.mark.parametrize(
    "rule,group,path,accepted,rejected",
    [
        (
            "OWN-1",
            "ownership",
            "integ_composer/integ_composer/compose.py",
            'cfg = {"update_rate": rate}',
            'cfg = {"update_rate": 1000}',
        ),
        (
            "OWN-1",
            "ownership",
            "hw_bus/urdf/bus.xacro",
            "<!-- control_frequency: 1000 -->",
            '<param name="control_frequency">1000</param>',
        ),
        (
            "OWN-2",
            "ownership",
            "hw_bus/rt_control_renderer.py",
            'path = "devices/motor.yaml"',
            'path = "evidence/observation.yaml"',
        ),
        (
            "NAME-1",
            "names",
            "integ_composer/integ_composer/compose.py",
            'name = "synthetic"',
            'name = "demo-v9"',
        ),
        (
            "NAME-1",
            "names",
            "integ_composer/web/app.js",
            "load('model.urdf')",
            "load('robot_demo_v9.urdf')",
        ),
        (
            "NAME-2",
            "names",
            "ctrl_motor/src/node.cpp",
            "// the head motors",
            "// the AcmeMotors head",
        ),
        (
            "NAME-3",
            "names",
            "integ_composer/config/poses.yaml",
            "pose: {}",
            "pose: {arm_joint1: 0.5}",
        ),
    ],
)
def test_content_rules(rule, group, path, accepted, rejected):
    texts = tree(**{path: accepted})
    assert findings(texts, group) == []
    texts["src/rt_control/" + path] = rejected
    assert [f.rule for f in findings(texts, group)] == [rule]


def test_frequency_in_manifest_and_composer_defaults_is_the_owner():
    texts = tree(
        **{
            "hwcfg_machines/machines/V/manifest.yaml": "update_rate: 1000\n",
            "integ_composer/config/controller_defaults.yaml": "update_rate: 1000\n",
            "hw_bus/test/test_rate.py": 'cfg = {"update_rate": 1000}',
        }
    )
    assert findings(texts, "ownership") == []


def test_exception_covers_its_file_and_a_stale_one_is_reported():
    path = "integ_composer/integ_composer/compose.py"
    texts = tree(**{path: 'name = "Demo_V9"'})
    approved = {
        "rule": "NAME-1",
        "path": "src/rt_control/" + path,
        "reason": "synthetic",
        "approved": "user",
    }
    texts[repo.RULES_FILE] = yaml.safe_dump(
        {"exceptions": [approved], "listen_only_peripherals": [], "vendor_names": []}
    )
    assert findings(texts, "names") == []
    texts["src/rt_control/" + path] = 'name = "synthetic"'
    assert [f.rule for f in findings(texts, "names")] == ["ARCH-CONFIG"]
    # Exceptions of rule families that did not run are not judged.
    assert findings(texts, "layers") == []


@pytest.mark.parametrize(
    "rules_text",
    [
        "exceptions: {}\n",
        yaml.safe_dump({"exceptions": [{"rule": "NAME-1", "path": "x"}]}),
        yaml.safe_dump({"listen_only_peripherals": ["hw_bus"]}),
    ],
)
def test_invalid_rules_file_is_reported(rules_text):
    texts = tree()
    texts[repo.RULES_FILE] = rules_text
    assert [f.rule for f in findings(texts)] == ["ARCH-CONFIG"]


def test_layer_comes_from_the_package_prefix():
    assert [repo.layer_of(p) for p in PACKAGES] == [
        "infra",
        "hw_config",
        "protocol",
        "protocol",
        "control",
        "periph",
        "integration",
    ]
    assert repo.layer_of("robot_description") is None


def test_shared_packages_may_not_build_depend_on_the_data_package():
    texts = tree()
    texts["src/description/robot_description/package.xml"] = (
        "<package><name>robot_description</name><depend>hwcfg_machines</depend></package>"
    )
    assert [f.rule for f in findings(texts)] == ["LAYER-3"]


def write_repository(root: Path, texts):
    for path, text in texts.items():
        (root / path).parent.mkdir(parents=True, exist_ok=True)
        (root / path).write_text(text, encoding="utf-8")
    subprocess.run(["git", "init", "-q", str(root)], check=True)


def test_command_line_reports_findings_and_exit_codes(tmp_path, capsys):
    write_repository(tmp_path, tree())
    assert arch_check.main(["--root", str(tmp_path)]) == 0
    assert "PASS: arch_check" in capsys.readouterr().out

    (tmp_path / "src/rt_control/ctrl_motor/src").mkdir(parents=True)
    (tmp_path / "src/rt_control/ctrl_motor/src/node.cpp").write_text('#include "hw_bus/a.hpp"\n')
    assert arch_check.main(["--root", str(tmp_path), "layers"]) == 1
    out = capsys.readouterr().out
    assert out.startswith("src/rt_control/ctrl_motor/src/node.cpp:1: LAYER-2 ")
    assert arch_check.main(["--root", str(tmp_path), "--json", "layers"]) == 1
    assert '"rule": "LAYER-2"' in capsys.readouterr().out
    with pytest.raises(SystemExit):
        arch_check.main(["--root", str(tmp_path), "nonexistent"])


def test_tool_failure_exits_two(tmp_path):
    assert arch_check.main(["--root", str(tmp_path / "missing")]) == 2


def test_dependencies_are_read_from_every_declaration_kind():
    kinds = {
        kind
        for _, kind, _, _ in layers.dependencies(
            "src/rt_control/a/package.xml",
            "<package><depend>x</depend><test_depend>y</test_depend></package>",
        )
    }
    assert kinds == {"depend", "test_depend"}


@pytest.mark.parametrize(
    "path,text,reported",
    [
        ("infra_ring/src/socket.cpp", "addr.can_family = AF_CAN;", False),
        ("hw_bus/src/bus.cpp", "addr.can_family = AF_CAN;", True),
        ("hw_bus/src/bus.cpp", "// opened by infra_ring (AF_CAN)", False),
        ("hw_bus/test/test_bus.cpp", "addr.can_family = AF_CAN;", False),
        ("infra_ring/src/socket.cpp", "auto bits = encode_double(x);", True),
    ],
)
def test_duplicate_signatures_only_in_their_owner(path, text, reported):
    texts = tree(**{path: text})
    assert [f.rule for f in findings(texts, "duplicates")] == (["DUP-1"] if reported else [])


def test_invalid_duplicate_signature_is_reported():
    texts = tree()
    texts[repo.RULES_FILE] = yaml.safe_dump(
        {"duplicates": [{"name": "x", "owner": "infra_ring", "pattern": "("}]}
    )
    assert [f.rule for f in findings(texts, "duplicates")] == ["ARCH-CONFIG"]


@pytest.mark.parametrize(
    "path,lines,reported",
    [
        ("hw_bus/src/bus.cpp", 1000, False),
        ("hw_bus/src/bus.cpp", 1001, True),
        ("integ_composer/integ_composer/compose.py", 601, True),
        ("hwcfg_machines/machines/Demo_V9/controllers.yaml", 401, True),
        ("integ_composer/test/test_compose.py", 2000, False),
        ("integ_composer/test/golden/arm/controllers.yaml", 900, False),
        ("integ_composer/web/presets.json", 5000, False),
    ],
)
def test_file_size_limits(path, lines, reported):
    texts = tree(**{path: "x\n" * lines})
    assert [f.rule for f in findings(texts, "size")] == (["SIZE-1"] if reported else [])


def test_compose_comparison_reports_lost_selections_and_changed_files():
    from tools.archcheck import compose_against

    files = {"controllers.yaml": "a: 1\nb: 2\n", "robot.xml": "<robot/>"}
    base = {
        "V/U/p/s/mock": {"ok": True, "files": dict(files)},
        "V/U/p/s/real": {"ok": True, "files": dict(files)},
        "V/U/p/t/mock": {"ok": False, "error": "ValueError: not composable"},
        "V/U/p/gone/mock": {"ok": True, "files": dict(files)},
    }
    current = {
        "V/U/p/s/mock": {
            "ok": True,
            "files": {"controllers.yaml": "a: 1\nb: 3\n", "diagnostics.yaml": "devices: []\n"},
        },
        "V/U/p/s/real": {"ok": False, "error": "KeyError: 'x'"},
        "V/U/p/t/mock": {"ok": False, "error": "ValueError: not composable"},
    }
    found = {(f.path, f.line): f.message for f in compose_against.compare(base, current)}
    assert found[("V/U/p/s/mock/controllers.yaml", 2)] == "产物不同：b: 2 → b: 3"
    assert found[("V/U/p/s/mock/diagnostics.yaml", 0)] == "产物文件新增"
    assert found[("V/U/p/s/mock/robot.xml", 0)] == "产物文件删除"
    assert found[("V/U/p/s/real", 0)] == "基线可组合，当前失败：KeyError: 'x'"
    assert found[("V/U/p/gone/mock", 0)] == "基线可组合，当前失败：选择已不存在"
    assert len(found) == 5
    assert compose_against.compare(base, base) == []
