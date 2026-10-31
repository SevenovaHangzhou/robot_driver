import json

from integ_diagnostics.report import write_self_check


def test_self_check_report_is_written_whether_or_not_it_blocks(tmp_path):
    warning = {"source": "arms/left_joint1/drive", "blocking": False, "detail": "0xA000"}
    path = write_self_check("arms_only", "/artifact", [warning], directory=tmp_path)
    report = json.loads(path.read_text())
    assert path == tmp_path / "self_check/arms_only.json"
    assert report["passed"] and report["entries"] == [warning]
    write_self_check("arms_only", "/artifact", [{**warning, "blocking": True}], directory=tmp_path)
    assert not json.loads(path.read_text())["passed"]
    write_self_check(
        "arms_only", "/artifact", [warning], directory=tmp_path, stage="after_activation"
    )
    assert json.loads(path.read_text())["stage"] == "after_activation"
