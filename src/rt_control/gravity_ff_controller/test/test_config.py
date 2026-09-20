from pathlib import Path
import yaml


def test_v3_draft_preserves_source_parameters_without_storing_derived_armature():
    config = yaml.safe_load((Path(__file__).parents[1] / "config/gravity_ff.draft.yaml").read_text())
    assert config["verified"] is False
    model = config["shared_model"]
    rows = model["source_joint_parameters"]
    assert [row["joint"] for row in rows] == [f"joint{i}" for i in range(1, 8)]
    assert [row["input_inertia_g_mm2"] for row in rows] == model["input_inertia_g_mm2"]
    assert [row["reduction_ratio"] for row in rows] == model["reduction_ratio"]
    assert all("armature" not in row for row in rows)
    assert "armature" not in model
    assert set(config["controllers"]) == {"right_gravity_ff", "left_gravity_ff"}
    assert all(controller["mode"] == "shadow" for controller in config["controllers"].values())
    assert all(controller["max_effort_nm"] == "TBD" for controller in config["controllers"].values())
