"""Exercise the external model contract using real local Git repositories."""

import subprocess

import pytest
import yaml

from tools import robot_model
from tools.archcheck import compose_against


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def commit(repo):
    git(repo, "add", ".")
    git(
        repo,
        "-c",
        "user.name=Model Test",
        "-c",
        "user.email=test@example.invalid",
        "commit",
        "-qm",
        "fixture",
    )
    return git(repo, "rev-parse", "HEAD")


@pytest.fixture
def checkout(tmp_path):
    driver = tmp_path / "driver"
    model = tmp_path / "robot_description"
    driver.mkdir()
    model.mkdir()
    git(model, "init", "-q")
    (model / "package.xml").write_text("<package><name>robot_description</name></package>\n")
    (model / "model.urdf").write_text('<robot name="model"><link name="base"/></robot>\n')
    revision = commit(model)
    (driver / "deps.repos").write_text(
        yaml.safe_dump(
            {
                "repositories": {
                    "robot_description": {
                        "type": "git",
                        "url": "https://github.com/SevenovaHangzhou/robot_description.git",
                        "version": revision,
                    }
                }
            }
        )
    )
    return driver, model, revision


def test_clean_checkout_and_linked_worktree_are_accepted(checkout, tmp_path):
    driver, model, revision = checkout
    assert robot_model.verify(driver) == model
    linked_driver = tmp_path / "linked" / "driver"
    linked_driver.mkdir(parents=True)
    (linked_driver / "deps.repos").write_bytes((driver / "deps.repos").read_bytes())
    linked_model = linked_driver.parent / "robot_description"
    git(model, "worktree", "add", "--detach", str(linked_model), revision)
    assert robot_model.verify(linked_driver) == linked_model


@pytest.mark.parametrize("change", ["tracked", "untracked", "commit"])
def test_verify_preserves_existing_changes_and_wrong_revision(checkout, change):
    driver, model, _ = checkout
    path = model / ("model.urdf" if change != "untracked" else "notes.txt")
    path.write_text("user data\n")
    if change == "commit":
        commit(model)
    before = git(model, "rev-parse", "HEAD"), git(model, "status", "--porcelain"), path.read_bytes()
    with pytest.raises(ValueError, match="preserved"):
        robot_model.verify(driver)
    assert before == (
        git(model, "rev-parse", "HEAD"),
        git(model, "status", "--porcelain"),
        path.read_bytes(),
    )


def test_missing_model_is_reported_without_creating_it(checkout, tmp_path):
    driver, model, _ = checkout
    model.rename(tmp_path / "authority")
    with pytest.raises(ValueError, match="place the pinned repository"):
        robot_model.verify(driver)
    assert not model.exists()


def test_vendor_manifest_excludes_only_the_supplied_model(checkout):
    driver, _, _ = checkout
    document = yaml.safe_load((driver / "deps.repos").read_text())
    vendor = {"type": "git", "url": "https://example.invalid/driver", "version": "a" * 40}
    document["repositories"]["src/vendor/driver"] = vendor
    (driver / "deps.repos").write_text(yaml.safe_dump(document))
    assert robot_model.vendor_manifest(driver) == {"repositories": {"src/vendor/driver": vendor}}
    assert yaml.safe_load((driver / "deps.repos").read_text()) == document


def test_verify_does_not_replace_an_existing_non_git_directory(checkout):
    driver, model, _ = checkout
    (model / ".git").rename(model / "saved-git")
    before = (model / "model.urdf").read_bytes()
    with pytest.raises(ValueError, match="missing model checkout"):
        robot_model.verify(driver)
    assert (model / "model.urdf").read_bytes() == before


def test_comparison_exports_the_base_model_revision_not_current_checkout(checkout, tmp_path):
    driver, model, _ = checkout
    git(driver, "init", "-q")
    (driver / "src/rt_control").mkdir(parents=True)
    (driver / "src/rt_control/marker").write_text("driver\n")
    ref = commit(driver)
    original = (model / "model.urdf").read_bytes()
    (model / "model.urdf").write_text('<robot name="new"/>\n')
    commit(model)
    destination = tmp_path / "export"
    destination.mkdir()
    compose_against._export(driver, ref, destination)
    assert (destination / "robot_description/model.urdf").read_bytes() == original
    assert (model / "model.urdf").read_bytes() != original
