from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def _workflow(name: str):
    with (ROOT / name).open(encoding="utf-8") as stream:
        return yaml.load(stream, Loader=yaml.BaseLoader)


def _named_step(job, name: str):
    return next(step for step in job["steps"] if step.get("name") == name)


def test_full_build_uses_refreshable_separate_caches():
    build = _workflow(".github/workflows/rt-control-ci.yml")["jobs"]["build"]
    vendor = _named_step(build, "Cache pinned vendor checkout")
    apt = _named_step(build, "Cache apt archives")
    compiler = _named_step(build, "Cache compiler artifacts")

    assert vendor["with"]["path"] == ".ci/vendor"
    assert apt["with"]["path"] == ".ci/apt"
    assert compiler["with"]["path"] == ".ci/ccache"
    assert "github.sha" in apt["with"]["key"]
    assert "github.sha" in compiler["with"]["key"]
    assert "rt-control-apt-" in apt["with"]["restore-keys"]
    assert "rt-control-ccache-" in compiler["with"]["restore-keys"]


def test_ci_base_image_tag_is_bound_to_repository_commit():
    publish = _workflow(".github/workflows/rt-control-ci-base.yml")["jobs"]["publish"]
    build = _named_step(publish, "Build and publish pinned CI base")
    assert "steps.versions.outputs.igh_commit" in build["with"]["tags"]
    assert "github.sha" in build["with"]["tags"]
