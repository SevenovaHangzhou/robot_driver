from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_main_source_changes_do_not_require_docker_packaging():
    root_contract = _read("AGENTS.md")
    collaboration = _read("collaboration-and-commit-standards.md")
    readme = _read("README.md")

    assert "普通源码 PR 进入 `main` 不再要求同步构建 Docker 镜像" in root_contract
    assert "普通源码 PR 不要求同步 Docker 封装或容器证据" in collaboration
    assert "稳定后人工封装" in readme
    assert "合并到 `main` 的变更必须给出镜像构建与容器内启动证据" not in root_contract
    assert "合并到 `main` 的 PR 必须提供容器构建与容器内启动证据" not in collaboration


def test_manual_packaging_still_requires_traceable_container_evidence():
    root_contract = _read("AGENTS.md")
    collaboration = _read("collaboration-and-commit-standards.md")
    domain_contract = _read("domains/rt_control/AGENTS.md")

    assert "明确记录 source SHA、镜像身份并完成容器验证" in root_contract
    assert "人工 Docker 封装或正式发布" in collaboration
    assert "镜像构建、容器内启动和镜像身份" in collaboration
    assert "只有人工封装/发布任务才要求完整镜像构建" in domain_contract


def test_container_smoke_is_scoped_to_packaged_release_images():
    catalog = _read("domains/rt_control/testing/cases/readonly-runtime.yaml")

    assert "人工封装的发布镜像" in catalog
    assert "规范要求的 main 容器 smoke" not in catalog
