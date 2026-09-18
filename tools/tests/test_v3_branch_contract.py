from pathlib import Path

from tools.check_v3_branch_contract import validate


ROOT = Path(__file__).resolve().parents[2]


def test_repository_satisfies_v3_branch_contract():
    assert validate(ROOT) == []
