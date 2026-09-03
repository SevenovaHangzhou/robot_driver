import sys
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.fault_catalog import (  # noqa: E402
    CatalogValidationError,
    FaultCatalog,
)


def test_production_catalog_keeps_vendor_error_codes_separate() -> None:
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")

    zeroerr = catalog.lookup("ZeroErr", 0xA000)
    ti5 = catalog.lookup("TI5", 0x7500)

    assert zeroerr.title_zh == "主站掉线"
    assert zeroerr.reset_policy == "reset_once_after_link_recovered"
    assert zeroerr.manual_version == "V1.9"
    assert zeroerr.manual_pages == "119,142"

    assert ti5.title_zh == "通信错误"
    assert ti5.reset_policy == "reset_once_after_link_recovered"
    assert ti5.manual_version == "1.1.1"
    assert ti5.manual_pages == "77-78"
    assert "OP" in " ".join(ti5.possible_causes)


def test_unknown_code_is_preserved_without_inventing_an_explanation() -> None:
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")

    unknown = catalog.lookup("TI5", 0xDEAD)

    assert unknown.code == 0xDEAD
    assert unknown.title_zh == "未知厂商错误"
    assert unknown.reset_policy == "do_not_reset"
    assert unknown.known is False


@pytest.mark.parametrize(
    "mutation, expected_fragment",
    [
        ({"source_reference": ""}, "source_reference"),
        ({"title_zh": ""}, "title_zh"),
        ({"reset_policy": "retry_forever"}, "reset_policy"),
        ({"severity": "fatalish"}, "severity"),
    ],
)
def test_catalog_rejects_unverifiable_or_invalid_entries(
    tmp_path: Path, mutation: dict[str, str], expected_fragment: str
) -> None:
    entry = {
        "vendor": "TI5",
        "code": "0x7500",
        "title_zh": "通信错误",
        "severity": "error",
        "reset_policy": "reset_once_after_link_recovered",
        "possible_causes": ["异常从 OP 切换到非 OP"],
        "operator_actions": ["确认 EtherCAT Link UP"],
        "manual_title": "钛虎 C1 关节模组通讯使用说明",
        "manual_version": "1.1.1",
        "manual_pages": "77-78",
        "source_reference": "manual-sha256:test",
    }
    entry.update(mutation)
    (tmp_path / "ti5.yaml").write_text(
        yaml.safe_dump({"faults": [entry]}, allow_unicode=True), encoding="utf-8"
    )

    with pytest.raises(CatalogValidationError, match=expected_fragment):
        FaultCatalog.load_directory(tmp_path)


def test_catalog_rejects_duplicate_vendor_and_code(tmp_path: Path) -> None:
    entry = {
        "vendor": "TI5",
        "code": "0x7500",
        "title_zh": "通信错误",
        "severity": "error",
        "reset_policy": "reset_once_after_link_recovered",
        "possible_causes": ["异常从 OP 切换到非 OP"],
        "operator_actions": ["确认 EtherCAT Link UP"],
        "manual_title": "钛虎 C1 关节模组通讯使用说明",
        "manual_version": "1.1.1",
        "manual_pages": "77-78",
        "source_reference": "manual-sha256:test",
    }
    (tmp_path / "ti5.yaml").write_text(
        yaml.safe_dump({"faults": [entry, entry]}, allow_unicode=True),
        encoding="utf-8",
    )

    with pytest.raises(CatalogValidationError, match="duplicate"):
        FaultCatalog.load_directory(tmp_path)
