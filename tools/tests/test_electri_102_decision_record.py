import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DECISION_RECORD = (
    ROOT / "domains/rt_control/docs/electri-102-clarification-decisions.md"
)
DECISION_HEADING = re.compile(r"^### (E102-D(?P<number>\d{2})) — ", re.MULTILINE)
OPTION_MARKER = re.compile(r"^- \*\*【[^】]*\b[A-Z]】\*\*", re.MULTILINE)


def decision_sections(text: str) -> list[tuple[str, int, str]]:
    headings = list(DECISION_HEADING.finditer(text))
    sections = []
    for index, heading in enumerate(headings):
        end = headings[index + 1].start() if index + 1 < len(headings) else len(text)
        sections.append(
            (
                heading.group(1),
                int(heading.group("number")),
                text[heading.end() : end],
            )
        )
    return sections


def test_every_electri_102_decision_has_options_recommendation_and_outcome():
    sections = decision_sections(DECISION_RECORD.read_text(encoding="utf-8"))

    assert [number for _, number, _ in sections] == list(range(1, 48))
    for decision_id, _, body in sections:
        assert "**问题**：" in body, f"{decision_id} lacks an explicit question"
        assert len(OPTION_MARKER.findall(body)) >= 2, (
            f"{decision_id} must record at least two decision options"
        )
        assert re.search(r"【[^】]*推荐[^】]*】", body), (
            f"{decision_id} does not mark the recommended option"
        )
        assert re.search(r"【[^】]*最终采用[^】]*】", body), (
            f"{decision_id} does not mark the final adopted option"
        )
