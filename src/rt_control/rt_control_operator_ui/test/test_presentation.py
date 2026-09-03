import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.battery import project_battery  # noqa: E402
from rt_control_operator_ui.cpu_metrics import (  # noqa: E402
    CpuCounters,
    CpuUsageHistory,
    parse_proc_stat,
)


def test_battery_projects_fraction_to_percent_and_voltage() -> None:
    display = project_battery(
        present=True, percentage=0.8, voltage=50.5, age_s=1.0
    )

    assert display.state == "ok"
    assert display.percent_text == "80%"
    assert display.voltage_text == "50.5 V"
    assert display.fraction == 0.8


def test_battery_distinguishes_offline_stale_and_invalid() -> None:
    assert project_battery(False, 0.8, 50.5, 1.0).state == "offline"
    assert project_battery(True, 0.8, 50.5, 6.1).state == "stale"

    invalid = project_battery(True, math.nan, 50.5, 1.0)
    assert invalid.state == "invalid"
    assert invalid.percent_text == "--%"


def test_parse_proc_stat_selects_cpu14_and_counts_iowait_as_idle() -> None:
    counters = parse_proc_stat(
        "cpu  1 2 3 4 5 6 7 8\n"
        "cpu13 10 20 30 40 50 60 70 80\n"
        "cpu14 100 20 30 400 50 10 5 2\n",
        cpu=14,
    )

    assert counters == CpuCounters(total=617, idle=450)


def test_cpu_history_reports_current_average_peak_and_threshold() -> None:
    history = CpuUsageHistory(max_samples=60)

    assert history.ingest(CpuCounters(total=100, idle=80)) is None
    first = history.ingest(CpuCounters(total=200, idle=130))
    second = history.ingest(CpuCounters(total=300, idle=150))

    assert first is not None and first.current_percent == 50.0
    assert second is not None and second.current_percent == 80.0
    assert second.average_10s_percent == 65.0
    assert second.peak_60s_percent == 80.0
    assert second.level == "alarm"


def test_cpu_history_rejects_counter_reset_without_fabricating_usage() -> None:
    history = CpuUsageHistory()
    history.ingest(CpuCounters(total=100, idle=80))

    assert history.ingest(CpuCounters(total=90, idle=70)) is None
