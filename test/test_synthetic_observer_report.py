"""The reporter must be reproducible and self-describing.

Accuracy *bounds* are deliberately not asserted here. A bound belongs with the
geometry it was measured on, and pinning one in the same commit that introduces
the instrument would freeze figures taken before the geometry it measures is
settled -- which is the mistake this tool exists to stop. What is asserted is
what must hold regardless: the same schedule produces the same numbers, and the
artifact states the schedule and provenance rather than leaving them to whoever
ran it.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from synthetic_observer_report import Schedule, run_report  # noqa: E402

# One profile, one speed, a short window and no raycast audit: enough to
# exercise the whole path in a few seconds. The published artifact runs the
# full declared schedule.
FAST = Schedule(rate_hz=15.0, speeds_mps=(1.0,), window_x_m=(0.0, 5.0))


@pytest.fixture(scope="module")
def report(tmp_path_factory: pytest.TempPathFactory) -> dict:
    return run_report(
        tmp_path_factory.mktemp("report"),
        schedule=FAST,
        profiles=("nominal_m6_n3",),
        check_visibility=False,
    )


def test_report_declares_its_sampling_schedule(report: dict) -> None:
    """A figure without its schedule is not reproducible; that caused R13."""

    schedule = report["schedule"]
    assert schedule["camera_rate_hz"] == 15.0
    assert schedule["path_speeds_mps"] == [1.0]
    assert schedule["window_x_m"] == [0.0, 5.0]
    assert "not a station grid" in schedule["sampling"]

    provenance = report["provenance"]
    for key in ("tool", "commit", "python", "numpy", "opencv", "scene", "renderer"):
        assert provenance[key], f"provenance is missing {key}"


def test_speed_error_is_primary_and_station_error_secondary(report: dict) -> None:
    block = report["profiles"]["nominal_m6_n3"]
    assert block["max_gate_speed_error_mps"] is not None
    assert block["max_station_error_m"] is not None
    run = block["runs"][0]
    assert run["accepted_frames"] > 0
    assert run["gate_measurements"], "the window must contain at least one gate pair"
    for measurement in run["gate_measurements"]:
        assert measurement["speed_error_mps"] >= 0.0
        assert measurement["speed_limit_mps"] > 0.0
    for sample in run["samples"]:
        assert sample["accepted_marker_ids"]
        assert sample["correspondence_rank"] == 3


def test_the_same_schedule_reproduces_the_same_numbers(
    tmp_path: Path, report: dict
) -> None:
    """Re-running must not move the figures, or citing them means nothing."""

    again = run_report(
        tmp_path / "again",
        schedule=FAST,
        profiles=("nominal_m6_n3",),
        check_visibility=False,
    )
    assert again["profiles"] == report["profiles"]
    assert again["summary"] == report["summary"]
