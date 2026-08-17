"""The run manifest, and the three ways it was asked to lose information.

Each case below is a mixing failure that actually happened in
`out/evidence/robot-a-gate/`, expressed as an assertion instead of as a
directory nobody could untangle afterwards.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from run_manifest import (  # noqa: E402
    CLASSIFICATIONS,
    add_error,
    classify,
    file_digest,
    load,
    merge,
)


def test_a_later_write_does_not_erase_an_earlier_one(tmp_path: Path) -> None:
    """Start records what it knows; the end finalises. Neither may destroy the other."""

    path = tmp_path / "run.json"
    merge(path, {"robot": "robot1", "profile": "nominal_m6_n3"})
    merge(path, {"exit_status": 0})

    payload = load(path)
    assert payload["robot"] == "robot1"
    assert payload["profile"] == "nominal_m6_n3"
    assert payload["exit_status"] == 0


def test_errors_accumulate_rather_than_replace(tmp_path: Path) -> None:
    path = tmp_path / "run.json"
    merge(path, {"robot": "robot1"})
    add_error(path, "slam_toolbox missed its lifecycle response")
    add_error(path, "map save failed")

    payload = load(path)
    assert [entry["message"] for entry in payload["errors"]] == [
        "slam_toolbox missed its lifecycle response",
        "map save failed",
    ]
    assert payload["robot"] == "robot1"


def test_the_first_classification_wins(tmp_path: Path) -> None:
    """A teardown running after an infrastructure exit must not relabel it.

    The exit-3 path fires first and says `rerun`; the exit trap fires second
    and would otherwise stamp `crash` over it, turning a rerun into a verdict.
    """

    path = tmp_path / "run.json"
    classify(path, "rerun", "contract precondition failed")
    classify(path, "crash", "run ended without a verdict")

    payload = load(path)
    assert payload["classification"] == "rerun"
    assert payload["classification_cause"] == "contract precondition failed"
    # The disagreement stays visible instead of being silently dropped.
    assert len(payload["classification_attempts"]) == 2
    assert payload["classification_attempts"][1]["classification"] == "crash"


def test_an_unclassified_run_has_no_verdict_at_all(tmp_path: Path) -> None:
    """The negative control for the default.

    Before this, a run that died mid-way left the previous run's artifacts in
    place and no trace of itself. The absence of a classification is what the
    caller's exit trap turns into `crash`, so it must be distinguishable from
    every real verdict rather than defaulting to one.
    """

    path = tmp_path / "run.json"
    merge(path, {"robot": "robot1"})
    assert load(path).get("classification") is None


def test_classification_is_one_of_three_and_pass_is_separate(tmp_path: Path) -> None:
    """A red result and a crash are different objects."""

    assert CLASSIFICATIONS == ("result", "rerun", "crash")
    with pytest.raises(ValueError):
        classify(tmp_path / "run.json", "FAILED")


def test_a_digest_identifies_the_scenario_that_was_run(tmp_path: Path) -> None:
    """The arena and the plan came apart, and no artifact could show it.

    Two files with the same NAME and different content must not be
    interchangeable in a record, which is what a path alone allows.
    """

    first = tmp_path / "arena.usd"
    second = tmp_path / "arena-other.usd"
    first.write_bytes(b"twelve metre corridor")
    second.write_bytes(b"three point six metre corridor")

    assert file_digest(first) != file_digest(second)
    assert file_digest(first) == file_digest(first)
    assert file_digest(tmp_path / "absent.usd") is None


def test_a_corrupt_manifest_does_not_take_the_run_down(tmp_path: Path) -> None:
    """Fail-open: recording a problem must never become the problem."""

    path = tmp_path / "run.json"
    path.write_text("{not json", encoding="utf-8")
    merge(path, {"robot": "robot1"})

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["robot"] == "robot1"
    assert any("unreadable" in entry["message"] for entry in payload["errors"])


# ------------------------------------------------- phases become covariates
#
# `phase()` writes start stamps (`HH:MM:SS +Ns <name>`) and nothing in the
# tree parsed them: the 2026-08-14 bring-up attribution was differenced by
# hand from seven logs. `phases_from_log` is that differencing, once, tested.


PHASES_LOG = """\
13:39:23 +0s corridor profile run: nominal_m6_n3 (reported only)
13:39:23 +0s simctl start
13:40:35 +72s lens
13:40:43 +80s precondition: robot1 contract (--domain 67 --seconds 8)
13:41:11 +108s nav stack
"""


def test_phase_stamps_become_durations(tmp_path: Path) -> None:
    from run_manifest import phases_from_log

    entries = phases_from_log(PHASES_LOG, total_s=130)

    assert [e["name"] for e in entries][1:3] == ["simctl start", "lens"]
    by_name = {e["name"]: e for e in entries}
    assert by_name["simctl start"]["at_s"] == 0
    assert by_name["simctl start"]["duration_s"] == 72
    assert by_name["lens"]["duration_s"] == 8
    # The last phase runs to the run's own end, not to a guess.
    assert by_name["nav stack"]["duration_s"] == 22


def test_the_last_phase_without_a_total_says_none_not_zero(tmp_path: Path) -> None:
    """A crash that never learned the run's length must not fabricate one."""

    from run_manifest import phases_from_log

    entries = phases_from_log(PHASES_LOG, total_s=None)
    assert entries[-1]["duration_s"] is None


def test_a_phase_log_with_noise_is_parsed_not_fatal(tmp_path: Path) -> None:
    """It is a log; a stray line must cost nothing but itself."""

    from run_manifest import phases_from_log

    noisy = "garbage line\n" + PHASES_LOG + "=== banner ===\n"
    entries = phases_from_log(noisy, total_s=130)
    assert len(entries) == 5


def test_record_phases_merges_and_fails_open(tmp_path: Path) -> None:
    from run_manifest import load, record_phases

    path = tmp_path / "run.json"
    merge(path, {"robot": "robot1"})
    log = tmp_path / "phases.log"
    log.write_text(PHASES_LOG, encoding="utf-8")

    record_phases(path, log, total_s=130)
    payload = load(path)
    assert payload["robot"] == "robot1", "phases must merge, never replace"
    assert len(payload["phases"]) == 5

    # An absent log is the crash-before-first-phase case: no write, no raise.
    before = load(path)
    record_phases(path, tmp_path / "absent.log", total_s=130)
    assert load(path) == before


def test_the_runner_records_the_shm_census_and_the_phases() -> None:
    """The runner's side of the covariates: three census points (before Isaac,
    after Isaac, at teardown) and the phases call on the one exit path every
    run takes. Structural, because the alternative is a full Isaac run."""

    runner = (ROOT / "tools/corridor_profile_run.sh").read_text(encoding="utf-8")

    for field in ("shm_fastrtps_pre", "shm_fastrtps_post_simctl",
                  "shm_fastrtps_teardown"):
        assert f"{field}=$(shm_fastrtps_count)" in runner, field
    # The census must count semaphores too -- _dds_shm.py's globs, both of them.
    assert "/dev/shm/fastrtps*" in runner and "/dev/shm/sem.fastrtps*" in runner
    # And the phases land in run.json from record_exit, which every path runs.
    record_exit = runner[runner.index("record_exit() {"):]
    record_exit = record_exit[:record_exit.index("\n}")]
    assert "phases" in record_exit and "--log" in record_exit
