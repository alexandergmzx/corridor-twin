"""The check that the arena and the plan are the same scenario.

Both directions, on real stages: it must accept the arena built from the
manifest, and it must reject the 12 m arena that every corridor run after the
rescale actually loaded.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/corridor_scene"))

from check_arena_matches_manifest import TOLERANCE_M, compare  # noqa: E402

NOMINAL = "nominal_m6_n3"
MANIFEST = ROOT / "out/corridor.manifest.json"
CURRENT_ARENA = ROOT / f"out/arena_corridor_robot1_{NOMINAL}.usd"
#: The pre-rescale arena, kept on disk. It is the negative control: if this
#: ever starts passing, the check has stopped checking anything.
STALE_ARENA = ROOT / f"out/arena_corridor_{NOMINAL}.usd"


def _skip_unless(*paths: Path) -> None:
    for path in paths:
        if not path.exists():
            pytest.skip(f"{path.name} is a generated artifact and is not present")


def test_the_rebuilt_arena_agrees_with_the_manifest() -> None:
    _skip_unless(MANIFEST, CURRENT_ARENA)

    report = compare(CURRENT_ARENA, MANIFEST, NOMINAL)
    assert report["pass"], report["failures"]

    named = {check["name"] for check in report["checks"]}
    assert "B" in named
    assert "B's landmark post" in named, "the post is the thing the stale arena lacked"
    assert all(check["error_m"] <= TOLERANCE_M for check in report["checks"])


def test_the_stale_twelve_metre_arena_is_rejected() -> None:
    """The exact fault, on the exact file that caused it.

    B is 11.76 m from where the plan says it is, the landmark post is absent
    from the stage entirely, and the corridor's east kerb is 12.95 m out.
    """

    _skip_unless(MANIFEST, STALE_ARENA)

    report = compare(STALE_ARENA, MANIFEST, NOMINAL)
    assert not report["pass"]

    by_name = {check["name"]: check for check in report["checks"]}
    assert by_name["B"]["error_m"] > 10.0
    assert by_name["B's landmark post"]["measured"] is None
    assert "absent from the arena" in by_name["B's landmark post"]["note"]


def test_a_missing_arena_fails_rather_than_raises(tmp_path: Path) -> None:
    """A precondition that dies with a traceback is a precondition nobody reads."""

    _skip_unless(MANIFEST)

    report = compare(tmp_path / "absent.usd", MANIFEST, NOMINAL)
    assert not report["pass"]
    assert report["failures"]


def test_the_runner_checks_before_it_spends_isaac_time() -> None:
    """Ordering matters: after simctl start this costs a session to discover."""

    runner = (ROOT / "tools/corridor_profile_run.sh").read_text(encoding="utf-8")
    check_at = runner.index("check_arena_matches_manifest.py")
    start_at = runner.index('"$SIMCTL" start')
    assert check_at < start_at, "the arena check must run before the simulator starts"


def test_the_artifact_records_what_was_compared() -> None:
    """A gate number that exists only in prose is not evidence (the F15 lesson)."""

    _skip_unless(MANIFEST, CURRENT_ARENA)

    report = compare(CURRENT_ARENA, MANIFEST, NOMINAL)
    round_tripped = json.loads(json.dumps(report))
    assert round_tripped["tolerance_m"] == TOLERANCE_M
    assert round_tripped["arena"].endswith(".usd")
    assert round_tripped["manifest"].endswith(".json")
