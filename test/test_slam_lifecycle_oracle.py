"""The SLAM readiness oracle, replayed over real launch logs.

The runner used to decide slam_toolbox's readiness with `ros2 lifecycle get`,
which needs the ros2 daemon -- and simctl stops that daemon at the end of every
run, so the call can block for seconds and return nothing. A failing attempt
therefore burned the whole 110 s deadline. Healthy activation takes **1.26 s**
(measured, run 20260814-031922), so the deadline was 87x the signal.

The log says the same thing, immediately and without a daemon. This file is the
evidence for that claim: the two markers are replayed over every archived
`slam-attempt*.log` on the box, and separately over six promoted ones so the
test still means something on a clean checkout.

Zero Isaac. This is the strongest test in the bring-up work and it costs nothing.
"""

from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools/corridor_profile_run.sh"
PROMOTED = ROOT / "docs/evidence/robot-a-gate/slam-lifecycle-logs"
SCRATCH = ROOT / "out/evidence/robot-a-gate"

#: Lifted from the runner so the test cannot drift from the code it checks.
READY = "Managed nodes are active"
FAILED = "failed to send response to /slam_toolbox/change_state"


def _verdict(text: str) -> str:
    ready, failed = READY in text, FAILED in text
    if ready and failed:
        return "BOTH"
    if ready:
        return "ready"
    if failed:
        return "failed"
    return "silent"


def test_the_runner_uses_exactly_these_two_markers() -> None:
    """If the runner's strings drift, this file is measuring nothing."""

    source = RUNNER.read_text(encoding="utf-8")
    assert f"grep -q '{READY}'" in source
    assert FAILED in source


@pytest.mark.parametrize(
    "name,expected",
    [
        ("023306-attempt1-healthy.log", "ready"),
        ("031348-attempt1-healthy.log", "ready"),
        ("031922-attempt1-healthy.log", "ready"),
        ("025555-attempt1-lost-response.log", "failed"),
        ("20260813-002222-attempt1-lost-response.log", "failed"),
        ("025555-attempt2-orphan-hang.log", "silent"),
    ],
)
def test_the_promoted_logs_are_classified_correctly(name: str, expected: str) -> None:
    """Six real launches, three outcomes, committed so this survives a clean
    checkout where `out/` (gitignored) does not exist."""

    log = PROMOTED / name
    assert log.is_file(), f"promoted evidence is missing: {name}"

    assert _verdict(log.read_text(errors="replace")) == expected


def test_the_orphan_hang_is_silent_on_both_markers() -> None:
    """**The case the group reap exists for, and the oracle's one blind spot.**

    20260814-025555 attempt 2 launched into attempt 1's un-reaped node and said
    nothing for the full 110 s: it never printed `Configuring` of its own. No
    log marker can catch that -- only not creating the orphan can, which is
    what `reap_launch_group` does. Recorded here so the oracle is not mistaken
    for complete.
    """

    text = (PROMOTED / "025555-attempt2-orphan-hang.log").read_text(errors="replace")

    assert _verdict(text) == "silent"
    assert "Configuring slam_toolbox" in text, "the manager did start the transition"
    assert text.count("Configuring") == 1, (
        "attempt 2's own node never configured; that is the hang"
    )


def test_the_oracle_never_disagrees_with_itself_across_the_whole_corpus() -> None:
    """Every archived launch on this box. Opportunistic: `out/` is gitignored,
    so this skips on a clean checkout and the promoted six carry the claim.
    """

    logs = sorted(SCRATCH.glob("*/slam-attempt*.log"))
    if len(logs) < 20:
        pytest.skip(f"only {len(logs)} archived logs here; the promoted set covers it")

    tally: dict[str, int] = {"ready": 0, "failed": 0, "silent": 0, "BOTH": 0}
    for log in logs:
        tally[_verdict(log.read_text(errors="replace"))] += 1

    assert tally["BOTH"] == 0, (
        f"{tally['BOTH']} logs claim both readiness and failure; the oracle is ambiguous"
    )
    assert tally["ready"] >= 20, f"implausibly few healthy launches: {tally}"
    assert tally["failed"] >= 1, f"the failure marker matches nothing: {tally}"
    # The silent case is the orphan hang. More than a handful would mean the
    # oracle is missing a common outcome rather than one known defect.
    assert tally["silent"] <= 2, f"the oracle is blind to too much: {tally}"
