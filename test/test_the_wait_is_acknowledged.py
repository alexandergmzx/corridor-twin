"""A long wait that says nothing is indistinguishable from a hang.

Bring-up is around two minutes and **over half of it is one silent step**.
The runner announced `simctl start` and then printed nothing of its own for
62 s while Kit loaded, which is fine once you have seen it and alarming the
first time. The operator's question during any wait is never "how long has
this taken" -- the banner already answers that -- it is *"is this normal"*,
and only a typical value answers it.

Medians are DESCRIPTIVE. Nothing branches on them, and these tests exist partly
to keep it that way: the moment a typical duration becomes a deadline it is a
second, worse watchdog competing with the real one.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools/corridor_profile_run.sh"


def _runner() -> str:
    return RUNNER.read_text(encoding="utf-8")


def test_the_longest_phase_declares_how_long_it_normally_takes() -> None:
    """`simctl start` is 52% of bring-up and the quiet part of it."""

    code = _runner()
    at = code.index("phase_typical_s() {")
    table = code[at:code.index("phase() {", at)]

    assert '"simctl start")' in table, "the longest wait has no typical duration"
    for phase in ("lens", "nav stack", "waiting for the TF chain"):
        assert f'"{phase}")' in table, f"{phase} has no typical duration"


def test_every_typical_value_is_a_plain_number_of_seconds() -> None:
    """A unit suffix or a range here would reach the banner as text and the
    reader would have to parse prose during the thing it describes."""

    code = _runner()
    at = code.index("phase_typical_s() {")
    table = code[at:code.index("phase() {", at)]

    values = re.findall(r"echo (\S+)\s*;;", table)
    numbers = [v for v in values if v != '""']
    assert numbers, "the typical table is empty"
    for value in numbers:
        assert value.isdigit(), f"{value!r} is not a plain seconds count"
        assert 1 <= int(value) <= 600, f"{value}s is not a plausible phase"


def test_the_typical_durations_are_never_enforced() -> None:
    """**The one that matters.** A descriptive number that acquires a branch
    becomes a timeout, and this run already has a watchdog. Two disagreeing
    deadlines is worse than one."""

    code = _runner()
    calls = [line for line in code.splitlines() if "phase_typical_s" in line]
    # One definition, one call inside phase(), and nothing else.
    assert len(calls) == 2, f"phase_typical_s is used somewhere new: {calls}"
    assert any("typical=$(phase_typical_s" in line for line in calls)

    at = code.index("phase() {")
    body = code[at:code.index("\n}", at)]
    for branch in ("-gt", "-lt", "timeout", "rerun", "exit"):
        assert branch not in body, (
            f"phase() branches on {branch!r}; a typical duration has become a deadline"
        )


def test_the_operator_is_told_before_the_waiting_starts() -> None:
    """The banner repeats it per phase, but the first 62 s wait arrives before
    any of that is useful. The header has to land first."""

    code = _runner()
    header = code.index('echo "  run    : $RUN_DIR"')
    first_phase_after = code.index('phase "precondition: the arena', header)
    preamble = code[header:first_phase_after]

    assert "bring-up is ~120 s" in preamble, "the total wait is not announced"
    assert "this is not a hang" in preamble, (
        "the quiet step is not named as expected behaviour"
    )


def test_the_deliberate_lens_reap_does_not_look_like_a_crash() -> None:
    """Job control printed `line 792: 3800360 Killed` into the launch log when
    the restart-once path fired -- in the middle of the one sequence the
    operator is being asked to trust. Safe to disown precisely because the pid
    comes from the announcement file and never from `$!`."""

    code = _runner()
    at = code.index('setsid python3 "$REPO/tools/lens/corridor_lens.py"')
    block = code[at:at + 900]

    assert "disown" in block, "the deliberate kill prints a crash-shaped line again"
    assert 'lens_pid="$LENS_PID"' in code, "the pid is being taken from $! again"


def test_the_runner_still_parses() -> None:
    """These edits are in the hot path of every run; a syntax error here costs
    an Isaac load to discover."""

    done = subprocess.run(["bash", "-n", str(RUNNER)], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
