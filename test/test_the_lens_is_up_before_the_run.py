"""The lens is up before the run, and the banner means it is SEEING.

ADRs 0035 and 0037. Three failures are pinned here, all measured on 2026-08-14.
The file keeps its name: the lens is still up before the *run* -- before SLAM,
Nav2, the mission and every `rerun()` exit that used to be invisible. What
moved is that it now starts after the simulator, for a measured reason.

**Invisible bring-up** (0035). The lens started after SLAM and Nav2, and ten of
the twelve `rerun()` exits precede that point, so a run that died in bring-up
wrote no `lens.log` at all -- runs 20260814-022725, -023029 and -025555 did
exactly that. The operator called them "faux launches".

**A banner that could lie** (0035). The old block printed
`http://127.0.0.1:8765/` from a literal, unconditionally, after a poll that
broke on success *or* on the lens dying -- while the lens walks to 8766-8770
when 8765 is taken. So it could announce a dead lens, or somebody else's stub.

**A banner that told the truth about the wrong thing** (0037). Two of six runs
were watched by a lens that answered `/healthz` for their whole length and
resolved nothing: 500 history rows, every metric column null. Serving is not
seeing. The banner is now gated on a non-zero scan rate, which is only askable
after `simctl start` -- hence the move, and hence the inverted ordering
assertion below.

These are source-level assertions on purpose: the alternative is a six-minute
Isaac run to watch a URL appear in the right order.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools/corridor_profile_run.sh"
LENS = ROOT / "tools/lens/corridor_lens.py"

#: The phase the lens must precede. Spelled out in full rather than as
#: `phase "precondition` so a future second precondition cannot satisfy it.
CONTRACT_PHASE = 'phase "precondition: $ROBOT contract'


def _runner() -> str:
    return RUNNER.read_text(encoding="utf-8")


def _index(code: str, needle: str) -> int:
    at = code.find(needle)
    assert at >= 0, f"anchor vanished from the runner: {needle!r}"
    return at


def test_the_lens_starts_after_the_simulator_and_before_the_run() -> None:
    """**The whole point, in its 0037 form.**

    After `simctl start`, because the seeing gate cannot be asked before /scan
    exists -- and because 2 of 6 lenses created before Isaac went deaf against
    0 of ~90 created after it.

    Still before the contract precondition, and therefore before SLAM, Nav2,
    the mission, and every `rerun()` exit that made bring-up invisible. That
    half of ADR 0035 is not loosened by 0037; it is the reason the block did
    not simply go back where it came from.
    """

    code = _runner()
    sim_at = _index(code, '"$SIMCTL" start')
    lens_at = _index(code, 'phase "lens"')
    contract_at = _index(code, CONTRACT_PHASE)

    assert sim_at < lens_at, (
        "the lens is created before the simulator again: 2 of 6 such lenses "
        "served for a whole run and heard nothing (ADR 0037)"
    )
    assert lens_at < contract_at, (
        "the lens starts after the preconditions again: bring-up deaths go "
        "back to being invisible (ADR 0035)"
    )


def test_the_lens_starts_after_the_things_it_needs() -> None:
    """Earlier is not unconditionally better: it cannot precede the workspace
    (rclpy), the traps (or it leaks on abort), or the occupancy gates."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')

    for anchor in ('source "$WS_SETUP"', "trap on_exit EXIT", "isaac_lock_acquire"):
        assert _index(code, anchor) < lens_at, f"the lens now precedes {anchor!r}"


def test_no_port_literal_survives_in_the_runner() -> None:
    """The banner can only print what the lens reported binding."""

    code = _runner()

    assert "8765" not in code, "a literal port is back; it can name a dead lens"
    assert "$LENS_URL" in code, "the banner no longer interpolates the announcement"


def test_a_lens_that_never_served_refuses_the_run() -> None:
    """CLAUDE.md makes the lens mandatory equipment, and an unwatchable run is
    the bug. Since 0037 the refusal costs an Isaac load -- deliberately: a
    blind run does not merely fail to help, it poisons the evidence."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    refusal = code.find("the lens never served", lens_at)

    assert refusal > 0, "a lens that never served no longer refuses"
    assert code.find(CONTRACT_PHASE) > refusal, (
        "the refusal must precede the mission, or the run is spent unwatched"
    )
    assert "--no-lens" in code[lens_at:refusal + 400], (
        "the refusal must name its own override"
    )


def test_a_lens_that_never_saw_anything_also_refuses_the_run() -> None:
    """**The 0037 refusal.** A lens serving `ok` while resolving nothing is the
    faux launch that survived the first fix, and it is the worse of the two:
    the earlier class at least failed loudly."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    refusal = code.find("the lens never SAW the session", lens_at)

    assert refusal > 0, "a deaf lens no longer refuses the run"
    assert code.find(CONTRACT_PHASE) > refusal, (
        "the deaf-lens refusal must precede the mission"
    )
    assert "--no-lens" in code[lens_at:refusal + 400], (
        "the refusal must name its own override"
    )


def test_the_deaf_lens_is_restarted_once_and_its_log_is_kept() -> None:
    """One restart, because the deafness has no identified mechanism and a
    retry is cheaper than a lost Isaac load. Attempt 1's log is the only
    artifact the failure leaves, so it is copied aside before the retry
    overwrites it."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    block = code[lens_at:code.find(CONTRACT_PHASE)]

    assert "for attempt in 1 2" in block, "the deaf lens is no longer retried"
    assert "lens-attempt1.log" in block, (
        "the retry overwrites the only evidence of the first failure"
    )
    assert 'kill -TERM "$LENS_PID"' in block, (
        "the deaf lens is reaped by $! again, which is the setsid wrapper"
    )


def test_the_banner_is_printed_only_where_the_lens_was_seen() -> None:
    """A banner outside the seen branch is the old bug with extra steps."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    block = code[lens_at:code.find(CONTRACT_PHASE)]

    banner = "echo \"  lens: $LENS_URL"
    assert block.count(banner) == 1, "the banner is printed in more than one place"
    seen_at = block.index('if [ "$LENS_SEEN" = 1 ]; then')
    assert block.index(banner) > seen_at, (
        "the banner is printed outside the branch that proved the lens sees"
    )


def test_the_announcement_is_written_only_after_the_bind() -> None:
    """Its existence is the proof the port is served, not an intention to."""

    source = LENS.read_text(encoding="utf-8")
    bind_at = source.index("async with server:")
    call_at = source.index("write_announcement(args.announce", bind_at)

    assert call_at > bind_at, "the announcement is written before the bind"


def test_the_runner_and_the_lens_agree_on_the_announcement(tmp_path) -> None:
    """**The seam test.** A shell/Python contract that no unit test on either
    side can cover: the lens writes the file, the runner parses it, and the two
    are edited in different languages by different reflexes.

    Runs the lens's own writer, then the runner's own extraction snippet.
    """

    sys.path.insert(0, str(ROOT / "tools/lens"))
    try:
        import corridor_lens
    except ImportError as exc:  # pragma: no cover - environment, not logic
        pytest.skip(f"the lens is not importable here: {exc}")

    target = tmp_path / "lens.announce.json"

    class _Args:
        map_topic = "/map"
        scan_topic = "/scan"

    assert corridor_lens.write_announcement(str(target), "127.0.0.1", 8767, _Args())

    # The extraction, lifted from the runner rather than retyped.
    code = _runner()
    start = code.index('python3 -c \'\nimport json, sys')
    end = code.index("'", code.index('print(a["url"]', start))
    snippet = code[start + len("python3 -c '"):end]

    out = subprocess.run([sys.executable, "-c", snippet, str(target)],
                         capture_output=True, text=True, check=True)
    url, port, pid = out.stdout.split()

    written = json.loads(target.read_text(encoding="utf-8"))
    assert url == written["url"] == "http://127.0.0.1:8767/"
    assert int(port) == 8767, "the runner would verify the wrong port"
    assert int(pid) == written["pid"]


def test_the_runner_and_the_lens_agree_on_what_seeing_means() -> None:
    """**The second seam test**, and the one that matters most today.

    The lens decides what `/healthz` says; the runner decides what counts as
    seeing. They are a shell/Python pair again, and getting them out of step
    means either refusing every healthy run or accepting every deaf one.

    So: run the lens's own `healthz_payload` over the three shapes it can
    produce, and judge each with the runner's own extracted snippet.
    """

    sys.path.insert(0, str(ROOT / "tools/lens"))
    try:
        import corridor_lens
    except ImportError as exc:  # pragma: no cover - environment, not logic
        pytest.skip(f"the lens is not importable here: {exc}")

    code = _runner()
    start = code.index("python3 -c '\nimport sys, json")
    end = code.index("'", code.index("rates.get(\"scan\")", start))
    snippet = code[start + len("python3 -c '"):end]

    def verdict(state) -> int:
        payload = json.dumps(corridor_lens.healthz_payload(state))
        return subprocess.run([sys.executable, "-c", snippet], input=payload,
                              capture_output=True, text=True).returncode

    seeing = {"t": 12.0, "frozen": False, "rates": {"scan": 14.3, "map": 0.4}}
    deaf = {"t": 12.0, "frozen": False, "rates": {"scan": 0.0, "map": 0.0}}
    frozen = {"t": 300.0, "frozen": True, "rates": {"scan": 0.0}}

    assert verdict(seeing) == 0, "the runner would refuse a healthy lens"
    assert verdict(deaf) == 1, "the runner would accept a deaf lens -- the bug"
    assert verdict(frozen) == 1, "a lens whose data stopped is not seeing either"
    assert verdict(None) == 1, "a lens with no sample yet has not seen anything"


def test_the_announcement_carries_the_pid_for_a_setsid_launch() -> None:
    """`$!` is the setsid wrapper, so the real pid can only come from here."""

    code = _runner()
    assert "setsid python3" in code, "the lens is no longer setsid-launched"
    assert 'lens_pid="$LENS_PID"' in code, (
        "lens_pid is being taken from $! again, which is the wrapper"
    )


def test_the_lens_is_asked_again_before_the_robot_moves() -> None:
    """**One check was not enough, and a run proved it.**

    `20260814-125254` passed the seeing gate, printed its banner, and went deaf
    seconds later: 300 samples over 60.2 s with every metric column null. It
    was created 71 s AFTER `simctl start`, which is the placement ADR 0037
    adopted precisely because no lens created there had been observed to go
    blind -- so the correlation that record leans on now has a counterexample.

    The gate is a moment; bring-up is ~130 s. Asking once more immediately
    before the mission converts that failure from a post-run covariate into a
    refusal, and costs only the bring-up already spent.
    """

    code = _runner()
    assert code.count("lens_is_seeing()") == 1, (
        "the seeing check is defined more than once; the two call sites would "
        "drift and one of them would judge a different thing"
    )
    calls = [at for at in range(len(code))
             if code.startswith("lens_is_seeing \"$LENS_PORT\"", at)]
    assert len(calls) == 2, f"expected two call sites, found {len(calls)}"

    lens_at = _index(code, 'phase "lens"')
    mission_at = _index(code, 'phase "T3.3a transit recorder')
    assert lens_at < calls[0] < mission_at, "the first check is not at the lens"
    assert calls[1] < mission_at, (
        "the second check runs after the robot has already moved, which is "
        "an autopsy rather than a gate"
    )
    refusal = code.find("the lens went deaf during bring-up", calls[1])
    assert 0 < refusal < mission_at, "a lens that went deaf no longer refuses"
    assert "--no-lens" in code[calls[1]:refusal + 400]


def test_the_second_check_is_short_because_it_is_not_a_startup_race() -> None:
    """A 20 s poll before every mission would add 20 s to every healthy run.
    The instrument has already proved it can hear by this point."""

    code = _runner()
    mission_at = _index(code, 'phase "T3.3a transit recorder')
    before = code[:mission_at]
    assert 'lens_is_seeing "$LENS_PORT" 6' in before, (
        "the pre-mission check no longer uses the short deadline"
    )
