"""The lens comes up before the simulator, and the banner is a verified port.

ADR 0035. Two failures are pinned here, both measured on 2026-08-14.

**Invisible bring-up.** The lens started after SLAM and Nav2, and ten of the
twelve `rerun()` exits precede that point, so a run that died in bring-up wrote
no `lens.log` at all -- runs 20260814-022725, -023029 and -025555 did exactly
that. The operator called them "faux launches".

**A banner that could lie.** The old block printed `http://127.0.0.1:8765/`
from a literal, unconditionally, after a poll that broke on success *or* on the
lens dying -- while the lens walks to 8766-8770 when 8765 is taken. So it could
announce a dead lens, or somebody else's stub.

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


def _runner() -> str:
    return RUNNER.read_text(encoding="utf-8")


def _index(code: str, needle: str) -> int:
    at = code.find(needle)
    assert at >= 0, f"anchor vanished from the runner: {needle!r}"
    return at


def test_the_lens_starts_before_the_simulator() -> None:
    """**The whole point.** Ten of twelve rerun() exits are after this line."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    sim_at = _index(code, '"$SIMCTL" start')

    assert lens_at < sim_at, (
        "the lens starts after simctl again: every bring-up death is invisible"
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
    """CLAUDE.md makes the lens mandatory equipment. Nothing has been spent at
    this point, so refusing is free -- and an unwatchable run is the bug."""

    code = _runner()
    lens_at = _index(code, 'phase "lens"')
    refusal = code.find("the lens never served", lens_at)

    assert refusal > 0, "a lens that never served no longer refuses"
    assert code.find('"$SIMCTL" start') > refusal, (
        "the refusal must precede simctl, or it throws away an Isaac load"
    )
    assert "--no-lens" in code[lens_at:refusal + 400], (
        "the refusal must name its own override"
    )


def test_the_lens_log_is_quotable_evidence_for_that_refusal() -> None:
    code = _runner()
    start = _index(code, "diagnosis_log() {")
    assert 'lens.log' in code[start:start + 600], (
        "a lens refusal cannot quote its own log"
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


def test_the_announcement_carries_the_pid_for_a_setsid_launch() -> None:
    """`$!` is the setsid wrapper, so the real pid can only come from here."""

    code = _runner()
    assert "setsid python3" in code, "the lens is no longer setsid-launched"
    assert 'lens_pid="$LENS_PID"' in code, (
        "lens_pid is being taken from $! again, which is the wrapper"
    )
