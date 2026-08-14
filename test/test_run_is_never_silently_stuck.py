"""A run that hangs must fail loudly, say where, and quote the log.

**The failure this guards against, measured.** On 2026-08-13 run
`20260813-002222` sat in `waiting for bt_navigator to reach ACTIVE` until a
human killed it, and its `run.json` ended up with `arena`, `git` and the
hashes -- and no `classification`, no cause, no phase, no end record at all.
Three of the first twenty-four corridor runs ended exactly that way. Twelve and
a half percent of runs produced no verdict.

Two mechanisms did it, and both are now closed:

* Both bring-up loops counted ITERATIONS, on the assumption an iteration costs
  ~5 s. Each `ros2 lifecycle get` blocked ~13 s and returned nothing -- the CLI
  needs the ros2 daemon and `simctl stop` stops it at the end of every run --
  so `14 x (13 + 5)` quietly became 255 s per attempt.
* `trap 'teardown || true' INT TERM` tore down and RETURNED, so Ctrl-C could
  not stop a run and the watchdog's own verdict was overwritten by whichever
  `rerun` the resumed loop reached next.

These read the runner's source. `test/test_corridor_profile_run.py` owns the
rest of its contract; this file owns "never silently stuck".
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools" / "corridor_profile_run.sh"


def _code() -> str:
    """The script with comment-only lines removed.

    The comments explain these very bugs by name, so a naive substring search
    would find the explanation and call it the defect.
    """

    return "\n".join(
        line for line in RUNNER.read_text(encoding="utf-8").splitlines()
        if not line.lstrip().startswith("#")
    )


def test_every_lifecycle_wait_is_bounded_in_wall_clock() -> None:
    """Not in iterations. An iteration is only ~5 s until it is 13."""

    code = _code()
    assert "LIFECYCLE_DEADLINE_S=" in code
    # Both loops compare `date +%s` against a deadline.
    assert code.count("date +%s) + LIFECYCLE_DEADLINE_S") == 2, (
        "both the slam and the nav lifecycle loops need a wall-clock deadline"
    )
    assert 'for _ in $(seq 1 14)' not in code, "the nav iteration count is back"
    assert 'for _ in $(seq 1 12)' not in code, "the slam iteration count is back"


def test_every_ros_cli_call_in_a_wait_loop_has_a_timeout() -> None:
    """The call that blocked 13 s and returned nothing, on every iteration."""

    code = _code()
    for call in re.findall(r"^.*ros2 lifecycle get.*$", code, re.M):
        assert "timeout " in call, f"unbounded ros2 CLI call in a poll: {call.strip()}"


def test_the_abort_line_is_checked_on_every_iteration() -> None:
    """115 s were burned after the verdict was already in the log.

    `Aborting bringup` used to be checked only from inside the `active*)`
    branch, so an attempt that aborted without ever reading active never saw
    it. The manager wrote that line three seconds after launch.
    """

    code = _code()
    body = code[code.index("nav_deadline="):code.index("NAV_ATTEMPTS=")]
    before_case = body[:body.index("case ")]
    assert "Aborting bringup" in before_case, (
        "the abort check must run before the state poll, not inside its branch"
    )


def test_bringup_readiness_does_not_depend_on_the_daemon_alone() -> None:
    """The bond line is written by bt_navigator itself, into its own log."""

    code = _code()
    assert "Creating bond (bt_navigator)" in code


def test_int_and_term_exit_instead_of_resuming() -> None:
    """Ctrl-C must stop a run, and the watchdog's verdict must be the one kept."""

    code = _code()
    assert "trap 'on_signal INT' INT" in code
    assert "trap 'on_signal TERM' TERM" in code
    assert "trap 'teardown || true' INT TERM" not in code, (
        "the returning handler is back; Ctrl-C will not stop a run"
    )
    handler = code[code.index("on_signal() {"):]
    handler = handler[:handler.index("trap on_exit EXIT")]
    assert "exit 3" in handler, "the signal handler must exit, not return"
    assert "classify rerun" in handler
    assert "write_diagnosis" in handler


def test_the_run_records_the_phase_it_is_in_as_it_goes() -> None:
    """So a death that skips the EXIT trap can still say where it was."""

    code = _code()
    assert 'PHASE="$1"' in code
    assert '> "$RUN_DIR/.phase"' in code
    # And every banner is a phase call, so nothing advances silently.
    assert not re.search(r'^echo "=== .* ==="$', code, re.M), (
        "a bare banner does not update the phase or carry a timestamp"
    )


def test_the_phase_banner_carries_a_clock() -> None:
    """The wall clock of the 2026-08-13 runs had to be reconstructed from file
    mtimes, because runner.log carries no time anywhere."""

    code = _code()
    banner = code[code.index("phase() {"):code.index("diagnosis_log()")]
    assert "date +%H:%M:%S" in banner
    assert "elapsed" in banner


def test_the_diagnose_subcommand_writes_a_log_tail(tmp_path: Path) -> None:
    """End to end, the way bash calls it: the artifact must carry the evidence.

    Uses the real shape of the failure -- a manager abort three seconds into a
    launch log -- because a diagnosis that does not quote the line that
    explains the death is a diagnosis nobody can act on.
    """

    manifest = tmp_path / "run.json"
    manifest.write_text(json.dumps({"run_id": "test"}) + "\n", encoding="utf-8")
    log = tmp_path / "nav-launch-attempt1.log"
    log.write_text(
        "[lifecycle_manager-5] [INFO] Configuring controller_server\n"
        "[lifecycle_manager-5] [ERROR] Failed to change state for node: "
        "controller_server.\n"
        "[lifecycle_manager-5] [ERROR] Failed to bring up all requested nodes. "
        "Aborting bringup.\n",
        encoding="utf-8",
    )

    finished = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "run_manifest.py"), "diagnose",
         "--path", str(manifest), "--why", "watchdog at the 600s cap",
         "--phase", "nav stack", "--elapsed-s", "601.4", "--log", str(log)],
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert finished.returncode == 0, finished.stderr

    payload = json.loads(manifest.read_text(encoding="utf-8"))
    entry = payload["diagnosis"][0]
    assert entry["phase"] == "nav stack"
    assert entry["elapsed_s"] == 601.4
    assert "Aborting bringup" in "\n".join(entry["log_tail"])
    # And it must not have clobbered what was already there.
    assert payload["run_id"] == "test"


def test_a_missing_log_is_recorded_rather_than_raising(tmp_path: Path) -> None:
    """Recording a problem must never become the problem."""

    manifest = tmp_path / "run.json"
    manifest.write_text("{}\n", encoding="utf-8")

    finished = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "run_manifest.py"), "diagnose",
         "--path", str(manifest), "--why", "test", "--phase", "p",
         "--log", str(tmp_path / "nope.log")],
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert finished.returncode == 0, finished.stderr
    entry = json.loads(manifest.read_text(encoding="utf-8"))["diagnosis"][0]
    assert "absent" in entry["log"]
    assert "log_tail" not in entry


# ------------------------------------------- the phase is the one it died in
#
# Run 20260814-025555 died in SLAM bring-up. Its run.json records
# `diagnosis[0].phase` as "precondition: robot1 contract (--domain 67 ...)",
# because that stage had no phase() of its own and inherited the previous one.
# The watchdog and the completion check both stamp ${PHASE} into the verdict,
# so a misattributed phase is a misattributed post-mortem.


def _phase_labels_with_positions(code: str) -> list[tuple[int, str]]:
    return [(m.start(), m.group(1))
            for m in re.finditer(r'^\s*phase "([^"]+)"', code, re.M)]


def _phase_covering(code: str, needle: str) -> str:
    """The phase in force where `needle` executes."""

    at = code.index(needle)
    before = [(pos, label) for pos, label in _phase_labels_with_positions(code)
              if pos < at]
    assert before, f"nothing sets a phase before {needle!r}"
    return before[-1][1]


def test_the_slam_bringup_is_labelled_slam() -> None:
    """The one with measured evidence: 025555's SLAM death, filed under the
    contract precondition."""

    code = _code()
    label = _phase_covering(code, "ros2 launch yahboomcar_config slam_launch.py")

    assert "slam" in label.lower(), (
        f"a SLAM death would be recorded as {label!r}, which is where 025555's "
        f"post-mortem sent the last reader"
    )


def test_the_map_save_is_labelled_for_its_own_timeout() -> None:
    """It carries a `timeout 60` and is watchdog-exposed."""

    code = _code()
    label = _phase_covering(code, "map_saver_cli")

    assert "map" in label.lower(), f"a map-save kill would be recorded as {label!r}"


def test_the_lock_wait_is_not_filed_under_the_arena_check() -> None:
    """The Isaac lock may legitimately poll for 45 minutes. An operator reading
    `.phase` during it must not be told the arena is being checked."""

    code = _code()
    label = _phase_covering(code, "isaac_lock_acquire")

    assert "arena" not in label.lower(), (
        f"a 45-minute lock wait reports {label!r}"
    )


def test_no_phase_is_set_inside_teardown() -> None:
    """`on_signal` tears down FIRST and then reads ${PHASE}. A phase set inside
    teardown would overwrite the very phase the signal handler exists to report.
    """

    code = _code()
    start = code.index("teardown() {")
    body = code[start:code.index("\n}\n", start)]

    assert not re.search(r'^\s*phase "', body, re.M), (
        "teardown sets a phase; it would erase the phase the run died in"
    )
