"""The runner's evidence discipline, pinned as text because it is a shell script.

`corridor_profile_run.sh` has no unit harness -- it starts a simulator. What CAN
be checked without one is the property that failed: every artifact it writes
must belong to an identifiable run, and every way it can end must say what
happened. Both are structural, so both are checkable here.

The CLI half is exercised for real: bash calls `run_manifest.py` as a
subprocess, so the subprocess contract is tested as a subprocess.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools/corridor_profile_run.sh"


def test_no_artifact_is_named_by_robot_and_profile_alone() -> None:
    """That naming is what mixed four sessions into one directory.

    `gate-robot1-nominal_m6_n3.json` said which robot and which profile, and
    nothing about WHICH RUN -- so each run overwrote the last, and a run that
    died before writing left its predecessor's file standing in for it.
    """

    source = RUNNER.read_text(encoding="utf-8")
    code = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )

    found = set(re.findall(r'\$EVIDENCE/[a-z-]+-\$ROBOT-\$PROFILE', code))
    # `latest-` is the one legitimate use: a symlink INTO the newest run
    # directory, not a file anything writes.
    offenders = found - {"$EVIDENCE/latest-$ROBOT-$PROFILE"}
    assert not offenders, f"flat, session-mixing artifact paths remain: {sorted(offenders)}"

    # And the replacement is actually in use.
    assert 'RUN_DIR="$EVIDENCE/$RUN_ID"' in code
    assert '$RUN_DIR/gate.json' in code
    assert '$RUN_DIR/nav.json' in code


def test_the_run_id_carries_a_timestamp_the_robot_and_the_profile() -> None:
    code = RUNNER.read_text(encoding="utf-8")
    assert 'RUN_ID="$(date +%Y%m%d-%H%M%S)-$ROBOT-$PROFILE"' in code
    # A stable path to the newest run, so `latest` is a symlink rather than a
    # filename everyone overwrites.
    assert 'ln -sfn "$RUN_ID" "$EVIDENCE/latest-$ROBOT-$PROFILE"' in code


def test_every_infrastructure_exit_classifies_itself() -> None:
    """Exit 3 was the whole record of a rerun, and only if somebody saw it.

    A bare `exit 3` outside the `rerun` helper and the watchdog block would put
    an unclassified rerun back into the evidence directory.
    """

    lines = RUNNER.read_text(encoding="utf-8").splitlines()
    bare = [
        (number, line.strip())
        for number, line in enumerate(lines, start=1)
        if re.search(r'(^|;|\s)exit 3\b', line) and not line.lstrip().startswith("#")
    ]
    # Two only: the one inside rerun(), and the watchdog's, which classifies on
    # the line above it.
    assert len(bare) == 2, f"unclassified infrastructure exits: {bare}"

    source = RUNNER.read_text(encoding="utf-8")
    assert 'classify rerun "watchdog killed the session' in source
    assert 'rerun() {' in source


def test_a_run_that_says_nothing_about_itself_is_a_crash() -> None:
    """The default verdict, and the reason the joint-velocities death was invisible."""

    source = RUNNER.read_text(encoding="utf-8")
    assert 'classify crash "run ended without a verdict' in source
    assert "trap on_exit EXIT" in source
    # The recording half must survive the handover that clears the teardown trap.
    assert "trap 'record_exit $?' EXIT" in source


def test_the_run_records_which_scenario_it_actually_ran() -> None:
    """A path is not an identity: the arena and the manifest came apart once."""

    source = RUNNER.read_text(encoding="utf-8")
    assert 'arena_sha256=$(digest "$ARENA")' in source
    assert 'manifest_sha256=$(digest "$MANIFEST")' in source


def test_the_manifest_cli_is_callable_the_way_bash_calls_it(tmp_path: Path) -> None:
    """Subprocess contract, exercised as a subprocess."""

    manifest = tmp_path / "run.json"
    tool = str(ROOT / "tools/run_manifest.py")

    subprocess.run(
        [sys.executable, tool, "set", "--path", str(manifest),
         "--set", "robot=robot1", "--set", "domain=67", "--set", 'flags={"dock":false}'],
        check=True, capture_output=True,
    )
    subprocess.run(
        [sys.executable, tool, "classify", "--path", str(manifest),
         "--classification", "rerun", "--cause", "contract precondition failed"],
        check=True, capture_output=True,
    )
    subprocess.run(
        [sys.executable, tool, "error", "--path", str(manifest), "--message", "map save failed"],
        check=True, capture_output=True,
    )

    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["robot"] == "robot1"
    # JSON literals keep their type through the shell; bare words stay strings.
    assert payload["domain"] == 67
    assert payload["flags"] == {"dock": False}
    assert payload["classification"] == "rerun"
    assert payload["errors"][0]["message"] == "map save failed"


def test_the_digest_subcommand_prints_something_bash_can_capture(tmp_path: Path) -> None:
    target = tmp_path / "arena.usd"
    target.write_bytes(b"corridor")
    done = subprocess.run(
        [sys.executable, str(ROOT / "tools/run_manifest.py"), "digest", "--file", str(target)],
        check=True, capture_output=True, text=True,
    )
    assert len(done.stdout.strip()) == 64

    # A missing file prints an empty line rather than failing: the runner
    # captures this in a `$(...)` under `set -e`, and a non-zero exit there
    # would take down the run over an optional artifact.
    absent = subprocess.run(
        [sys.executable, str(ROOT / "tools/run_manifest.py"),
         "digest", "--file", str(tmp_path / "no")],
        check=True, capture_output=True, text=True,
    )
    assert absent.stdout.strip() == ""
