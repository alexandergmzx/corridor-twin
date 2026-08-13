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


def _residents_function() -> str:
    """The preflight detector, lifted out of the runner so it can be run."""

    source = RUNNER.read_text(encoding="utf-8")
    start = source.index("residents() {")
    end = source.index("\n}\n", start) + len("\n}\n")
    return source[start:end]


def test_the_preflight_sees_an_orphan_and_ignores_a_namespaced_stack() -> None:
    """Run, with decoys. A detector nobody has watched fire is not a detector.

    On 2026-08-12 domain 67 carried the previous run's own un-namespaced
    behavior_server -- alive 84 minutes after its session ended, offering the
    same recovery actions as the next run's, holding a dead session's costmap.
    `occupants` could not see it: that looks for the SIMULATOR, and these
    outlive it.

    The /robot2 decoy is the control. A namespaced stack is a different graph
    and somebody else's business; refusing on it would block the corridor on a
    fleet session that cannot collide with it.
    """

    orphan = (
        "/opt/ros/jazzy/lib/nav2_behaviors/behavior_server "
        "--ros-args -r __node:=behavior_server"
    )
    namespaced = f"{orphan} -r __ns:=/robot2"
    script = (
        f"{_residents_function()}\n"
        f'bash -c \'exec -a "{orphan}" sleep 8\' &\n'
        "first=$!\n"
        f'bash -c \'exec -a "{namespaced}" sleep 8\' &\n'
        "second=$!\n"
        "sleep 1\n"
        "residents\n"
        "kill $first $second 2>/dev/null\n"
        "wait 2>/dev/null\n"
    )
    found = subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=60,
    ).stdout.strip().splitlines()

    assert len(found) == 1, f"expected exactly the un-namespaced orphan, got: {found}"
    assert "__ns:=" not in found[0]
    assert "behavior_server" in found[0]


def test_the_preflight_is_quiet_when_nothing_is_running() -> None:
    """The other half of the control: it must not refuse every run.

    Asserted against this machine as the tests run -- if a corridor stack is up
    while the suite runs, this fails, and that is the correct answer.
    """

    found = subprocess.run(
        ["bash", "-c", f"{_residents_function()}\nresidents\n"],
        capture_output=True, text=True, timeout=30,
    ).stdout.strip()
    assert found == "", f"un-namespaced ROS nodes are alive on this machine: {found}"


def test_teardown_polls_for_death_instead_of_sleeping_once() -> None:
    """'It was not dead 3 seconds ago' is not a verification."""

    source = RUNNER.read_text(encoding="utf-8")
    assert '[ -z "$(occupants)" ] && [ -z "$(residents)" ] && break' in source
    assert "survived teardown:" in source
    assert 'teardown_verified=${teardown_verified:-0}' in source


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


def test_the_precondition_does_not_drive_the_missions_robot() -> None:
    """The startup circle, pinned so it cannot come back.

    `check_isaac_contract.py` drives vx 0.12 / wz 0.3 for the first half of its
    window -- a 0.4 m-radius arc -- published straight to /cmd_vel, bypassing
    the governor, before SLAM or Nav2 exist. Measured in the bag of run
    20260812-164717: 802 moving commands at a constant (0.120, 0.300) between
    t=16.31 s and t=31.28 s, while /cmd_vel_raw's first message of any kind is
    at t=86.05 s.

    Three sessions read that as a Nav2 recovery, a stale behavior_server and a
    DWB critic. It was the health check.
    """

    source = RUNNER.read_text(encoding="utf-8")
    assert "--speed 0.0 --turn 0.0" in source or '--speed 0.0" "--turn 0.0' in source, (
        "the robot1 contract check may not drive the mission's robot"
    )
    # And the reason travels with it.
    assert "startup circle" in source


def test_the_probe_watches_the_ungoverned_topic_too() -> None:
    """An instrument that only sees the well-behaved path cannot find a bad one.

    Watching /cmd_vel_raw alone, the probe reported "zero rotation before the
    goal" on three runs while the robot was turning 253 deg on /cmd_vel.
    """

    probe = (ROOT / "tools/corridor_startup_probe.py").read_text(encoding="utf-8")
    assert 'f"{namespace}/cmd_vel"' in probe
    assert 'f"{namespace}/cmd_vel_raw"' in probe
    assert "moving_on_cmd_vel_directly" in probe
    assert "ungoverned_rotation_deg" in probe


def test_the_recorder_outlives_the_nav_gate() -> None:
    """The instrument must not stop before the thing it measures.

    Sizing the transit window capped the RECORDER while leaving the nav window
    at whatever the cap allowed, and run 20260812-182237 then ran a 200 s
    recorder inside a 429 s nav window -- so a slow but successful delivery
    would have been truncated by the instrument watching it. The nav window is
    what gets capped; the recorder is sized from it.
    """

    source = RUNNER.read_text(encoding="utf-8")
    assert 'NAV_TIMEOUT="$TRANSIT_WINDOW_S"' in source
    assert ': "${GATE_SECONDS:=$((NAV_TIMEOUT + 10))}"' in source
    # And the recorder is never capped independently of it.
    assert 'GATE_SECONDS="$TRANSIT_WINDOW_S"' not in source


def test_goal_not_accepted_is_decided_by_whether_the_robot_moved() -> None:
    """"Goal not accepted" means two different things, twenty minutes apart.

    20260812-183327: bt_navigator was inactive, the goal was refused, the robot
    moved 0.13 m, and the run recorded three true numbers about a robot that
    was never asked to do anything. Infrastructure.

    20260812-184220: the goal was ACCEPTED -- "Begin navigating from current
    location (0.00, 0.00) to (4.11, -2.93)" is in the launch log -- and A drove
    7.865 m to within 0.178 m. What went missing was the acceptance RESPONSE,
    which corridor_nav_gate.py already documents as a nav failure that never
    happened. A result.

    So the runner asks the recorder what the robot did, rather than trusting
    what the gate reported, and the threshold is the transit gate's own.
    """

    source = RUNNER.read_text(encoding="utf-8")
    assert '"failure": "goal not accepted"' in source
    assert "ground_truth_distance_m" in source
    assert "acceptance response lost" in source
    assert "and the robot never moved" in source
    # It must run AFTER the recorder's verdict, or the evidence it needs does
    # not exist yet.
    assert source.index("=== transit recorder verdict ===") < source.index(
        "acceptance response lost"
    )


def test_the_lifecycle_poll_does_not_match_inactive() -> None:
    """`*active*` matches "inactive", and that cost four runs on 2026-08-12.

    `ros2 lifecycle get` prints "active [3]", "inactive [2]", "unconfigured [1]".
    Both lifecycle polls in this runner used `*active*`, so a node that was
    still configuring was declared ready, the goal went out, and bt_navigator
    answered "Action server is inactive. Rejecting the goal." It read as a flaky
    bringup race for a whole session. The stack was telling the truth; the
    runner was mis-reading it.
    """

    source = RUNNER.read_text(encoding="utf-8")
    code = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )
    assert "*active*" not in code, "the lifecycle poll matches 'inactive' again"
    assert code.count("active*)") >= 2, "both polls must test the state's PREFIX"


def test_the_lifecycle_glob_is_checked_against_real_states() -> None:
    """The four states this poll can actually see, run through the glob."""

    script = (
        'for s in "active [3]" "inactive [2]" "unconfigured [1]" "activating [6]"; do\n'
        '  case "$s" in active*) echo "$s|yes";; *) echo "$s|no";; esac\n'
        "done\n"
    )
    out = subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=30
    ).stdout.strip().splitlines()
    verdict = dict(line.split("|") for line in out)

    assert verdict["active [3]"] == "yes"
    assert verdict["inactive [2]"] == "no", "this is the bug that cost four runs"
    assert verdict["unconfigured [1]"] == "no"
    assert verdict["activating [6]"] == "no"
