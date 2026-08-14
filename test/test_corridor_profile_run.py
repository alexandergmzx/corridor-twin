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

import pytest

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
    # Three: `rerun()`, the watchdog's late flag check, and the INT/TERM
    # handler -- each of which classifies before it exits. The handler joined
    # them on 2026-08-13, when INT/TERM stopped returning into the middle of a
    # bring-up loop and started ending the run the way every other exit does.
    assert len(bare) == 3, f"unclassified infrastructure exits: {bare}"

    source = RUNNER.read_text(encoding="utf-8")
    assert 'classify rerun "watchdog killed the session' in source
    assert 'rerun() {' in source

    # Each of the three classifies within a few lines above its exit, rather
    # than relying on something downstream noticing.
    for number, _text in bare:
        window = "\n".join(lines[max(0, number - 12):number])
        assert "classify " in window or "rerun()" in window or "classify(" in window, (
            f"exit 3 at line {number} does not classify the run first"
        )


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


def _a_corridor_run_is_in_flight() -> bool:
    """Is `corridor_profile_run.sh` executing on this machine right now?

    The two tests below read live host state, which is the whole point of them:
    they caught two real orphaned nav2 nodes on 2026-08-13, still alive on
    domain 67 an hour after a hand-killed run. That value is kept.

    What is NOT kept is failing the suite merely because a run is legitimately
    in progress. The nav stack is *supposed* to be up then, and the runner's
    own preflight has already checked the host; blocking the suite for the
    duration of every run is what stopped code being tested while a run
    executed. So: a live runner means SKIP, and nav2 nodes with no runner still
    means FAIL, because that is an orphan.
    """

    found = subprocess.run(
        ["pgrep", "-f", "corridor_profile_run.sh"],
        capture_output=True, text=True, timeout=30, check=False,
    ).stdout.strip()
    # pgrep -f also matches the shell that invoked pytest if the command line
    # mentions the runner, so require a line that is the script itself.
    return any(
        "corridor_profile_run.sh --profile" in line or line.endswith("corridor_profile_run.sh")
        for line in subprocess.run(
            ["ps", "-eo", "args="], capture_output=True, text=True,
            timeout=30, check=False,
        ).stdout.splitlines()
    ) and bool(found)


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

    if _a_corridor_run_is_in_flight():
        pytest.skip("a corridor run is in flight; its nav2 nodes are not orphans")

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

    Asserted against this machine as the tests run. An un-namespaced nav2 node
    with no run behind it is an ORPHAN -- exactly the two this found on
    2026-08-13, alive an hour after a hand-killed run -- and that is a failure.
    A run legitimately in flight is not, and skips.
    """

    if _a_corridor_run_is_in_flight():
        pytest.skip("a corridor run is in flight; its nav2 nodes are not orphans")


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
    # The banner is a phase() call since 2026-08-13, so every one carries a
    # timestamp and updates the phase the run records for itself.
    assert source.index('phase "transit recorder verdict"') < source.index(
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


# ------------------------------------------------ the lens is not a "resident"
#
# ADR 0035. `residents()` exists to catch a node that "offers the same recovery
# actions as this run's and can command the robot". The lens has zero
# publishers, services and action clients, so it cannot. Listing it there meant
# a lens left serving after a run REFUSED the next one -- which is why the lens
# was killed at teardown, which is why every post-run look found a dead port.


def test_a_lingering_lens_does_not_refuse_the_next_run() -> None:
    """**The negative control this change is really about.**

    Three decoys at once: the un-namespaced orphan is still caught, the
    namespaced one is still ignored, and the lens is now ignored too.
    """

    if _a_corridor_run_is_in_flight():
        pytest.skip("a corridor run is in flight; its nav2 nodes are not orphans")

    orphan = (
        "/opt/ros/jazzy/lib/nav2_behaviors/behavior_server "
        "--ros-args -r __node:=behavior_server"
    )
    namespaced = f"{orphan} -r __ns:=/robot2"
    lens = "python3 /home/x/tools/lens/corridor_lens.py --domain 67"
    # Short-lived decoys, and the script WAITS for its own to be gone before
    # exiting. A neighbouring test's `sleep 8` decoys outliving their run made
    # this suite flaky the first time three of them existed at once.
    script = (
        f"{_residents_function()}\n"
        "for _ in $(seq 1 40); do [ -z \"$(residents)\" ] && break; sleep 0.25; done\n"
        f'bash -c \'exec -a "{orphan}" sleep 3\' &\n'
        "first=$!\n"
        f'bash -c \'exec -a "{namespaced}" sleep 3\' &\n'
        "second=$!\n"
        f'bash -c \'exec -a "{lens}" sleep 3\' &\n'
        "third=$!\n"
        "sleep 1\n"
        "residents\n"
        "kill $first $second $third 2>/dev/null\n"
        "wait 2>/dev/null\n"
        "for _ in $(seq 1 40); do [ -z \"$(residents)\" ] && break; sleep 0.25; done\n"
    )
    found = subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=60,
    ).stdout.strip().splitlines()

    assert len(found) == 1, f"expected only the un-namespaced orphan, got: {found}"
    assert "behavior_server" in found[0]
    assert not any("corridor_lens" in line for line in found), (
        "a lingering lens refuses the next run again; ADR 0035 says it must not"
    )


def test_teardown_does_not_kill_the_lens() -> None:
    """It must still be serving when the operator looks, which is after the run."""

    source = RUNNER.read_text(encoding="utf-8")
    start = source.index("teardown() {")
    body = source[start:source.index("\n}\n", start)]

    assert 'kill -TERM "$lens_pid"' not in body, (
        "teardown kills the lens again; every post-run look finds a dead port"
    )
    # And the escalation cannot reach it either, because residents() no longer
    # matches it -- pinned separately above.
    assert "ADR 0035" in body, "the reason the lens survives teardown is unstated"


def test_the_next_run_replaces_the_previous_lens() -> None:
    """One lens per run, without refusing to start next to the old one."""

    source = RUNNER.read_text(encoding="utf-8")

    assert "reap_previous_lens() {" in source
    assert source.index("reap_previous_lens\n") > source.index("reap_previous_lens() {")
    # It must not be able to match its own command line -- that mistake cost
    # four false positives in one session and killed a shell.
    start = source.index("reap_previous_lens() {")
    body = source[start:source.index("\n}\n", start)]
    assert 'pgrep -f "$pattern"' in body, (
        "the reaper interpolates its pattern literally and can match itself"
    )


# ------------------------------- a failed attempt is reaped as a process group
#
# THE ROOT CAUSE of the SLAM double-failure, not bad luck. Run 20260814-025555:
# attempt 1 logged "failed to send response to /slam_toolbox/change_state
# (timeout)"; the runner TERMed only the `ros2 launch` pid, which teardown's own
# comment already records as not propagating; so attempt 1's
# async_slam_toolbox_node survived and attempt 2's lifecycle manager spent its
# full 110 s talking to it. Its log reads "Configuring slam_toolbox" and then
# nothing at all.


def _lift(name: str) -> str:
    """A shell function, lifted out of the runner so it can be run for real."""

    source = RUNNER.read_text(encoding="utf-8")
    start = source.index(f"{name}() {{")
    return source[start:source.index("\n}\n", start) + len("\n}\n")]


def _reaper_preamble() -> str:
    return _lift("launch_pgid") + "\n" + _lift("reap_launch_group") + "\n"


def test_a_launch_group_is_reaped_whole() -> None:
    """The children are the point: TERMing the launcher never reached them."""

    script = (
        f"{_reaper_preamble()}"
        "setsid bash -c 'sleep 30 & sleep 30 & sleep 30 & wait' &\n"
        "leader=$!\n"
        "sleep 1\n"
        "pgid=$(ps -o pgid= -p $leader | tr -d ' ')\n"
        'echo "BEFORE $(pgrep -g $pgid | wc -l)"\n'
        'reap_launch_group "$leader" "test" >/dev/null 2>&1\n'
        'echo "AFTER $(pgrep -g $pgid | wc -l)"\n'
    )
    out = subprocess.run(["bash", "-c", script], capture_output=True,
                         text=True, timeout=60).stdout
    before = int(re.search(r"BEFORE (\d+)", out).group(1))
    after = int(re.search(r"AFTER (\d+)", out).group(1))

    assert before >= 2, f"the fixture did not build a real group: {out!r}"
    assert after == 0, f"{after} of {before} survived the group reap"


def test_the_reaper_refuses_our_own_process_group() -> None:
    """**The guard that stops this taking down the run and the shell.**

    setsid is what makes the groups different. If it silently did not, an
    unguarded group kill would signal the runner itself. A process started
    WITHOUT setsid shares our group, and must be handled by the single-pid
    fallback instead -- announced, never silent.
    """

    script = (
        f"{_reaper_preamble()}"
        "sleep 30 &\n"
        "same_group=$!\n"
        "sleep 0.5\n"
        'pgid=$(launch_pgid "$same_group")\n'
        'echo "PGID[$pgid]"\n'
        'reap_launch_group "$same_group" "test" 2>&1\n'
        'echo "SHELL_ALIVE"\n'
    )
    out = subprocess.run(["bash", "-c", script], capture_output=True,
                         text=True, timeout=60)
    combined = out.stdout + out.stderr

    assert "PGID[]" in combined, "launch_pgid returned our own group; a kill would be suicide"
    assert "no private process group" in combined, "the fallback is silent"
    assert "SHELL_ALIVE" in out.stdout, "the reaper killed the shell that called it"


def test_both_bringup_launches_get_their_own_group() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert "setsid ros2 launch yahboomcar_config slam_launch.py" in source
    assert "setsid ros2 launch $NAV_LAUNCH" in source


def test_both_retry_loops_reap_before_the_next_attempt() -> None:
    """Detection without reaping just makes the orphan fresher."""

    source = RUNNER.read_text(encoding="utf-8")

    assert 'reap_launch_group "$slam_pid"' in source
    assert 'reap_launch_group "$nav_pid"' in source
    assert 'kill -TERM "$slam_pid"' not in source, "the single-pid TERM is back"
    assert 'kill -TERM "$nav_pid"' not in source, "the single-pid TERM is back"


def test_the_nav_retry_never_reaps_by_name() -> None:
    """**The dangerous naive fix, pinned as absent.**

    `lifecycle_manager_slam` runs from /opt/ros/jazzy/lib/nav2_lifecycle_manager/
    and therefore matches residents()' `nav2_[a-z_]+`. A residents-based reap
    between nav attempts would kill the SLAM stack, and with it the map->odom
    the whole run is built on.
    """

    source = RUNNER.read_text(encoding="utf-8")
    start = source.index("nav_attempt=0")
    # The loop body only: from `nav_attempt=0` to the `done` that closes the
    # while, so the post-loop diagnosis block is not mistaken for loop code.
    end = source.index("\ndone\n", start)
    # USE, not mention: the loop carries a comment explaining why it must not
    # do this, and that comment should survive.
    body = "\n".join(line for line in source[start:end].splitlines()
                     if not line.lstrip().startswith("#"))

    assert "residents" not in body, (
        "the nav retry loop reaps by name; it would take the SLAM stack with it"
    )


def test_teardown_signals_the_group_so_setsid_does_not_weaken_it() -> None:
    """With setsid, a polite TERM to the pid alone no longer reaches children."""

    body = _lift("teardown")

    assert 'signal_launch_group "$nav_pid"' in body
    assert 'signal_launch_group "$slam_pid"' in body


# ------------------------------- the map score is a covariate, never a gate
#
# Gating a run on map divergence is the obvious idea and it is wrong here.
# Measured 2026-08-14: all five runs exceeded MAX_DUPLICATE_WALL_M = 0.20
# (0.76, 0.92, 0.84, 1.58, 1.42 m) and the WORST map -- 031348 at 1.580 m --
# produced the BEST approach, the run that touched B. A gate would have killed
# the three best deliveries of the night.


def _map_extractor() -> str:
    """The runner's own extraction snippet, lifted so it can be run."""

    source = RUNNER.read_text(encoding="utf-8")
    start = source.index('map_fields=$("$REPO/.venv/bin/python" - ')
    # The heredoc marker is followed by `|| true`, so the body starts at the
    # next NEWLINE, not at a fixed offset past the marker.
    marker = source.index("<<'PYEOF'", start)
    body = source[source.index("\n", marker) + 1:]
    return body[:body.index("PYEOF")]


def test_the_extractor_reads_a_real_score_artifact(tmp_path) -> None:
    """Run 031922's actual numbers, through the runner's actual snippet."""

    artifact = tmp_path / "map-score.json"
    artifact.write_text(json.dumps({"score": {"passed": False, "rows": [
        {"metric": "duplicate wall extent", "measured": "1.420 m"},
        {"metric": "median wall thickness", "measured": "0.020 m"},
        {"metric": "bounding box (context only)", "measured": "7.06 x 7.44 m"},
    ]}}), encoding="utf-8")

    out = subprocess.run([sys.executable, "-c", _map_extractor(), str(artifact)],
                         capture_output=True, text=True, timeout=30).stdout
    fields = dict(pair.split("=", 1) for pair in out.split())

    assert fields["map_duplicate_wall_extent_m"] == "1.42"
    assert fields["map_median_wall_thickness_m"] == "0.02"
    assert fields["map_score_passed"] == "false"
    assert "bounding" not in out, "a context-only row must not become a field"


def test_the_extractor_survives_a_malformed_row(tmp_path) -> None:
    """Fail-open: a scorer change must not take the run's manifest with it."""

    artifact = tmp_path / "map-score.json"
    artifact.write_text(json.dumps({"score": {"passed": True, "rows": [
        {"metric": "duplicate wall extent", "measured": "n/a"},
    ]}}), encoding="utf-8")

    result = subprocess.run([sys.executable, "-c", _map_extractor(), str(artifact)],
                            capture_output=True, text=True, timeout=30)

    assert result.returncode == 0, result.stderr
    assert "map_score_passed=true" in result.stdout
    assert "duplicate" not in result.stdout


def test_the_map_score_gates_nothing() -> None:
    """**The finding, pinned.** The worst map produced the best approach."""

    source = RUNNER.read_text(encoding="utf-8")
    at = source.index("map score recorded in run.json")
    window = source[at - 2000:at + 500]
    code = "\n".join(line for line in window.splitlines()
                     if not line.lstrip().startswith("#"))

    assert "rerun " not in code, "the map score aborts the run; measured 100% false-positive"
    assert "status=1" not in code.split("map_fields=")[-1], (
        "the map score sets a red status of its own"
    )
