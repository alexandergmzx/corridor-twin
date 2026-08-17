"""The one-line entry point resolves the three defaults that are wrong.

`tools/demo.sh` exists because three scripts it calls default to something that
runs, finishes, and writes plausible artifacts for the WRONG scenario:

  1. `corridor_profile_run.sh` defaults `--robot` to robot2 -- the twin ADR 0027
     rejected.
  2. `run_demo.sh` defaults STAGE to the v1 stage and SPEED_MPS to 1.0, roughly
     five times A's top measured speed. Its own header says in capitals that a
     bare run is not the v2 demonstration.
  3. `build_corridor_arena.py` defaults `--robot` to rasptank, so it writes an
     arena under a name the enforcement pass does not open.

None of the three errors. That is the point: a reader who forgets a flag gets a
confident wrong answer, which is the failure mode this repository keeps finding
and this file pins.

Assertions are source-level on purpose, in the style of `test_demo_launch.py`
and `test_corridor_profile_run.py`. The alternative is a multi-minute Isaac run
per assertion, and the composition is what can regress -- the dry-run tests
below exercise it end to end without a GPU.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
DEMO = ROOT / "tools" / "demo.sh"
ARENA_BUILDER = ROOT / "tools" / "build_corridor_arena.py"
PROFILE_RUNNER = ROOT / "tools" / "corridor_profile_run.sh"


def _run(*args: str, dry: bool = True) -> subprocess.CompletedProcess[str]:
    env = dict(os.environ)
    if dry:
        env["CORRIDOR_DEMO_DRY_RUN"] = "1"
    return subprocess.run(
        ["bash", str(DEMO), *args],
        capture_output=True,
        text=True,
        env=env,
        cwd=ROOT,
        timeout=60,
    )


def test_the_entry_point_is_executable_and_names_both_runs() -> None:
    assert DEMO.exists(), "tools/demo.sh is the documented entry point"
    assert os.access(DEMO, os.X_OK), "it is invoked as a command, so it is executable"
    text = DEMO.read_text(encoding="utf-8")
    assert "deliver" in text and "enforce" in text


def test_a_bare_invocation_refuses_rather_than_guessing() -> None:
    """Picking a default subcommand would run one of two very different things."""
    result = _run()
    assert result.returncode != 0
    combined = result.stdout + result.stderr
    assert "deliver" in combined and "enforce" in combined


def test_an_unknown_subcommand_refuses() -> None:
    result = _run("nonsense")
    assert result.returncode != 0
    assert "unknown subcommand" in (result.stdout + result.stderr)


def test_deliver_names_robot1_and_never_robot2() -> None:
    """ADR 0027: A is robot1. The runner's own default is robot2."""
    result = _run("deliver")
    assert result.returncode == 0, result.stderr
    assert "--robot robot1" in result.stdout
    assert "robot2" not in result.stdout


def test_deliver_overrides_the_contract_precondition_that_fails_on_this_host() -> None:
    """robot1's twin publishes /scan off its declared rate; without the flag the
    runner classifies that as INFRASTRUCTURE and refuses to start."""
    result = _run("deliver")
    assert "--allow-contract-fail" in result.stdout


def test_deliver_uses_canonical_slam_unless_asked_for_the_recorded_arm() -> None:
    """--corridor-slam selects a params file whose own first line reads
    "NOT IN USE, kept as a record". It is opt-in, not the default."""
    default = _run("deliver")
    assert "--corridor-slam" not in default.stdout
    assert "canonical" in default.stdout

    recorded = _run("deliver", "--as-recorded")
    assert "--corridor-slam" in recorded.stdout


def test_enforce_opens_a_composed_arena_and_never_the_v1_stage() -> None:
    """run_demo.sh falls back to out/corridor.usda, which is v1's kinematic box.
    A run against it looks entirely normal and demonstrates the wrong thing."""
    result = _run("enforce")
    assert result.returncode == 0, result.stderr
    assert "arena_corridor_robot1_nominal_m6_n3.usd" in result.stdout

    stage = re.search(r"STAGE=(\S+)", result.stdout)
    assert stage, "the enforce command must set STAGE explicitly"
    assert not stage.group(1).endswith("corridor.usda")


def test_enforce_drives_a_at_its_measured_speed_not_v1s() -> None:
    """A's measured band is 0.056-0.207 m/s. run_demo.sh defaults to 1.0."""
    result = _run("enforce")
    assert "SPEED_MPS=0.22" in result.stdout
    assert "SPEED_MPS=1.0" not in result.stdout


def test_enforce_sets_the_whole_f3_1_environment() -> None:
    """Every figure in DELIVERY.md's speed table came from this environment
    (docs/evidence/ship-day/NOTES.md). A partial set silently changes the run."""
    result = _run("enforce")
    for key in (
        "MANIFEST=",
        "CORRIDOR_PROFILE=nominal_m6_n3",
        "ROBOT_PRIM=/World/Robot",
        "DEACTIVATE_PHYSICS=1",
        "UPDATES=3000",
    ):
        assert key in result.stdout, f"missing {key} from the F3.1 environment"


def test_enforce_writes_bulk_output_under_out_and_never_into_committed_evidence() -> None:
    result = _run("enforce")
    evidence = re.search(r"EVIDENCE_DIR=(\S+)", result.stdout)
    assert evidence, "the run must direct its own output"
    path = evidence.group(1)
    assert "/out/evidence/" in path
    assert "docs/evidence" not in path


def test_a_dry_run_needs_neither_isaac_nor_a_built_arena() -> None:
    """The dry run must be hermetic, or it cannot do its job.

    CI has no GPU, no Isaac interpreter, and no `out/` -- the arena is generated
    and gitignored. The first version of this script called its Isaac-Python
    guard before the dry-run branch, so `enforce` died on CI while passing on a
    developer host that happened to have both. That is the same
    local-has-state-CI-lacks failure this repository has now hit three times.
    """
    env = dict(os.environ)
    env["CORRIDOR_DEMO_DRY_RUN"] = "1"
    env["ISAAC_PYTHON"] = "/nonexistent/isaac/python"
    result = subprocess.run(
        ["bash", str(DEMO), "enforce"],
        capture_output=True,
        text=True,
        env=env,
        cwd=ROOT,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert "SPEED_MPS=0.22" in result.stdout, "the composition must still be printed"


def _enforce_body() -> str:
    text = DEMO.read_text(encoding="utf-8")
    return text[text.index("cmd_enforce()") : text.index("subcommand=")]


def test_the_arena_is_built_for_robot1_not_the_builders_rasptank_default() -> None:
    """build_corridor_arena.py defaults --robot to rasptank, which writes
    arena_corridor_rasptank_<profile>.usd -- a name enforce never opens."""
    build = re.search(r"local -a build=\((.*?)\n\s*\)", _enforce_body(), re.S)
    assert build, "enforce must invoke the arena builder in a build=() array"
    body = build.group(1)
    assert "--robot" in body and '"$ROBOT"' in body
    assert "rasptank" not in body


def test_the_arena_it_builds_is_the_arena_it_opens() -> None:
    """The invariant behind the whole guard: if the builder's --robot and the
    stage filename ever disagree, enforce composes one arena and opens another,
    which is exactly the trap the builder's rasptank default sets."""
    body = _enforce_body()
    stage = re.search(r'local stage="([^"]+)"', body)
    assert stage, "enforce must derive its stage path"
    assert "${ROBOT}" in stage.group(1), "the stage name must carry the same robot"

    build = re.search(r"local -a build=\((.*?)\n\s*\)", body, re.S)
    assert build and '"$ROBOT"' in build.group(1)


def test_enforce_takes_the_machine_wide_isaac_lock() -> None:
    """run_demo.sh starts Isaac and does NOT take the lock, unlike
    corridor_profile_run.sh. Two instances can take down the whole host."""
    text = DEMO.read_text(encoding="utf-8")
    enforce = text[text.index("cmd_enforce()") :]
    assert "isaac_lock_acquire" in enforce
    assert "isaac_lock_release" in enforce
    assert "trap" in enforce, "the lock must be released on every exit path"


def test_deliver_does_not_take_the_lock_its_child_already_holds() -> None:
    """corridor_profile_run.sh acquires the lock itself; taking it here too
    would deadlock against the child."""
    text = DEMO.read_text(encoding="utf-8")
    deliver = text[text.index("cmd_deliver()") : text.index("cmd_enforce()")]
    assert "isaac_lock_acquire" not in deliver


@pytest.mark.parametrize("profile", ["nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6"])
def test_every_authored_profile_composes(profile: str) -> None:
    result = _run("enforce", "--profile", profile)
    assert result.returncode == 0, result.stderr
    assert profile in result.stdout


def test_an_unknown_profile_is_refused_with_the_list() -> None:
    result = _run("enforce", "--profile", "does_not_exist")
    assert result.returncode != 0
    assert "nominal_m6_n3" in (result.stdout + result.stderr)


def test_the_profile_list_matches_the_arena_builders_own() -> None:
    """Two lists that must not drift: a profile valid here but absent there
    composes an arena that does not exist."""
    builder = ARENA_BUILDER.read_text(encoding="utf-8")
    declared = re.search(r"^PROFILES\s*=\s*\((.*?)\)", builder, re.M | re.S)
    assert declared
    expected = set(re.findall(r'"([^"]+)"', declared.group(1)))

    demo = DEMO.read_text(encoding="utf-8")
    known = re.search(r'^KNOWN_PROFILES="([^"]+)"', demo, re.M)
    assert known
    assert set(known.group(1).split()) == expected


def test_unrecognised_flags_reach_the_underlying_runner() -> None:
    """The entry point must not become a wall between the operator and the
    runner's own options."""
    result = _run("deliver", "--nav-timeout", "300")
    assert result.returncode == 0, result.stderr
    assert "--nav-timeout 300" in result.stdout


def test_the_runner_still_defaults_to_robot2_so_this_override_stays_necessary() -> None:
    """A canary. If the runner's default is ever corrected, this test fails and
    the override, its comment, and this file's premise should be revisited
    rather than left as folklore."""
    text = PROFILE_RUNNER.read_text(encoding="utf-8")
    assert re.search(r'^ROBOT="\$\{CORRIDOR_RUN_ROBOT:-robot2\}"', text, re.M), (
        "corridor_profile_run.sh no longer defaults to robot2 -- "
        "re-check whether demo.sh still needs to force --robot robot1"
    )
