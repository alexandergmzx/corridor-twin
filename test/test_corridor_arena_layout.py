"""Pin the arena composer's path resolution against the symlink trap (D5).

This repository is consumed twice: directly, and as
``robot-fleet/src/corridor-twin``, which is a symlink to the same checkout. The
composer imports ``author_lidar`` from a sibling repository through that fleet
path, and that import is the only thing carrying the C1 lidar constants -- so a
resolver that leaves the logical path loses the sensor contract, not merely a
convenience.

Two distinct mistakes produce that loss, and both are silent:

``realpath``
    Resolves the symlink to ``~/Development/omniverse_twin``, whose parent
    directory holds no sibling repositories at all.

``abspath`` on a relative path
    ``os.getcwd()`` resolves symlinks even when the shell's own cwd is the
    logical path, so this lands in exactly the same wrong place.

Every test here runs against a synthetic fleet built in a tmpdir rather than the
real one. A test asserting against the developer's own checkout would pass on
this machine and prove nothing about the resolver.

The realpath resolver below is a NEGATIVE CONTROL, not dead code. It is the
implementation the composer's docstring forbids, and
``test_a_realpath_resolver_loses_the_sibling`` asserts it actually fails on this
fixture. Without it the positive assertions would be equally satisfied by a
fixture with no symlink in it -- the same skip-never-pass rule the isolation and
occlusion gates run under.
"""

from __future__ import annotations

import ast
import os
import sys
from pathlib import Path

import pytest

# Deliberately not `.resolve()`: this file is about not resolving symlinks, and
# the house pattern for reaching tools/ is a path insert (test_viewpoints.py).
ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import build_corridor_arena as composer  # noqa: E402

TOOL_SOURCE = ROOT / "tools" / "build_corridor_arena.py"


@pytest.fixture
def fleet(tmp_path: Path) -> dict[str, Path]:
    """A miniature of the real layout: a physical checkout, symlinked into src/.

    The symlink target is relative and two levels up, mirroring the real
    ``../../omniverse_twin`` exactly; an absolute-target symlink resolves
    differently and would not reproduce the trap.
    """

    physical = tmp_path / "physical" / "omniverse_twin"
    (physical / "tools").mkdir(parents=True)
    (physical / "tools" / "build_corridor_arena.py").write_text("# stand-in\n", encoding="utf-8")

    src = tmp_path / "fleet" / "src"
    (src / "yahboomcar-ros2" / "tools").mkdir(parents=True)
    (src / "yahboomcar-ros2" / "tools" / "build_arena.py").write_text("", encoding="utf-8")

    link = src / "corridor-twin"
    link.symlink_to(os.path.join("..", "..", "physical", "omniverse_twin"))

    return {
        "src": src,
        "physical": physical,
        # The path a caller standing in the fleet layout actually uses.
        "logical_tool": link / "tools" / "build_corridor_arena.py",
    }


def _realpath_resolver(start: str) -> str:
    """The forbidden implementation, kept so the guard can be shown to fire."""

    here = os.path.realpath(start)
    return os.path.normpath(os.path.join(os.path.dirname(here), os.pardir, os.pardir))


def test_the_logical_walk_reaches_the_sibling_repositories(fleet, monkeypatch) -> None:
    monkeypatch.delenv(composer.FLEET_SRC_ENV, raising=False)

    root = composer.fleet_src_root(str(fleet["logical_tool"]))

    assert Path(root) == fleet["src"]
    assert Path(composer.yahboom_tools(str(fleet["logical_tool"]))).is_dir()


def test_a_realpath_resolver_loses_the_sibling(fleet) -> None:
    """The negative control. If this passes, the fixture stopped testing anything."""

    lost = Path(_realpath_resolver(str(fleet["logical_tool"])))

    assert lost != fleet["src"], (
        "realpath resolved to the fleet src/, so this fixture no longer reproduces "
        "the symlink trap and the positive tests above prove nothing"
    )
    assert not (lost / "yahboomcar-ros2" / "tools").is_dir(), (
        "the realpath walk found the sibling anyway; the trap is not reproduced"
    )


def test_the_composer_never_resolves_symlinks() -> None:
    """A source-level guard, because one careless edit re-introduces the trap.

    Walked as a syntax tree rather than grepped: the composer's docstring names
    ``realpath`` several times explaining why it is absent, so a text search
    would either trip on the prose or be loose enough to miss a real call.
    """

    tree = ast.parse(TOOL_SOURCE.read_text(encoding="utf-8"))
    offenders = [
        f"line {node.lineno}: {ast.unparse(node.func)}"
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"realpath", "resolve"}
    ]

    assert offenders == [], (
        f"the composer resolves symlinks, which escapes the fleet checkout: {offenders}"
    )


def test_a_relative_invocation_does_not_fall_through_to_the_physical_cwd(
    fleet, monkeypatch
) -> None:
    """The second trap: `os.getcwd()` is physical even when the shell's cwd is not.

    Simulates standing *in* the symlinked tools directory and invoking the tool
    by a bare relative name, which is what `python tools/build_corridor_arena.py`
    does. `$PWD` is the logical value the shell exports; the process cwd is the
    physical one it resolved to.
    """

    monkeypatch.delenv(composer.FLEET_SRC_ENV, raising=False)
    logical_tools = fleet["src"] / "corridor-twin" / "tools"
    monkeypatch.chdir(logical_tools)
    monkeypatch.setenv("PWD", str(logical_tools))

    # Precondition: the process cwd really did resolve away from the logical path.
    assert Path(os.getcwd()) == fleet["physical"] / "tools"

    root = composer.fleet_src_root("build_corridor_arena.py")

    assert Path(root) == fleet["src"]


def test_an_already_absolute_dunder_file_does_not_decide_the_walk(fleet, monkeypatch) -> None:
    """The trap that actually bit on the first live run, pinned so it cannot return.

    Since Python 3.9 `__file__` is always absolute, made so against the PROCESS
    cwd -- so inside the symlinked checkout it arrives already rewritten to the
    physical path, with no realpath call anywhere. Resolution therefore starts
    from `sys.argv[0]`, the string the caller typed. Here argv[0] is the
    relative name a `python tools/build_corridor_arena.py` invocation produces
    while `__file__` points at the physical copy, which is precisely the
    combination that failed.
    """

    monkeypatch.delenv(composer.FLEET_SRC_ENV, raising=False)
    checkout = fleet["src"] / "corridor-twin"
    monkeypatch.chdir(checkout)
    monkeypatch.setenv("PWD", str(checkout))
    # Exactly what `python tools/build_corridor_arena.py` puts in argv[0].
    monkeypatch.setattr(composer.sys, "argv", ["tools/build_corridor_arena.py"])

    # The module's own __file__ is the real one in this checkout, i.e. useless
    # for the fixture -- which is the point: argv[0] must be what decides.
    assert Path(composer.this_file()) == checkout / "tools" / "build_corridor_arena.py"
    assert Path(composer.fleet_src_root()) == fleet["src"]


def test_an_absolute_argv0_is_taken_as_given(fleet, monkeypatch) -> None:
    """No guessing: a caller who names a path explicitly gets that path.

    The physical form then fails loudly in `yahboom_tools` rather than being
    silently repaired, because a tool that quietly relocates its own inputs is
    how the sibling import goes missing without anyone noticing.
    """

    monkeypatch.delenv(composer.FLEET_SRC_ENV, raising=False)
    physical_tool = fleet["physical"] / "tools" / "build_corridor_arena.py"
    monkeypatch.setattr(composer.sys, "argv", [str(physical_tool)])

    assert Path(composer.fleet_src_root()) == fleet["physical"].parent

    with pytest.raises(FileNotFoundError):
        composer.yahboom_tools()


def test_a_stale_pwd_is_not_trusted(fleet, monkeypatch) -> None:
    """`$PWD` is believed only when it names the same directory as the process cwd.

    An inherited `$PWD` from another directory would otherwise silently redirect
    every path this tool resolves.
    """

    monkeypatch.delenv(composer.FLEET_SRC_ENV, raising=False)
    monkeypatch.chdir(fleet["physical"] / "tools")
    monkeypatch.setenv("PWD", str(fleet["src"] / "yahboomcar-ros2"))

    assert Path(composer.logical_cwd()) == Path(os.getcwd())


def test_the_environment_override_wins(fleet, monkeypatch) -> None:
    """Environment first, per the `_layout.py` contract."""

    monkeypatch.setenv(composer.FLEET_SRC_ENV, str(fleet["src"]))

    root = composer.fleet_src_root("/somewhere/else/entirely/tools/x.py")

    assert Path(root) == fleet["src"]


def test_the_missing_layout_is_an_error_that_names_the_path(tmp_path, monkeypatch) -> None:
    """A silent fallback here becomes a missing lidar contract three steps later."""

    monkeypatch.setenv(composer.FLEET_SRC_ENV, str(tmp_path))

    with pytest.raises(FileNotFoundError) as caught:
        composer.yahboom_tools()

    assert str(tmp_path) in str(caught.value)


def test_the_c1_contract_matches_the_figures_the_fleet_measured() -> None:
    """These constants are the reason the sibling import has to keep working.

    ``minDistBetweenEchosM`` is checked separately from the rest because
    ``author_lidar`` hardcodes it rather than accepting it as a keyword: the
    composer verifies it on readback, so the value it verifies against must be
    the fleet's measured one and not a second opinion.
    """

    assert (composer.C1_BEAMS, composer.C1_HZ) == (500, 10)
    assert composer.C1_RANGE == (0.05, 12.0)
    assert composer.C1_XYZ == (0.08, 0.0, 0.10)
    assert composer.C1_NAME == "c1_lidar"
    assert composer.C1_MIN_ECHO_M == 0.05


def test_every_authored_profile_is_offered(tmp_path) -> None:
    """The composer must cover all three, not just the default one."""

    assert composer.PROFILES == ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")


def test_the_arena_env_is_the_one_the_rasptank_runner_reads() -> None:
    """The composer's output is only useful if the runner's hook is what it names."""

    assert composer.ARENA_ENV == "RASPTANK_ARENA_USD"
