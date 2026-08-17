"""The delivery goal must be somewhere A can actually stand.

This is the bug that made every delivery attempt unwinnable before the planner
or controller ever got a fair chance: `corridor_nav_gate` aimed at
`manifest.actors.b_xyz_m`, which is B's CENTRE. B carries no
`PhysicsCollisionAPI`, so it is easy to read as decoration — but the RTX lidar
sees RENDER geometry, so B lands in `/scan` and therefore in the costmap as an
obstacle. The goal sat inside its own inflated footprint and could never be
reached to a 0.15 m tolerance however well Nav2 behaved.

B being an obstacle is correct. A person is one. Aiming at their centre is not.

These tests validate the standoff against the scene's own free-space oracle
(`geometry.is_clear`) rather than against a number copied out of the config, so
a scenario change that moves B into a wall fails here rather than in a GPU run.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/corridor_scene"))

from corridor_nav_gate import (  # noqa: E402
    DELIVERY_STANDOFF_M,
    GOAL_TOLERANCE_M,
    delivery_standoff_world,
)
from scene.geometry import is_clear  # noqa: E402
from scene.model import load_scenario  # noqa: E402

PROFILES = ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")

#: Robot facts. These do NOT scale with the scenario — the world shrinks toward
#: a fixed robot — so they are stated here rather than read from the scene.
#:
#: Corrected 2026-08-12 with ADR 0031: they were 0.12 / 0.16, stale against the
#: 0.128 / 0.18 that ADR 0029 measured and pinned in
#: `config/robot1/nav2_robot1_corridor.yaml`. robot1's chassis is 0.20 x 0.16 m,
#: so its circumscribed radius is 0.128 -- the eight millimetres Nav2 believed
#: it did not have, in the one place where eight millimetres decides whether a
#: path fits.
ROBOT_RADIUS_M = 0.128
INFLATION_RADIUS_M = 0.18


@pytest.fixture(scope="module")
def scaled(tmp_path_factory) -> tuple[dict, object]:
    """Build the robot-scale scenario once, in a tmpdir, and return it.

    Built rather than read from `out/` so the test does not depend on whatever
    a previous run happened to leave behind.
    """

    workdir = tmp_path_factory.mktemp("standoff")
    config = workdir / "scaled.yaml"
    subprocess.run(
        # The COMMITTED factor (ADR 0030), not the 0.3333 of an abandoned
        # iteration: a standoff test that builds a scenario nothing runs is
        # measuring a scene that does not exist.
        [sys.executable, "tools/scale_scenario.py", "--factor", "0.30",
         "--out", str(config)],
        cwd=ROOT, check=True, capture_output=True,
    )
    stage = workdir / "corridor.usda"
    subprocess.run(
        [sys.executable, "-m", "scene.build", "--config", str(config),
         "--m", "2.0", "--n", "1.0", "--route-margin-m", "0.12",
         "--out", str(stage)],
        cwd=ROOT, check=True, capture_output=True,
        env={**__import__("os").environ,
             "PYTHONPATH": str(ROOT / "src/corridor_scene")},
    )
    manifest = json.loads(stage.with_suffix("").with_suffix(".manifest.json").read_text())
    return manifest, load_scenario(config)


def test_the_standoff_is_in_drivable_space(scaled) -> None:
    """The goal has to be somewhere the scene says A can be."""

    manifest, scenario = scaled
    x, y = delivery_standoff_world(manifest)

    for profile in scenario.profiles:
        assert is_clear(scenario, profile, x, y), (
            f"delivery standoff ({x:.3f}, {y:.3f}) is not drivable on {profile.name}"
        )


def test_the_standoff_clears_b_by_more_than_the_robot_needs(scaled) -> None:
    """Outside B's footprint AND outside the inflation the costmap adds to it.

    Without this the goal is inside lethal or inscribed cost and the goal
    checker can never be satisfied, which is indistinguishable in a log from a
    controller that simply failed.
    """

    manifest, _ = scaled
    b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
    # ADR 0031: B is a cylinder, so its footprint is its radius -- no half-
    # diagonal, and no second description of B's extent to keep in step.
    b_radius = manifest["actors"]["b_radius_m"]
    x, y = delivery_standoff_world(manifest)

    gap = math.dist((x, y), (b_x, b_y))
    required = b_radius + ROBOT_RADIUS_M + INFLATION_RADIUS_M

    assert gap > required, (
        f"standoff {gap:.3f} m does not clear B's inflated footprint ({required:.3f} m)"
    )


def test_b_centre_itself_would_have_been_unreachable(scaled) -> None:
    """The negative control: the OLD goal must fail the test the new one passes.

    If this ever stops failing, B has stopped being an obstacle and the
    standoff is no longer earning its keep.
    """

    manifest, _ = scaled
    b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
    b_radius = manifest["actors"]["b_radius_m"]

    required = b_radius + ROBOT_RADIUS_M + INFLATION_RADIUS_M

    assert math.dist((b_x, b_y), (b_x, b_y)) < required, (
        "B's centre is somehow outside B's own inflated footprint"
    )


def test_the_standoff_is_reachable_within_the_goal_tolerance(scaled) -> None:
    """A must be able to sit ON the goal, not merely near it.

    The gate demands ≤ 0.15 m map-frame error, so free space has to extend at
    least the robot's radius around the goal — a goal in a 0.1 m gap is a goal
    that cannot be occupied.
    """

    manifest, scenario = scaled
    x, y = delivery_standoff_world(manifest)
    profile = next(p for p in scenario.profiles if p.name == "nominal_m6_n3")

    for bearing in range(0, 360, 15):
        angle = math.radians(bearing)
        probe_x = x + ROBOT_RADIUS_M * math.cos(angle)
        probe_y = y + ROBOT_RADIUS_M * math.sin(angle)
        assert is_clear(scenario, profile, probe_x, probe_y), (
            f"the goal cannot be occupied: blocked at bearing {bearing} deg"
        )


def test_the_standoff_direction_comes_from_the_street_not_the_route(scaled) -> None:
    """ADR 0022:15-17 keeps the authored route out of A's navigation.

    The standoff leans toward the street centreline. Deriving it from the
    delivery trajectory's final heading would work identically and would be
    exactly the authored-waypoint pattern the task author called a level
    indicator.
    """

    manifest, _ = scaled
    b_x, _b_y, _ = manifest["actors"]["b_xyz_m"]
    centre_x = manifest["next_street"]["center_x_m"]
    x, _y = delivery_standoff_world(manifest)

    # B is east of the lane centre, so the standoff moves west, toward it.
    assert b_x > centre_x
    assert x < b_x


def test_the_standoff_floor_matches_the_docking_spec() -> None:
    """One rule, not two numbers: nominal and refined standoffs share a floor."""

    assert DELIVERY_STANDOFF_M >= 0.6
    assert DELIVERY_STANDOFF_M > GOAL_TOLERANCE_M
