"""The scenario that actually runs, held to the same account as the authored one.

Every other test in this package builds from `authored_config_path()` -- the
12 m scene reconciled with the supplied drawing, which is the source of record
for every RATIO in the project. None of them describe what the robot drives in.

They used to, by accident, because `load_scenario()` defaulted to the authored
file. So `scene.build` with no `--config` wrote a 12 m stage and manifest into
`out/`; `build_corridor_arena.py` defaults to those same two paths and composed
its arenas from them; and `corridor_profile_run.sh` defaulted to the same 12 m
manifest. Only an exported `CORRIDOR_MANIFEST` made a run plan at the scale it
was driving -- and on 2026-08-12 it did not: every corridor run since the
rescale drove a 0.30-scale plan inside a 1.0-scale arena, which is the whole of
that run's 5.754 m delivery error and of a landmark "confirmed" in a stage that
contains no post.

The default is now the scaled scenario. These are the tests that make that
default answerable.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml
from scene.build import ROUTE_MARGIN_DEFAULT_M, build_scene
from scene.geometry import validate_layout
from scene.model import authored_config_path, default_config_path, load_scenario
from scene.occlusion import verify
from scene.trajectory import delivery_trajectory, validate_trajectory

#: The committed scale. One number, and every length below follows from it.
SCALE_FACTOR = 0.30

#: Values `scale_scenario.py` deliberately does NOT scale, and why each one is
#: a property of something other than the scenario.
UNSCALED = {
    # policy, not geometry (ADR 0023 -- still un-pinned)
    "limit_mps",
    # B's radius is sized for the SENSOR: at 0.30 it became 0.045 m, the MS200
    # put 1.7 beams on it at 3 m, and the detector confirmed a phantom
    # (c805e26). Since ADR 0031 B is one cylinder, and its HEIGHT is
    # deliberately absent from this set -- a height describes a person and
    # scales, a detectable radius describes what the lidar can resolve and does
    # not. One object, two authorities, both named.
    "b_radius_m",
}


@pytest.fixture(scope="module")
def as_run():
    return load_scenario(default_config_path())


@pytest.fixture(scope="module")
def authored():
    return load_scenario(authored_config_path())


def test_the_default_scenario_is_the_one_that_runs(as_run) -> None:
    """A default nothing runs is a trap, and this one was."""

    assert default_config_path().name == "corridor-robot-scale.yaml"
    assert default_config_path() != authored_config_path()
    assert as_run.corridor_length_m == pytest.approx(3.6)


def test_scaling_preserves_every_ratio_in_the_scenario() -> None:
    """The argument for a uniform factor, checked rather than asserted.

    The corridor was tightened by SCALE, not by editing the profile widths,
    because editing widths changes the ratios and every geometric argument in
    this project is built on them -- doing it broke the ADR 0019 occlusion
    certificate outright. This walks both YAMLs and requires the factor to be
    exactly uniform over lengths, with the unscaled set named.
    """

    authored_raw = yaml.safe_load(authored_config_path().read_text(encoding="utf-8"))
    scaled_raw = yaml.safe_load(default_config_path().read_text(encoding="utf-8"))

    offenders: list[str] = []

    def walk(left, right, path: str = "") -> None:
        if isinstance(left, dict) and isinstance(right, dict):
            for key in left:
                if key in right:
                    walk(left[key], right[key], f"{path}.{key}")
        elif isinstance(left, list) and isinstance(right, list) and len(left) == len(right):
            for index, (a, b) in enumerate(zip(left, right, strict=True)):
                walk(a, b, f"{path}[{index}]")
        elif isinstance(left, (int, float)) and not isinstance(left, bool) and left:
            leaf = path.rsplit(".", 1)[-1].split("[")[0]
            # Only metre-denominated leaves are lengths. Angles, pixels, rates,
            # counts and fractions are scale-free by construction.
            if not leaf.endswith("_m"):
                return
            if leaf in UNSCALED:
                if right != left:
                    offenders.append(f"{path}: {left} -> {right} (must not scale)")
                return
            if abs(right / left - SCALE_FACTOR) > 1e-6:
                offenders.append(f"{path}: {left} -> {right} (ratio {right / left:.4f})")

    walk(authored_raw, scaled_raw)
    assert not offenders, "not a uniform scaling:\n  " + "\n  ".join(offenders)


def test_the_as_run_route_admits_robot1_at_every_profile(as_run) -> None:
    """0.3 m of half-width described a vehicle 3.75x wider than robot1.

    It cost nothing in a 6 m corridor and rejected all three profiles in a
    1.8 m one -- so the scenario-as-run failed its own validator, which is why
    nothing was building it.
    """

    assert ROUTE_MARGIN_DEFAULT_M == 0.128, "robot1's circumscribed radius, ADR 0029"
    for profile in as_run.profiles:
        validate_layout(as_run, profile)
        validate_trajectory(
            as_run, profile, delivery_trajectory(as_run, profile),
            margin_m=ROUTE_MARGIN_DEFAULT_M,
        )


def test_the_old_margin_is_the_negative_control(as_run) -> None:
    """If 0.3 ever starts passing, this test has stopped testing anything."""

    profile = next(p for p in as_run.profiles if p.name == "nominal_m6_n3")
    with pytest.raises(ValueError):
        validate_trajectory(
            as_run, profile, delivery_trajectory(as_run, profile), margin_m=0.3,
        )


def test_the_authored_widths_are_not_a_profile_of_the_scaled_scenario(as_run) -> None:
    """Asking for (6.0, 3.0) here is asking for a 6 m entry in a 3.6 m corridor.

    `resolve_profiles` APPENDS an unmatched (m, n) as a new profile rather than
    failing, so this does not raise -- it silently authors nonsense. Pinned so
    that a caller passing the authored numbers is a visible mistake.
    """

    assert {p.name for p in as_run.profiles} == {
        "nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6",
    }
    nominal = next(p for p in as_run.profiles if p.name == "nominal_m6_n3")
    assert (nominal.entry_width_m, nominal.corner_width_m) == (1.8, 0.9)


def test_the_as_run_route_length_is_pinned(as_run) -> None:
    """7.380 m at nominal. The landmark containment window is derived from it."""

    lengths = {
        profile.name: round(delivery_trajectory(as_run, profile).length_m, 3)
        for profile in as_run.profiles
    }
    assert lengths == {
        "nominal_m6_n3": 7.380,
        "wide_corner_m6_n4_5": 7.146,
        "uniform_m6_n6": 6.923,
    }


def test_the_occlusion_certificate_holds_at_the_scale_that_runs(tmp_path: Path) -> None:
    """ADR 0019's program, proved on the scenario the robot actually drives.

    It was only ever proved on the authored one. A uniform similarity preserves
    every sightline, so this should hold by construction -- which is exactly why
    it is worth asserting: it is the check that catches a scale that was not
    uniform after all.
    """

    nominal = next(
        p for p in load_scenario(default_config_path()).profiles if p.name == "nominal_m6_n3"
    )
    stage_path, manifest_path = build_scene(
        None, tmp_path / "corridor.usda", nominal.entry_width_m, nominal.corner_width_m
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    for name in manifest["profiles"]:
        result = verify(stage_path, manifest_path, name)
        assert result.passed, f"profile {name}: {result.camera_visible_intervals}"
        assert result.camera_visible_intervals == ()
        assert result.usd_audit_failures == 0


def test_building_with_no_config_writes_the_scenario_that_runs(tmp_path: Path) -> None:
    """The end of the trap: `scene.build` with no --config is now the as-run scene.

    The composer and the runner both default to the paths this writes.
    """

    _, manifest_path = build_scene(None, tmp_path / "corridor.usda", 1.8, 0.9)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["corridor_length_m"] == pytest.approx(3.6)
    assert manifest["selected_profile"] == "nominal_m6_n3"
    # B present and at robot scale. The arenas built before this had B twelve
    # metres further out and no detectable body at all.
    assert manifest["actors"]["b_xyz_m"][0] == pytest.approx(5.038, abs=1e-3)
    # ADR 0031: one object. The radius the detector fits and the position the
    # goal is derived from now describe the same prim, and the radius is the
    # sensor's while the height is the person's.
    assert manifest["actors"]["b_radius_m"] == pytest.approx(0.12)
    assert manifest["actors"]["b_height_m"] == pytest.approx(1.7 * SCALE_FACTOR)
    assert "landmark_radius_m" not in manifest["actors"]
    assert "landmark_xyz_m" not in manifest["actors"]


def test_b_is_solid_because_the_delivery_is_a_contact(tmp_path: Path) -> None:
    """**B carried no collider at all until ADR 0033.**

    Every wall has had one since the beginning. B never did, and it did not
    matter while arrival was a distance: the RTX lidar traces render geometry,
    so A could always SEE B, and Nav2 kept clear because the scan made B a
    lethal costmap cell. B was visible, avoidable and intangible at the same
    time -- A would have driven straight through it.

    A bump needs something to bump. Static like the walls, so B does not topple
    or slide when a 0.2 m robot leans on it: the collider is applied, and no
    rigid body is.
    """

    from pxr import Usd, UsdPhysics

    stage_path, _ = build_scene(None, tmp_path / "corridor.usda", 1.8, 0.9)
    stage = Usd.Stage.Open(str(stage_path))
    b = stage.GetPrimAtPath("/World/Actors/B")

    assert b and b.IsValid(), "B must exist"
    assert b.HasAPI(UsdPhysics.CollisionAPI), "B must be solid -- the delivery is a contact"
    assert UsdPhysics.CollisionAPI(b).GetCollisionEnabledAttr().Get() is True
    # Static, not dynamic. A rigid body would let A push B down the street.
    assert not b.HasAPI(UsdPhysics.RigidBodyAPI), "B must not be pushable"
