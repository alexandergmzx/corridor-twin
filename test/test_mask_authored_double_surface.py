"""The mask, and the thing a mask must never do.

Hiding a region from a detector is a dangerous kind of fix, so the tests here
are weighted toward what it must STILL catch. The oracle going to 0.000 is one
assertion; the rest are controls.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from mask_authored_double_surface import (  # noqa: E402
    MASKED_WALLS,
    UNKNOWN,
    mask,
    polygons_from_manifest,
    read_map,
    write_map,
)

MANIFEST = ROOT / "out/corridor.manifest.json"
SCORER = Path.home() / "Development/robot-fleet/src/yahboomcar-ros2/tools/score_slam_map.py"
NOMINAL = "nominal_m6_n3"
LIMIT_M = 0.20


def _oracle(tmp_path: Path) -> Path:
    """Render the authored 'perfect SLAM' map: no sensor, no drift, no error."""

    out = tmp_path / "oracle.yaml"
    subprocess.run(
        [sys.executable, "tools/authored_reference_map.py", "--profile", NOMINAL,
         "--resolution", "0.02", "--out", str(out)],
        cwd=ROOT, check=True, capture_output=True,
    )
    return out


def _duplicate_extent_m(map_path: Path) -> float:
    done = subprocess.run(
        [sys.executable, str(SCORER), "--map", str(map_path)],
        capture_output=True, text=True, check=False,
    )
    line = next(row for row in done.stdout.splitlines() if "duplicate wall extent" in row)
    return float(line.split()[3])


def _needs_scorer() -> None:
    if not SCORER.exists():
        pytest.skip("fleet scorer not present")
    if not MANIFEST.exists():
        pytest.skip("out/corridor.manifest.json is a generated artifact")


def test_the_perfect_map_fails_the_metric_unmasked(tmp_path: Path) -> None:
    """The reason this tool exists, asserted rather than remembered.

    A scene with no sensor and no drift in it reads 0.340 m of 'duplicate wall'
    against a 0.20 m limit, because the metric asks whether anything stands
    within 0.40 m of the outermost wall -- and two things do, on purpose.
    """

    _needs_scorer()
    assert _duplicate_extent_m(_oracle(tmp_path)) == pytest.approx(0.340, abs=0.02)


def test_masking_the_two_authored_features_returns_the_oracle_to_zero(
    tmp_path: Path,
) -> None:
    """The acceptance: a perfect map scores a perfect 0.000, so the 0.20 m
    limit measures a run's error and nothing else. No subtraction anywhere."""

    _needs_scorer()
    oracle = _oracle(tmp_path)
    report = mask(oracle, MANIFEST, NOMINAL, "world", tmp_path / "masked.yaml")

    assert _duplicate_extent_m(tmp_path / "masked.yaml") == pytest.approx(0.0, abs=1e-9)
    # Small, and the number is in the artifact rather than in prose.
    assert report["masked_fraction"] < 0.01
    assert report["walls"] == list(MASKED_WALLS)


def test_the_mask_still_catches_ghosting_outside_it(tmp_path: Path) -> None:
    """**The control that matters.** A mask that hides real divergence is worse
    than no metric at all.

    A duplicate wall is painted along the NORTH wall -- the far side of the map
    from both masked polygons -- and the masked map must still convict it.
    """

    _needs_scorer()
    oracle = _oracle(tmp_path)
    grid, resolution, origin_x, origin_y, _ = read_map(oracle)
    height = len(grid)

    # Paint a second line 0.10 m south of the north wall, 1.5 m long, in the
    # corridor's western half: inside the scorer's band, far outside the masks.
    ghost_y, ghost_from_x, ghost_to_x = 0.75, 0.5, 2.0
    row = height - 1 - int(round((ghost_y - origin_y) / resolution))
    for column in range(
        int(round((ghost_from_x - origin_x) / resolution)),
        int(round((ghost_to_x - origin_x) / resolution)),
    ):
        grid[row][column] = 0
    ghosted = tmp_path / "ghosted.yaml"
    write_map(grid, resolution, origin_x, origin_y, ghosted)

    # It is a real defect before masking...
    assert _duplicate_extent_m(ghosted) > LIMIT_M
    # ...and it is still a real defect after.
    mask(ghosted, MANIFEST, NOMINAL, "world", tmp_path / "ghosted-masked.yaml")
    surviving = _duplicate_extent_m(tmp_path / "ghosted-masked.yaml")
    assert surviving > LIMIT_M, (
        f"the mask swallowed a painted duplicate wall: {surviving} m"
    )


def test_the_masked_list_is_complete_and_no_longer() -> None:
    """Two, and exactly two. Masking a third authored wall changes nothing,
    which is how the list is known not to be 'whatever it took to go green'."""

    _needs_scorer()
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    walls = manifest["profiles"][NOMINAL]["walls"]

    assert set(MASKED_WALLS) <= set(walls), "the mask names a wall the scene does not author"
    # Every other authored wall is an OUTER wall the metric is meant to see.
    assert set(walls) - set(MASKED_WALLS) == {
        "NorthBuilding", "SouthBuilding", "EastBuilding", "CornerBuilding",
    }


def test_dropping_either_feature_leaves_the_oracle_failing(tmp_path: Path) -> None:
    """Neither entry is padding: removing either one puts the oracle back over
    the limit, so both are load-bearing and the pair is minimal."""

    _needs_scorer()
    oracle = _oracle(tmp_path)
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    import mask_authored_double_surface as module

    original = module.MASKED_WALLS
    try:
        for single in original:
            module.MASKED_WALLS = (single,)
            out = tmp_path / f"only-{single}.yaml"
            mask(oracle, MANIFEST, NOMINAL, "world", out)
            assert _duplicate_extent_m(out) > LIMIT_M, (
                f"masking {single} alone already passes; the pair is not minimal"
            )
    finally:
        module.MASKED_WALLS = original
    assert manifest["profiles"][NOMINAL]["walls"]  # the fixture was real


def test_the_two_frames_are_not_the_same_polygon() -> None:
    """A silently wrong frame would mask empty space and change nothing.

    The oracle is world-framed; a SLAM map starts at A's spawn pose.
    """

    if not MANIFEST.exists():
        pytest.skip("out/corridor.manifest.json is a generated artifact")

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    world = polygons_from_manifest(manifest, NOMINAL, "world")
    robot = polygons_from_manifest(manifest, NOMINAL, "robot_start")

    assert len(world) == len(robot) == len(MASKED_WALLS)
    assert world != robot, "the frames coincide, so --frame is not doing anything"


def test_masking_only_erases_and_never_invents(tmp_path: Path) -> None:
    """Cells become UNKNOWN. Nothing becomes occupied, and the map keeps its
    shape -- a mask that resized the grid would move every other metric."""

    if not MANIFEST.exists():
        pytest.skip("out/corridor.manifest.json is a generated artifact")

    oracle = _oracle(tmp_path)
    before, resolution, origin_x, origin_y, _ = read_map(oracle)
    mask(oracle, MANIFEST, NOMINAL, "world", tmp_path / "masked.yaml")
    after, after_res, after_x, after_y, _ = read_map(tmp_path / "masked.yaml")

    assert (after_res, after_x, after_y) == (resolution, origin_x, origin_y)
    assert len(after) == len(before) and len(after[0]) == len(before[0])
    for row_before, row_after in zip(before, after, strict=True):
        for value_before, value_after in zip(row_before, row_after, strict=True):
            assert value_after in (value_before, UNKNOWN)
