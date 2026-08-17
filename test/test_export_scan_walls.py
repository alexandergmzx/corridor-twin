"""The scan filter's wall model, and the control that proves it was wrong before.

The acceptance for this is a live one -- the filter accepts from scan 1 with no
fail-open -- but the mechanism is checkable here, offline, against the fleet's
own raycaster and the fleet's own impossible-fraction test: a scan synthesised
from the corridor's geometry must be ACCEPTED against the corridor's walls and
REJECTED against the stock 4 x 4 m room.

If the second half ever stops failing, the room model stopped mattering and
these tests prove nothing.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from export_scan_walls import wall_segments  # noqa: E402

MANIFEST = ROOT / "out/corridor.manifest.json"
NOMINAL = "nominal_m6_n3"
FLEET_TOOLS = Path(
    "/home/alexmint/Development/robot-fleet/src/yahboomcar-ros2/tools"
)
FLEET_SIM = FLEET_TOOLS.parent / "yahboomcar_sim"

#: The relay's own gate: a scan with more than this fraction of beams returning
#: from beyond the wall model is corrupt (`_scan_frame_relay.py:66`).
IMPOSSIBLE_GATE = 0.10


def _fleet():
    if not (FLEET_TOOLS.is_dir() and FLEET_SIM.is_dir()):
        pytest.skip("the fleet layout is not in place")
    for path in (str(FLEET_TOOLS), str(FLEET_SIM)):
        if path not in sys.path:
            sys.path.insert(0, path)
    from yahboomcar_sim.arena import raycast, segments_room

    return raycast, segments_room


def _manifest() -> dict:
    if not MANIFEST.exists():
        pytest.skip("out/corridor.manifest.json is a generated artifact")
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def _corridor_walls() -> list[tuple[tuple[float, float], tuple[float, float]]]:
    return [
        ((a[0], a[1]), (b[0], b[1])) for a, b in wall_segments(_manifest(), NOMINAL)
    ]


def _impossible_fraction(ranges, pose, walls, raycast) -> float:
    """The relay's test, reproduced from `_scan_frame_relay.py:82-98`."""

    import numpy as np

    expect = raycast((pose[0], pose[1]), pose[2], walls)
    got = np.asarray(ranges, dtype=float)
    valid = np.isfinite(got) & (got > 0.0)
    both = np.isfinite(expect) & valid
    if int(both.sum()) < 90:
        return 1.0
    return float(np.mean(got[both] > expect[both] + 0.25))


def _scan_from(pose, walls, raycast):
    """A perfect scan of the corridor: what a clean revolution looks like."""

    return raycast((pose[0], pose[1]), pose[2], walls)


def test_the_export_closes_every_building() -> None:
    """Four edges per rectangle. An open polygon would let beams through a wall."""

    manifest = _manifest()
    walls = manifest["profiles"][NOMINAL]["walls"]
    segments = wall_segments(manifest, NOMINAL)

    assert len(segments) == 4 * len(walls)
    for corners in walls.values():
        first, last = corners[0], corners[-1]
        closing = [[last[0], last[1]], [first[0], first[1]]]
        assert closing in segments, "a building's polygon was not closed"


def test_a_clean_corridor_scan_is_accepted_against_the_corridor() -> None:
    """The point of the whole exercise: the filter can work here at all."""

    raycast, _ = _fleet()
    walls = _corridor_walls()
    # A pose on the approach, inside the corridor and clear of every wall.
    pose = (1.0, 0.1, 0.0)
    ranges = _scan_from(pose, walls, raycast)

    assert _impossible_fraction(ranges, pose, walls, raycast) <= IMPOSSIBLE_GATE


def test_the_same_scan_is_rejected_against_the_stock_room() -> None:
    """**The control.** This is what has been happening on every corridor run.

    The stock 4 x 4 m room is not this corridor, so a clean scan looks like it
    sees through the walls -- and the relay drops it, publishes nothing for
    ~21 s, then disables itself for the rest of the run.
    """

    raycast, segments_room = _fleet()
    walls = _corridor_walls()
    pose = (1.0, 0.1, 0.0)
    ranges = _scan_from(pose, walls, raycast)

    stock = _impossible_fraction(ranges, pose, segments_room(), raycast)
    assert stock > IMPOSSIBLE_GATE, (
        f"the stock room now accepts a corridor scan ({stock:.3f}); the control "
        "has stopped controlling for anything"
    )


def test_several_poses_along_the_route_are_all_accepted() -> None:
    """One lucky pose is not a wall model."""

    raycast, _ = _fleet()
    walls = _corridor_walls()
    manifest = _manifest()
    trajectory = manifest["profiles"][NOMINAL]["delivery_trajectory"]
    heading = trajectory["approach_heading"]
    yaw = math.atan2(heading[1], heading[0])

    for station in (0.5, 1.0, 1.5, 2.0, 2.5):
        pose = (station * heading[0], station * heading[1], yaw)
        ranges = _scan_from(pose, walls, raycast)
        measured = _impossible_fraction(ranges, pose, walls, raycast)
        assert measured <= IMPOSSIBLE_GATE, f"station {station} m: {measured:.3f}"


def test_the_relay_defaults_to_the_stock_room_and_fails_closed_on_a_bad_path(
    tmp_path: Path,
) -> None:
    """The fleet's own behaviour is unchanged, and a broken hand-off is loud.

    Falling back to the 4 x 4 m room when a caller MEANT to supply geometry
    would reproduce the exact silent failure this change ends.
    """

    _fleet()
    from _scan_frame_relay import WALLS_ENV, load_walls

    previous = os.environ.pop(WALLS_ENV, None)
    try:
        walls, source = load_walls()
        assert len(walls) == 4, "the stock room is four walls"
        assert "room" in source

        os.environ[WALLS_ENV] = str(tmp_path / "absent.json")
        with pytest.raises(OSError):
            load_walls()

        empty = tmp_path / "empty.json"
        empty.write_text("[]", encoding="utf-8")
        os.environ[WALLS_ENV] = str(empty)
        with pytest.raises(ValueError):
            load_walls()
    finally:
        os.environ.pop(WALLS_ENV, None)
        if previous is not None:
            os.environ[WALLS_ENV] = previous


def test_the_exported_file_is_what_the_relay_loads(tmp_path: Path) -> None:
    """End to end across the seam: this repo writes it, that repo reads it."""

    _fleet()
    from _scan_frame_relay import WALLS_ENV, load_walls

    exported = tmp_path / "walls.json"
    exported.write_text(json.dumps(wall_segments(_manifest(), NOMINAL)), encoding="utf-8")

    previous = os.environ.get(WALLS_ENV)
    os.environ[WALLS_ENV] = str(exported)
    try:
        walls, source = load_walls()
    finally:
        if previous is None:
            os.environ.pop(WALLS_ENV, None)
        else:
            os.environ[WALLS_ENV] = previous

    assert len(walls) == 24, "six buildings, four edges each"
    assert str(exported) in source
    assert walls[0][0] == tuple(wall_segments(_manifest(), NOMINAL)[0][0])
