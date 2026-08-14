"""The two bench verdicts that must never silently flip.

`tools/creep_bench.py` runs six scenarios; this locks the two that carry the
argument. The full matrix stays a tool invocation -- 13 s is too much for every
suite run -- but a disc that stops contacting, or a cone that starts, is a
regression in the one behaviour this session existed to fix, and neither should
wait for an Isaac cycle to surface.

Why a closed-loop bench rather than more unit tests: on 2026-08-13 the fleet's
governor suite was green while the robot could not touch its target. Every
docking test modelled the target as a single beam, which cannot leak outside an
angular mask. The unit tests now carry a ray-traced cylinder, but a fixture can
only be wrong in the way its author imagined -- this runs the real detector,
the real state machine and the real governor against a raycast of the authored
corridor, and asks whether the robot arrives.
"""
import json
import math
import os
import sys
from pathlib import Path

import pytest

# `abspath`, never `resolve()` (D5): resolving this file's path escapes the
# symlinked checkout into ~/Development, and the bench's fleet imports are
# walked logically from here. `abspath` normalises without following symlinks.
ROOT = Path(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
TOOLS = ROOT / "tools"
MANIFEST = ROOT / "out/corridor.manifest.json"
NOMINAL = "nominal_m6_n3"

#: The fleet `src/`, for the env-override branch of the resolver. Hardcoded as
#: `test_export_scan_walls.py` does -- a test may know the host layout; the
#: tool it exercises may not.
FLEET_SRC = Path("/home/alexmint/Development/robot-fleet/src")

#: Centre-to-centre range at which A's bumper meets B, from the authored radii.
CONTACT_RANGE_M = 0.2175
#: A contact is a contact; this is beam discretisation, not a tolerance on truth.
CONTACT_TOL_M = 0.005


def _bench():
    """Import the bench, or skip if the fleet layout is not resolvable.

    NOTE the absence of `.resolve()` on the fleet side of this: the bench walks
    the LOGICAL path out of the symlinked checkout (D5), and resolving it would
    escape into ~/Development and break the sibling imports. `ROOT` above is
    this repo's own path and is not used for that walk.
    """

    if not MANIFEST.is_file():
        pytest.skip("out/corridor.manifest.json is not built")
    # Env-override-first, so the suite passes from either the symlinked or the
    # physical checkout. Only set it when the fleet is actually there, and never
    # clobber an operator's own override.
    if not os.environ.get("CORRIDOR_FLEET_SRC"):
        if not (FLEET_SRC / "yahboomcar-ros2" / "yahboomcar_sim").is_dir():
            pytest.skip("the fleet layout is not in place")
        os.environ["CORRIDOR_FLEET_SRC"] = str(FLEET_SRC)
    if str(TOOLS) not in sys.path:
        sys.path.insert(0, str(TOOLS))
    try:
        import creep_bench
    except SystemExit:  # the bench exits on an incomplete fleet layout
        pytest.skip("the fleet layout is not in place")
    return creep_bench


@pytest.fixture(scope="module")
def manifest():
    if not MANIFEST.is_file():
        pytest.skip("out/corridor.manifest.json is not built")
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def test_the_disc_creep_reaches_contact(manifest):
    """The headline: A closes on B and touches it, under the real filter."""

    bench = _bench()
    result = bench.SCENARIOS["disc"](manifest, NOMINAL)

    assert result["contacted"], "A must physically reach B"
    assert result["state"] == "DELIVERED_CONFIRMED"
    assert result["final_true_range_m"] == pytest.approx(
        CONTACT_RANGE_M, abs=CONTACT_TOL_M
    )
    # And it must arrive inside the budget it is actually given, not eventually.
    assert result["elapsed_s"] < 25.0
    # No sustained governor stall on the way in -- that was blocker 2.
    assert result["pin"] is None, f"the creep stalled: {result['pin']}"


def test_the_cone_creep_cannot_reach_contact(manifest):
    """**The permanent negative control. This test passing means the bug is real.**

    Delete the disc and this is what the robot does: pins roughly 0.19 m short
    and sits there until the timeout, braking on B's own leaked shoulders. If
    this ever starts contacting, the bench has stopped modelling the geometry
    that grounded nine Isaac runs, and its green verdicts are worth nothing.
    """

    bench = _bench()
    result = bench.SCENARIOS["cone_leak"](manifest, NOMINAL)

    assert not result["contacted"]
    assert result["final_true_range_m"] > CONTACT_RANGE_M + 0.10
    pin = result["pin"]
    assert pin is not None and pin["held_s"] > 5.0, "the cone must pin, not crawl"
    assert "obstacle at" in pin["reason"]
    # The thing it brakes on is inside the stop distance and is B itself: the
    # leaked return is nearer than the target centre by roughly one radius.
    assert pin["leaked_min_m"] < 0.35
    assert pin["declared_range_m"] - pin["leaked_min_m"] < 0.16


def test_a_governor_stop_is_never_reported_as_a_delivery(manifest):
    """Blocker 4. A pinned robot claiming success is worse than a stuck one."""

    bench = _bench()
    result = bench.SCENARIOS["forgery"](manifest, NOMINAL)

    assert not result["contacted"]
    assert not result.get("forged"), "a governor stop was reported as a bump"
    assert result["state"] != "DELIVERED_CONFIRMED"


def test_the_target_subtends_more_than_the_cone_it_was_given():
    """The arithmetic behind all of the above, with no simulation in it.

    B's half-width is asin(R/r). The cone was fixed at 15 deg. There is no range
    inside 0.46 m at which those are compatible, and at contact the target is
    more than twice the mask.
    """

    radius, cone = 0.12, math.radians(15.0)

    assert math.asin(radius / 0.50) < cone      # outside, the cone still covers B
    assert math.asin(radius / 0.4636) == pytest.approx(cone, abs=1e-3)  # crossover
    assert math.asin(radius / 0.40) > cone      # inside, it does not
    assert math.degrees(math.asin(radius / CONTACT_RANGE_M)) == pytest.approx(
        33.5, abs=0.5
    )
