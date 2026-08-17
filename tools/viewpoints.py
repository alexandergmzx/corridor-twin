"""Viewport perspectives for the Isaac demonstration, without importing Isaac.

Placing the GUI camera needs a running Kit application. Deciding *where* to put
it does not, so the eye/target arithmetic lives here and is exercised on an
ordinary interpreter, including the quadrants a single live run never visits.

Nothing in this module touches the sensor. Isaac's ``set_camera_view`` moves
Kit's pre-existing ``/OmniverseKit_Persp``, which the GUI already owns; it
creates no prim and no render product. The one-camera budget is counted over
``IsaacCreateRenderProduct`` graph nodes, which a viewpoint never adds to, and
the ROS camera stays ``/World/Actors/PCameraMast/PCam`` -- P's mast.

The world is Z-up with metres as its unit, station runs along +X, and the
corridor's fixed north face is at +Y, so a viewpoint at negative Y looks at the
tapering face from outside the scene.
"""

from __future__ import annotations

import math

Vec3 = tuple[float, float, float]

CHASE_VIEW = "chase"

# Static perspectives, as (eye, target) in world metres.
#
# `rviz` is the RViz view expressed in world coordinates rather than eyeballed,
# so the two windows show the same angle during a screenshare. Its RViz form is
# an Orbit controller in `src/police_observer/rviz/corridor_twin.rviz` at
# distance 30, focal (8, -1, 0), pitch 1.1 rad, yaw 4.2 rad; an orbit camera
# sits at `focal + distance * (cos yaw cos pitch, sin yaw cos pitch, sin pitch)`,
# which is the eye below. Pitch 1.1 rad is 63 degrees above horizontal.
#
# `corner` frames the junction instead of the whole scene: the throat, the turn,
# and P behind the east wall are all in shot at once, which is where the
# violation fires and where the occlusion claim is visible rather than asserted.
VIEWPOINTS: dict[str, tuple[Vec3, Vec3]] = {
    "rviz": ((1.33, -12.86, 26.74), (8.0, -1.0, 0.0)),
    "corner": ((2.0, -16.0, 14.0), (13.0, -6.0, 1.0)),
}

VIEW_NAMES: tuple[str, ...] = (*sorted(VIEWPOINTS), CHASE_VIEW)

# Chase geometry, in metres. Far enough back that A stays small against the
# corridor rather than filling the frame, and high enough to see over the walls
# it is driving between.
CHASE_BACK_M = 6.0
CHASE_HEIGHT_M = 4.0
CHASE_AHEAD_M = 3.0
CHASE_TARGET_HEIGHT_M = 0.5


def parse_vec3(text: str) -> Vec3:
    """Parse ``"X,Y,Z"`` into a finite 3-tuple of metres.

    Non-finite values are rejected rather than passed through. ``nan`` compares
    false against every bound, so an unchecked one propagates into the camera
    transform and produces a black viewport with no error -- the same shape of
    hole `f2e2504` closed in the speed policy, where `nan <= 0` is also false.
    """

    parts = [piece.strip() for piece in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError(f"expected three comma-separated numbers, got {text!r}")
    values = []
    for part in parts:
        try:
            value = float(part)
        except ValueError as error:
            raise ValueError(f"{part!r} in {text!r} is not a number") from error
        if not math.isfinite(value):
            raise ValueError(f"{part!r} in {text!r} is not finite")
        values.append(value)
    return (values[0], values[1], values[2])


def resolve(
    name: str,
    eye: Vec3 | None = None,
    target: Vec3 | None = None,
) -> tuple[Vec3, Vec3] | None:
    """Return the ``(eye, target)`` for a perspective, or ``None`` for chase.

    An explicit eye and target override the named preset, which is what makes
    trying a new angle a re-run rather than an edit. ``chase`` has no fixed eye
    because it is recomputed from A's pose, so it returns ``None`` and the
    caller drives it from the route instead.
    """

    if eye is not None and target is not None:
        if eye == target:
            raise ValueError("--view-eye and --view-target must differ")
        return (eye, target)
    if (eye is None) != (target is None):
        raise ValueError("--view-eye and --view-target must be given together")
    if name == CHASE_VIEW:
        return None
    if name not in VIEWPOINTS:
        valid = ", ".join(VIEW_NAMES)
        raise ValueError(f"unknown view {name!r}; expected one of {valid}")
    return VIEWPOINTS[name]


def chase_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    back_m: float = CHASE_BACK_M,
    height_m: float = CHASE_HEIGHT_M,
    ahead_m: float = CHASE_AHEAD_M,
) -> tuple[Vec3, Vec3]:
    """Place the viewport behind and above A, looking along its heading.

    Yaw is the same convention the authored trajectory reports: measured in the
    XY plane from +X toward +Y, so the heading is ``(cos yaw, sin yaw)`` and
    subtracting it walks backwards along the route.
    """

    if back_m <= 0.0 or height_m <= 0.0 or ahead_m <= 0.0:
        raise ValueError("chase distances must be positive")
    heading_x = math.cos(yaw_rad)
    heading_y = math.sin(yaw_rad)
    eye = (x_m - back_m * heading_x, y_m - back_m * heading_y, height_m)
    target = (
        x_m + ahead_m * heading_x,
        y_m + ahead_m * heading_y,
        CHASE_TARGET_HEIGHT_M,
    )
    return (eye, target)


def format_vec3(vector: Vec3) -> str:
    """Render a vector for the run's evidence marker."""

    return ",".join(f"{component:g}" for component in vector)
