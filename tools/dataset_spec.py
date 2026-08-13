"""The dataset's pinned numbers, in one place.

`docs/DATASET-SPEC.md` is the argument; this is the same numbers as code, so
the specification and the artifact cannot drift into two descriptions of
different datasets. Importable from the system venv (no Isaac, no pxr), which
is what lets the tests check it.

Pinned 2026-08-13, **before any frame was rendered**. A dataset whose counts and
ranges are chosen after looking at the first training curve is not evidence.
"""

from __future__ import annotations

#: The three corridor geometries, all sampled equally. None is held out: there
#: are only three, and losing one to a test split costs more than it measures.
PROFILES = ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")

#: Paired resolutions, rendered from the SAME camera on the SAME scene state.
#: `lo` is the v1 contract and the resolution ADR 0026 measured the crossing at
#: (image delivery 0.954). `hi` is that session's ceiling trial (0.926 against
#: CameraInfo 0.998). ADR 0024 decision 5 makes the choice between them a
#: measurement, and this is the measurement's input.
RESOLUTIONS: dict[str, tuple[int, int]] = {
    "lo": (640, 360),
    "hi": (1280, 720),
}

#: 1000 per profile, 3000 paired frames, 6000 images.
FRAMES_PER_PROFILE = 1000

#: Stratified WITHIN each profile, and validation is reported per profile as
#: well as pooled.
TRAIN_FRACTION = 0.8

#: A's lateral offset from the route, as a fraction of the local clear
#: half-width. The detector has to work when A is not perfectly centred, and a
#: run-shaped sample would only ever show it where the controller put it.
LATERAL_FRACTION = 0.4

#: Yaw jitter about the route tangent, degrees. Wide enough to cover the
#: heading error a live run actually carries, narrow enough that A is never
#: facing away down a corridor it is supposed to be driving along.
YAW_JITTER_DEG = 25.0

#: Dome light intensity range.
DOME_INTENSITY = (300.0, 1500.0)

#: Dome light colour temperature range, kelvin.
DOME_TEMPERATURE_K = (4000.0, 8000.0)

#: Key light yaw range, degrees.
KEY_LIGHT_YAW_DEG = (0.0, 360.0)

#: How many label overlays are inspected before bulk generation. A dataset whose
#: boxes are silently offset trains a detector to be silently wrong, and the
#: lens lesson of this repository is that a number will not show it.
ACCEPTANCE_OVERLAYS = 20
