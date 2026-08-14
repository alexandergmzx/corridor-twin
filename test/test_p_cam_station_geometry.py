"""Pixels to a station on the ground, and the two ways it fails silently.

The projection in `p_cam_infer` is the whole of P's positioning: no depth, no
pose, no TF. It is also the part that cannot announce its own mistakes -- a
wrong camera pose, a wrong convention, or a flipped axis all produce a
plausible metre for every frame and fail nothing.

So it is tested by round trip. Put a known point on the ground, project it into
the camera with the same intrinsics, hand the pixel back, and require the
original point. A test that only checked "a number came out" would have passed
every one of the mistakes below.

`torch` is imported lazily inside `load_detector`, so this runs in the system
venv without Isaac's interpreter.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from p_cam_infer import GROUND_Z_M, camera_pose, ground_intersection, station_from_box  # noqa: E402

#: P's mast as the composed arena carries it, and the 640x360 contract.
CAMERA_XYZ = np.array([5.235, 0.72, 1.5])
K = np.array([[417.0, 0.0, 320.0], [0.0, 417.0, 180.0], [0.0, 0.0, 1.0]])


def _rotation(look_at) -> np.ndarray:
    """world_from_optical for a camera at CAMERA_XYZ aimed at `look_at`."""

    forward = np.asarray(look_at, dtype=float) - CAMERA_XYZ
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack([right, down, forward])


def _project(point, rotation) -> tuple[float, float]:
    """World point -> pixel. The inverse of what the estimator does."""

    camera = rotation.T @ (np.asarray(point, dtype=float) - CAMERA_XYZ)
    assert camera[2] > 0, "the test point is behind the camera"
    homogeneous = K @ camera
    return float(homogeneous[0] / homogeneous[2]), float(homogeneous[1] / homogeneous[2])


@pytest.mark.parametrize("point", [
    (0.6, 0.0, 0.0), (1.8, 0.2, 0.0), (3.0, -0.15, 0.0), (2.4, 0.0, 0.0),
])
def test_a_ground_point_survives_the_round_trip(point) -> None:
    """**The core claim.** Millimetres, or the geometry is not what recovers
    the station and every number downstream is coincidence."""

    rotation = _rotation((1.8, 0.0, 0.0))
    pixel = _project(point, rotation)

    hit = ground_intersection(pixel, K, CAMERA_XYZ, rotation)

    assert hit is not None
    assert hit[0] == pytest.approx(point[0], abs=1e-6)
    assert hit[1] == pytest.approx(point[1], abs=1e-6)


def test_a_ray_above_the_horizon_is_refused_rather_than_invented() -> None:
    """A box whose bottom edge lands on the sky is a detection of something not
    standing on this floor. Returning a station for it would put a fabricated
    point into the fit, and it would be a large one -- the intersection races
    off toward infinity as the ray approaches horizontal."""

    rotation = _rotation((1.8, 0.0, 0.0))
    # Far above the principal point: upward in the image is upward in the world.
    assert ground_intersection((320.0, -4000.0), K, CAMERA_XYZ, rotation) is None


def test_the_station_is_taken_from_the_bottom_edge_not_the_middle() -> None:
    """The convention is load-bearing and invisible. A box centre sits at an
    unknown height above the floor, so back-projecting it to z=0 lands beyond
    the robot -- further from the camera, by more at longer range. It is the
    exact shape of the bias measured on this pipeline, so a silent switch here
    would be indistinguishable from a detector problem."""

    rotation = _rotation((1.8, 0.0, 0.0))
    contact = (2.0, 0.0, GROUND_Z_M)
    top = (2.0, 0.0, 0.20)
    u_bottom, v_bottom = _project(contact, rotation)
    _u_top, v_top = _project(top, rotation)
    assert v_top < v_bottom, "the body's top is not above its base in the image"

    box = {"x_min": u_bottom - 12, "x_max": u_bottom + 12,
           "y_min": v_top, "y_max": v_bottom}

    assert station_from_box(box, K, CAMERA_XYZ, rotation) == pytest.approx(2.0, abs=1e-6)


def test_a_pose_file_round_trips_through_the_exporter(tmp_path) -> None:
    """**The shell/ABI seam.** `pxr` writes the pose in the venv and `torch`
    reads it in Isaac's interpreter, so no single process ever checks both
    ends. The exporter's own output is fed to the consumer's own reader."""

    stage = ROOT / "out/arena_corridor_robot1_nominal_m6_n3.usd"
    if not stage.is_file():
        pytest.skip("the composed arena is not built here")

    target = tmp_path / "pose.json"
    subprocess.run(
        [str(ROOT / ".venv/bin/python"), str(ROOT / "tools/export_camera_pose.py"),
         "--stage", str(stage), "--out", str(target)],
        check=True, capture_output=True, text=True)

    position, rotation = camera_pose(target)

    assert position[2] > 0.0, "the camera is at or below the ground plane"
    # Orthonormal, or it is not a rotation and every ray is skewed.
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)
    # +Z forward in the optical convention, and pointed at the floor.
    assert rotation[2, 2] < 0.0, "the camera looks up; every ground hit is refused"


def test_the_table_refuses_a_pose_from_a_different_stage(tmp_path) -> None:
    """**The mistake this repository actually made.** Both the v1 stage and the
    composed arena carry a mast at the same prim path, 2.1 m apart. The wrong
    one produced a full five-gate table, a confirmed violation at the same
    gate, and a *better-looking* error than the correct one. Nothing failed."""

    stations = tmp_path / "stations.json"
    stations.write_text(json.dumps({
        "camera_pose_stage": "/x/out/corridor.usda",
        "frames": [{"index": 0, "stamp_s": 0.0, "station_m": 1.0}]}),
        encoding="utf-8")
    schedule = tmp_path / "schedule.json"
    schedule.write_text(json.dumps({
        "stage": "/x/out/arena_corridor_robot1_nominal_m6_n3.usd",
        "samples": [{"sim_time_s": 0.0, "x_m": 0.0, "route_s_m": 0.0},
                    {"sim_time_s": 1.0, "x_m": 0.2, "route_s_m": 0.2}]}),
        encoding="utf-8")

    done = subprocess.run(
        [str(ROOT / ".venv/bin/python"), str(ROOT / "tools/p_cam_speed_table.py"),
         "--stations", str(stations), "--schedule", str(schedule),
         "--out", str(tmp_path / "table.json")],
        capture_output=True, text=True)

    assert done.returncode != 0, "a cross-stage pose was accepted"
    assert "camera pose came from" in (done.stderr + done.stdout)
