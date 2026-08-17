from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
from police_observer.synthetic import SyntheticCamera
from scene.build import build_scene
from scene.model import authored_config_path
from scene.trajectory import trajectory_from_manifest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import aruco_render_gate as gate  # noqa: E402


@pytest.fixture()
def rendered_sequence(tmp_path: Path) -> tuple[Path, Path, Path]:
    stage_path = tmp_path / "corridor.usda"
    # THE AUTHORED SCENE, explicitly. This gate is the v1 ArUco program and
    # its stations (0.5-7.0 m) are authored-scale; the default config is now
    # the 3.6 m scenario the robot drives, where x = 5.0 is past the corner.
    _, manifest_path = build_scene(authored_config_path(), stage_path, 6.0, 3.0)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = str(manifest["selected_profile"])
    camera = SyntheticCamera(manifest_path, profile)
    trajectory = trajectory_from_manifest(
        manifest["profiles"][profile]["delivery_trajectory"]
    )

    capture_dir = tmp_path / "capture"
    frame_dir = capture_dir / "frames"
    frame_dir.mkdir(parents=True)
    frames: list[dict[str, object]] = []
    dwells: list[dict[str, object]] = []
    stamp_ns = 1_000_000_000
    period_ns = round(1e9 / camera.rate_hz)
    stations_x_m = (0.5, 1.5, 3.0, 5.0, 7.0)
    for index, station_x_m in enumerate(stations_x_m):
        route_s_m = trajectory.approach_s_at_x(station_x_m)
        actor_pose = trajectory.pose_at(route_s_m)
        camera_pose = trajectory.camera_pose_at(route_s_m)
        dwell_stamps: list[int] = []
        for _ in range(gate.RENDER_GATE_CRITERIA["required_frames_per_dwell"]):
            relative = Path("frames") / f"frame_{stamp_ns:019d}.png"
            assert cv2.imwrite(str(capture_dir / relative), camera.render(station_x_m))
            calibration = camera.calibration
            frames.append(
                {
                    "stamp_ns": stamp_ns,
                    "image_path": relative.as_posix(),
                    "encoding": "rgb8",
                    "frame_id": calibration.frame_id,
                    "width": calibration.width,
                    "height": calibration.height,
                    "step": calibration.width * 3,
                    "distortion_model": "plumb_bob",
                    "d": calibration.distortion.tolist(),
                    "k": calibration.matrix.reshape(-1).tolist(),
                    "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    "p": [
                        calibration.matrix[0, 0],
                        0.0,
                        calibration.matrix[0, 2],
                        0.0,
                        0.0,
                        calibration.matrix[1, 1],
                        calibration.matrix[1, 2],
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                    ],
                }
            )
            dwell_stamps.append(stamp_ns)
            stamp_ns += period_ns
        dwells.append(
            {
                "index": index,
                "required_for_estimation": True,
                "route_s_m": route_s_m,
                "expected_station_x_m": camera_pose.x_m,
                "actor_pose": vars(actor_pose),
                "camera_pose": vars(camera_pose),
                "sim_start_s": dwell_stamps[0] / 1e9,
                "settled_start_s": dwell_stamps[0] / 1e9,
                "sim_end_s": dwell_stamps[-1] / 1e9,
            }
        )

    capture_path = capture_dir / "capture.json"
    capture_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "kind": "test_camera_capture",
                "contract": {
                    "width": camera.calibration.width,
                    "height": camera.calibration.height,
                    "frame_id": camera.calibration.frame_id,
                },
                "clock": {
                    "samples": len(frames),
                    "first_ns": frames[0]["stamp_ns"],
                    "last_ns": frames[-1]["stamp_ns"],
                },
                "frames": frames,
            }
        ),
        encoding="utf-8",
    )
    truth_path = tmp_path / "static-truth.json"
    truth_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "profile": profile,
                "dwells": dwells,
            }
        ),
        encoding="utf-8",
    )
    return capture_path, truth_path, manifest_path


def _evaluate(
    capture_path: Path,
    truth_path: Path,
    manifest_path: Path,
    transform: str = "none",
) -> dict[str, object]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    profile = str(manifest["selected_profile"])
    analysis = gate.analyse_capture(capture_path, manifest_path, profile, transform)
    return gate.compare_to_truth(analysis, truth, manifest)


def test_static_gate_recovers_world_x_and_corner_order(rendered_sequence) -> None:
    report = _evaluate(*rendered_sequence)
    assert report["gate_passed"]
    assert report["camera_contract"]["passed"]
    assert all(dwell["passed"] for dwell in report["dwells"])
    errors = [
        frame["station_error_m"]
        for dwell in report["dwells"]
        for frame in dwell["frames"]
    ]
    assert max(errors) < gate.RENDER_GATE_CRITERIA["maximum_station_error_m"]


@pytest.mark.parametrize("transform", ("blank", "mirror"))
def test_static_gate_rejects_pixel_negative_controls(rendered_sequence, transform) -> None:
    report = _evaluate(*rendered_sequence, transform=transform)
    assert not report["gate_passed"]


def test_static_gate_rejects_wrong_camera_info(rendered_sequence) -> None:
    capture_path, truth_path, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    for frame in capture["frames"]:
        frame["k"][0] *= 1.2
        frame["k"][4] *= 1.2
    changed = capture_path.parent / "wrong-k.json"
    changed.write_text(json.dumps(capture), encoding="utf-8")
    report = _evaluate(changed, truth_path, manifest_path)
    assert not report["gate_passed"]
    assert not report["camera_contract"]["passed"]


def test_static_gate_rejects_a_non_production_encoding(rendered_sequence) -> None:
    """A uniformly wrong encoding must fail even though it is constant."""

    capture_path, truth_path, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    for frame in capture["frames"]:
        frame["encoding"] = "bgr8"
    changed = capture_path.parent / "bgr8.json"
    changed.write_text(json.dumps(capture), encoding="utf-8")
    report = _evaluate(changed, truth_path, manifest_path)
    assert not report["gate_passed"]
    assert not report["camera_contract"]["passed"]
    assert report["camera_contract"]["encoding_constant"]
    assert not report["camera_contract"]["encoding_expected"]


def test_static_gate_rejects_encoding_that_changes_mid_sequence(rendered_sequence) -> None:
    capture_path, truth_path, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["frames"][0]["encoding"] = "mono8"
    changed = capture_path.parent / "mixed-encoding.json"
    changed.write_text(json.dumps(capture), encoding="utf-8")
    report = _evaluate(changed, truth_path, manifest_path)
    assert not report["gate_passed"]
    assert not report["camera_contract"]["encoding_constant"]


def test_bad_encoding_fails_through_the_public_cli(rendered_sequence, monkeypatch) -> None:
    """Exercise the entry point operators actually run, not just a helper."""

    capture_path, truth_path, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    for frame in capture["frames"]:
        frame["encoding"] = "bgr8"
    changed = capture_path.parent / "cli-bgr8.json"
    changed.write_text(json.dumps(capture), encoding="utf-8")
    report_path = capture_path.parent / "cli-report.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "aruco_render_gate.py",
            "--capture",
            str(changed),
            "--truth",
            str(truth_path),
            "--manifest",
            str(manifest_path),
            "--out",
            str(report_path),
        ],
    )
    assert gate.main() == 1
    written = json.loads(report_path.read_text(encoding="utf-8"))
    assert not written["gate_passed"]


def test_static_gate_rejects_the_half_pixel_principal_point_convention(
    rendered_sequence,
) -> None:
    """The 0.5 px convention drift must fail the named intrinsic criterion.

    The previous 0.5 px tolerance accepted it exactly, so this class of drift
    could never be detected.
    """

    assert gate.RENDER_GATE_CRITERIA["maximum_intrinsic_error_px"] < 0.5
    capture_path, truth_path, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    for frame in capture["frames"]:
        frame["k"][2] -= 0.5
        frame["k"][5] -= 0.5
    changed = capture_path.parent / "half-pixel.json"
    changed.write_text(json.dumps(capture), encoding="utf-8")
    report = _evaluate(changed, truth_path, manifest_path)
    assert not report["camera_contract"]["passed"]
    assert report["camera_contract"]["maximum_matrix_error_px"] == pytest.approx(0.5)


def test_static_gate_rejects_route_distance_as_world_x_truth(rendered_sequence) -> None:
    capture_path, truth_path, manifest_path = rendered_sequence
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    wrong = copy.deepcopy(truth)
    for dwell in wrong["dwells"]:
        dwell["expected_station_x_m"] = dwell["route_s_m"]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    profile = str(manifest["selected_profile"])
    analysis = gate.analyse_capture(capture_path, manifest_path, profile)
    report = gate.compare_to_truth(analysis, wrong, manifest)
    assert not report["gate_passed"]


def test_static_gate_reports_an_unsurveyed_detected_id(rendered_sequence) -> None:
    capture_path, _, manifest_path = rendered_sequence
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    first = capture_path.parent / capture["frames"][0]["image_path"]
    image = np.full((360, 640), 220, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    marker = np.zeros((120, 120), dtype=np.uint8)
    if hasattr(cv2.aruco, "generateImageMarker"):
        cv2.aruco.generateImageMarker(dictionary, 99, 120, marker, 1)
    else:
        cv2.aruco.drawMarker(dictionary, 99, 120, marker, 1)
    image[120:240, 260:380] = marker
    assert cv2.imwrite(str(first), image)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    analysis = gate.analyse_capture(
        capture_path,
        manifest_path,
        str(manifest["selected_profile"]),
    )
    assert analysis["phantom_ids"] == [99]
