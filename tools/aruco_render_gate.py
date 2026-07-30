#!/usr/bin/env python3
"""Gate production-rendered ArUco frames against survey and separate pose evidence."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from police_observer.estimator import ArucoStationEstimator, Calibration, MarkerMap

RENDER_GATE_CRITERIA = {
    "required_frames_per_dwell": 3,
    "minimum_passing_frames_per_dwell": 2,
    "maximum_station_error_m": 0.05,
    "maximum_reprojection_rmse_px": 3.0,
    "maximum_corner_rmse_px": 3.0,
    "minimum_rate_hz": 14.5,
    "maximum_rate_hz": 15.5,
    "expected_width": 640,
    "expected_height": 360,
    "expected_frame_id": "robot_front_camera_optical_frame",
    "expected_encoding": "rgb8",
    # Production delivers cx=width/2 to within 1.5e-05 px. A tolerance of 0.5
    # would silently accept the (width-1)/2 convention, which differs by
    # exactly 0.5 px, so it could never catch that class of drift.
    "maximum_intrinsic_error_px": 0.05,
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--profile")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("out/evidence/static-fiducials/aruco-gate.json"),
    )
    parser.add_argument("--negative-control", choices=("none", "mirror", "blank"), default="none")
    parser.add_argument("--expect-fail", action="store_true")
    return parser.parse_args()


def _calibration(frame: dict[str, Any]) -> Calibration:
    return Calibration(
        width=int(frame["width"]),
        height=int(frame["height"]),
        matrix=np.asarray(frame["k"], dtype=np.float64).reshape(3, 3),
        distortion=np.asarray(frame["d"], dtype=np.float64),
        frame_id=str(frame["frame_id"]),
        distortion_model=str(frame["distortion_model"]),
    )


def _transform_image(image: np.ndarray, transform: str) -> np.ndarray:
    if transform == "none":
        return image
    if transform == "mirror":
        return cv2.flip(image, 1)
    if transform == "blank":
        return np.full_like(image, 127)
    raise ValueError(f"unknown image transform {transform!r}")


def analyse_capture(
    capture_path: Path,
    manifest_path: Path,
    profile_name: str,
    image_transform: str = "none",
) -> dict[str, Any]:
    """Recover observations from pixels and survey data, without commanded pose."""

    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if profile_name not in manifest["profiles"]:
        raise ValueError(f"manifest has no profile {profile_name!r}")
    marker_map = MarkerMap.from_manifest(manifest_path, profile_name)
    dictionary_name = str(manifest["fiducials"]["dictionary"])
    estimator = ArucoStationEstimator(
        marker_map,
        dictionary_name,
        maximum_reprojection_rmse_px=RENDER_GATE_CRITERIA[
            "maximum_reprojection_rmse_px"
        ],
    )
    surveyed_ids = set(marker_map.marker_corners)
    frames: list[dict[str, Any]] = []
    phantom_ids: set[int] = set()
    for frame in capture["frames"]:
        image_path = capture_path.parent / str(frame["image_path"])
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"failed to read captured frame {image_path}")
        image = _transform_image(image, image_transform)
        calibration = _calibration(frame)
        corners, identifiers = estimator.detect(image)
        detections: list[dict[str, Any]] = []
        if identifiers is not None:
            for detected_corners, identifier in zip(corners, identifiers, strict=True):
                marker_id = int(identifier[0])
                pixels = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
                detections.append({"id": marker_id, "corners_px": pixels.tolist()})
                if marker_id not in surveyed_ids:
                    phantom_ids.add(marker_id)
        stamp_ns = int(frame["stamp_ns"])
        observation = estimator.estimate(image, calibration, stamp_ns / 1e9)
        frames.append(
            {
                "stamp_ns": stamp_ns,
                "image_path": str(frame["image_path"]),
                "encoding": str(frame["encoding"]),
                "calibration": {
                    "width": calibration.width,
                    "height": calibration.height,
                    "frame_id": calibration.frame_id,
                    "distortion_model": str(frame["distortion_model"]),
                    "d": calibration.distortion.tolist(),
                    "k": calibration.matrix.reshape(-1).tolist(),
                },
                "detections": detections,
                "observation": (
                    None
                    if observation is None
                    else {
                        "timestamp_s": observation.timestamp_s,
                        "station_m": observation.station_m,
                        "station_stddev_m": observation.station_stddev_m,
                        "reprojection_rmse_px": observation.reprojection_rmse_px,
                        "marker_ids": list(observation.marker_ids),
                    }
                ),
            }
        )
    return {
        "schema_version": "1.0",
        "profile": profile_name,
        "capture_contract": capture["contract"],
        "clock": capture["clock"],
        "frames": frames,
        "phantom_ids": sorted(phantom_ids),
        "image_transform": image_transform,
    }


def _project_world_corners(
    world_corners: np.ndarray,
    camera_pose: dict[str, Any],
    calibration: dict[str, Any],
) -> np.ndarray:
    yaw = float(camera_pose["yaw_rad"])
    origin = np.asarray(
        [camera_pose["x_m"], camera_pose["y_m"], camera_pose["z_m"]],
        dtype=np.float64,
    )
    relative = np.asarray(world_corners, dtype=np.float64) - origin
    forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.asarray([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.asarray([0.0, 0.0, -1.0])
    camera_points = np.column_stack(
        (relative @ right, relative @ down, relative @ forward)
    )
    if np.any(camera_points[:, 2] <= 0.0):
        raise ValueError("detected surveyed marker projects behind the commanded camera")
    matrix = np.asarray(calibration["k"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(calibration["d"], dtype=np.float64)
    projected, _ = cv2.projectPoints(
        camera_points,
        np.zeros(3),
        np.zeros(3),
        matrix,
        distortion,
    )
    return projected.reshape(4, 2)


def _camera_contract_result(
    frames: list[dict[str, Any]], manifest: dict[str, Any], clock: dict[str, Any]
) -> dict[str, Any]:
    criteria = RENDER_GATE_CRITERIA
    if len(frames) < 2:
        return {"passed": False, "reason": "fewer than two camera frames"}
    stamps = sorted(int(frame["stamp_ns"]) for frame in frames)
    intervals = np.diff(np.asarray(stamps, dtype=np.float64)) / 1e9
    rate_hz = float(1.0 / np.median(intervals))
    first = frames[0]["calibration"]
    constant = all(
        frame["calibration"] == first and int(frame["stamp_ns"]) > 0 for frame in frames
    )
    width = int(first["width"])
    height = int(first["height"])
    matrix = np.asarray(first["k"], dtype=np.float64).reshape(3, 3)
    expected_focal = (width / 2.0) / math.tan(
        math.radians(float(manifest["camera"]["horizontal_fov_deg"])) / 2.0
    )
    expected_matrix = np.asarray(
        [[expected_focal, 0.0, width / 2.0], [0.0, expected_focal, height / 2.0], [0, 0, 1]],
        dtype=np.float64,
    )
    matrix_error_px = float(np.max(np.abs(matrix - expected_matrix)))
    distortion = np.asarray(first["d"], dtype=np.float64)
    last_clock_ns = clock.get("last_ns")
    # Encoding is checked explicitly rather than folded into the calibration
    # equality test, so a wrong-but-constant encoding cannot hide behind it.
    encodings = {str(frame["encoding"]) for frame in frames}
    encoding_constant = len(encodings) == 1
    encoding_expected = encodings == {criteria["expected_encoding"]}
    passed = all(
        (
            constant,
            encoding_constant,
            encoding_expected,
            width == criteria["expected_width"],
            height == criteria["expected_height"],
            first["frame_id"] == criteria["expected_frame_id"],
            first["distortion_model"] == "plumb_bob",
            not np.any(np.abs(distortion) > 1e-9),
            matrix_error_px <= criteria["maximum_intrinsic_error_px"],
            criteria["minimum_rate_hz"] <= rate_hz <= criteria["maximum_rate_hz"],
            last_clock_ns is not None and int(last_clock_ns) >= stamps[-1],
        )
    )
    return {
        "passed": passed,
        "constant": constant,
        "encodings": sorted(encodings),
        "encoding_constant": encoding_constant,
        "encoding_expected": encoding_expected,
        "rate_hz": rate_hz,
        "matrix": matrix.reshape(-1).tolist(),
        "expected_matrix": expected_matrix.reshape(-1).tolist(),
        "maximum_matrix_error_px": matrix_error_px,
        "distortion": distortion.tolist(),
        "frame_id": first["frame_id"],
        "resolution": [width, height],
    }


def compare_to_truth(
    analysis: dict[str, Any], truth: dict[str, Any], manifest: dict[str, Any]
) -> dict[str, Any]:
    """Compare pixel-only observations with evaluator-only commanded camera poses."""

    profile = str(analysis["profile"])
    if truth["profile"] != profile:
        raise ValueError("capture analysis and truth schedule profiles differ")
    marker_corners = {
        int(marker["id"]): np.asarray(marker["aruco_corner_order_xyz_m"], dtype=np.float64)
        for marker in manifest["profiles"][profile]["markers"]
    }
    frame_by_stamp = sorted(analysis["frames"], key=lambda frame: int(frame["stamp_ns"]))
    dwell_results: list[dict[str, Any]] = []
    all_required_frames_valid = True
    for dwell in truth["dwells"]:
        if not dwell["required_for_estimation"]:
            continue
        start_ns = round(float(dwell["settled_start_s"]) * 1e9)
        end_ns = round(float(dwell["sim_end_s"]) * 1e9)
        matched = [
            frame for frame in frame_by_stamp if start_ns <= int(frame["stamp_ns"]) <= end_ns
        ]
        selected = matched[: RENDER_GATE_CRITERIA["required_frames_per_dwell"]]
        frame_results: list[dict[str, Any]] = []
        for frame in selected:
            observation = frame["observation"]
            station_error = (
                None
                if observation is None
                else abs(
                    float(observation["station_m"])
                    - float(dwell["expected_station_x_m"])
                )
            )
            corner_errors: list[float] = []
            for detection in frame["detections"]:
                marker_id = int(detection["id"])
                world = marker_corners.get(marker_id)
                if world is None:
                    continue
                projected = _project_world_corners(
                    world,
                    dwell["camera_pose"],
                    frame["calibration"],
                )
                detected = np.asarray(detection["corners_px"], dtype=np.float64)
                residual = projected - detected
                corner_errors.append(
                    float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
                )
            maximum_corner_error = max(corner_errors, default=None)
            passed = bool(
                observation is not None
                and len(observation["marker_ids"]) >= 2
                and station_error is not None
                and station_error <= RENDER_GATE_CRITERIA["maximum_station_error_m"]
                and float(observation["reprojection_rmse_px"])
                <= RENDER_GATE_CRITERIA["maximum_reprojection_rmse_px"]
                and maximum_corner_error is not None
                and maximum_corner_error <= RENDER_GATE_CRITERIA["maximum_corner_rmse_px"]
            )
            frame_results.append(
                {
                    "stamp_ns": frame["stamp_ns"],
                    "observation": observation,
                    "station_error_m": station_error,
                    "maximum_corner_rmse_px": maximum_corner_error,
                    "passed": passed,
                }
            )
        passing = sum(bool(frame["passed"]) for frame in frame_results)
        usable_with_bad_error = any(
            frame["observation"] is not None and not frame["passed"] for frame in frame_results
        )
        dwell_passed = bool(
            len(selected) == RENDER_GATE_CRITERIA["required_frames_per_dwell"]
            and passing >= RENDER_GATE_CRITERIA["minimum_passing_frames_per_dwell"]
            and not usable_with_bad_error
        )
        all_required_frames_valid = all_required_frames_valid and dwell_passed
        dwell_results.append(
            {
                "index": dwell["index"],
                "route_s_m": dwell["route_s_m"],
                "expected_station_x_m": dwell["expected_station_x_m"],
                "matched_frames": len(matched),
                "selected_frames": len(selected),
                "passing_frames": passing,
                "passed": dwell_passed,
                "frames": frame_results,
            }
        )

    camera_contract = _camera_contract_result(analysis["frames"], manifest, analysis["clock"])
    gate_passed = bool(
        dwell_results
        and all_required_frames_valid
        and not analysis["phantom_ids"]
        and camera_contract["passed"]
    )
    return {
        "schema_version": "1.0",
        "gate_passed": gate_passed,
        "profile": profile,
        "criteria": RENDER_GATE_CRITERIA,
        "image_transform": analysis["image_transform"],
        "phantom_ids": analysis["phantom_ids"],
        "camera_contract": camera_contract,
        "dwells": dwell_results,
    }


def main() -> int:
    args = arguments()
    capture_path = args.capture.resolve()
    truth_path = args.truth.resolve()
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    profile = args.profile or str(manifest["selected_profile"])
    analysis = analyse_capture(
        capture_path,
        manifest_path,
        profile,
        image_transform=args.negative_control,
    )
    report = compare_to_truth(analysis, truth, manifest)
    report["capture"] = str(capture_path)
    report["truth"] = str(truth_path)
    report["manifest"] = str(manifest_path)
    report["expect_fail"] = bool(args.expect_fail)
    report["expectation_met"] = bool(report["gate_passed"] != args.expect_fail)
    output_path = args.out.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if report["expectation_met"]:
        marker = (
            "ARUCO_RENDER_NEGATIVE_CONTROL_PASS"
            if args.expect_fail
            else "ARUCO_RENDER_GATE_PASS"
        )
        print(marker, f"profile={profile}", f"report={output_path}", flush=True)
        return 0
    marker = "ARUCO_RENDER_GATE_UNEXPECTED_PASS" if args.expect_fail else "ARUCO_RENDER_GATE_FAIL"
    print(marker, f"profile={profile}", f"report={output_path}", file=sys.stderr, flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
