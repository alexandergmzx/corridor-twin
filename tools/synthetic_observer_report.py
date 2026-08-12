"""Measure observer accuracy on a declared schedule and emit one artifact.

Every accuracy figure in the design notes has to come from here. The reason is
specific: the figures published before this existed were maxima taken over a
0.4 m station grid, a schedule the system never runs. The camera publishes at
15 Hz, so at 1.0 m/s it samples roughly 35 times more stations than that grid,
and a denser sample finds worse frames -- 0.0391 m became 0.0705 m on the
nominal profile once the real cadence was used. Hand-measuring on a convenient
grid and publishing the result is the failure this tool exists to prevent, so
the schedule is declared in the output rather than left implicit in whoever
ran it.

Two quantities are reported and they are not interchangeable.

The primary metric is **gate-derived speed error**: the error in the speed
between two surveyed gates, which is what the police observer actually
delivers and what the speed policy is written about. Gate crossing times are
interpolated between observations, so per-frame station noise partly averages
out and this number is smaller than the raw station error.

The secondary metric is **per-frame station error**, kept as a health check.
It is descriptive: it says how noisy the pose stream is, and it moves for
reasons that never reach a speed measurement. Only speed error gates.

Each sample also records which markers were accepted, the rank of their
correspondence set, and whether every accepted marker was genuinely unoccluded
in the composed stage -- SyntheticCamera projects but never raycasts, so a
plate hidden behind a building still lands in its images. Reporting those from
the same run means the coverage claims and the accuracy claims cannot drift
apart.
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ("src/police_observer", "src/corridor_scene"):
    candidate = str(ROOT / package)
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

import cv2  # noqa: E402
from police_observer.estimator import (  # noqa: E402
    ArucoStationEstimator,
    MarkerMap,
    ObserverPipeline,
)
from police_observer.synthetic import SyntheticCamera  # noqa: E402
from scene.build import build_scene  # noqa: E402
from scene.model import authored_config_path, load_scenario  # noqa: E402
from scene.occlusion import _mesh_triangles, _segment_hits_triangle, opaque_mesh_prims  # noqa: E402
from scene.trajectory import delivery_trajectory  # noqa: E402

SCHEMA_VERSION = "1.0.0"
DEFAULT_OUT = Path("out/evidence/synthetic-observer/summary.json")

# The declared schedule. These are the speeds the enforcement tests drive --
# compliant, corner-only offense, and sustained offense -- so the accuracy
# figures describe the same runs the behaviour is asserted on rather than a
# separate set chosen to look good. The window ends past the last enforcement
# gate at 10.0 m by enough to bracket its crossing; beyond it only the coplanar
# east-face pair remains in view and the estimator correctly returns nothing.
SCHEDULE_SPEEDS_MPS = (0.6, 1.0, 1.8)
SCHEDULE_WINDOW_X_M = (0.0, 10.8)
PROFILE_NAMES = ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")


@dataclass(frozen=True)
class Schedule:
    """The sampling schedule, stated rather than left to the caller's habits."""

    rate_hz: float
    speeds_mps: tuple[float, ...]
    window_x_m: tuple[float, float]

    def as_dict(self) -> dict[str, Any]:
        return {
            "camera_rate_hz": self.rate_hz,
            "path_speeds_mps": list(self.speeds_mps),
            "window_x_m": list(self.window_x_m),
            "sampling": (
                "one sample per camera period at the published rate, at the station the "
                "robot has reached travelling at the stated path speed -- not a station grid"
            ),
        }


def _rank(corners: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(corners - corners.mean(axis=0), tol=1e-6))


def _run_one(
    manifest_path: Path,
    stage_path: Path,
    profile_name: str,
    truth_speed_mps: float,
    schedule: Schedule,
    check_visibility: bool,
) -> dict[str, Any]:
    """Drive the full pixel-to-violation stack once at a known true speed."""

    from pxr import Usd

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    marker_map = MarkerMap.from_manifest(manifest_path, profile_name)
    camera = SyntheticCamera(manifest_path, profile_name)
    pose = ArucoStationEstimator(marker_map, camera.dictionary_name)
    pipeline = ObserverPipeline(marker_map)

    surveyed = {
        marker["id"]: np.asarray(marker["corners_xyz_m"], dtype=np.float64)
        for marker in manifest["profiles"][profile_name]["markers"]
    }
    # The trajectory is needed unconditionally, not only for the raycast audit:
    # the schedule below advances along route arc length and reads world X back
    # off the authored path. Commanding `x = v*t*path_axis_fraction` instead
    # would multiply by the same field the estimator divides by, so an error in
    # it cancelled exactly and this report -- the primary accuracy instrument --
    # was blind to the one conversion the tapered corridor made necessary.
    # The v1 camera program's own scene. Its schedule windows are stated in
    # authored metres, so it must not follow the default to the robot-scale
    # scenario -- that would silently sample a corridor a third the length.
    scenario = load_scenario(authored_config_path())
    trajectory = delivery_trajectory(scenario, scenario.profile(profile_name))
    triangles: list[Any] = []
    if check_visibility:
        stage = Usd.Stage.Open(str(stage_path))
        # The audit has to see the meshes this profile actually composes, so
        # select its variant rather than whatever the build left selected.
        variants = stage.GetPrimAtPath("/World").GetVariantSet("corridorProfile")
        if not variants.SetVariantSelection(profile_name):
            raise ValueError(f"stage has no corridorProfile variant {profile_name!r}")
        triangles = [t for prim in opaque_mesh_prims(stage) for t in _mesh_triangles(prim)]

    start_x, end_x = schedule.window_x_m
    start_s_m = trajectory.approach_s_at_x(start_x)
    samples: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    frames = 0
    frame = 0
    while True:
        elapsed = frame / schedule.rate_hz
        # Distance actually travelled along the route, which is what the speed
        # policy is written about; world X follows from the authored path.
        route_s_m = start_s_m + truth_speed_mps * elapsed
        commanded_x_m = trajectory.pose_at(route_s_m).x_m
        if commanded_x_m > end_x:
            break
        frames += 1
        observation = pose.estimate(
            camera.render(commanded_x_m), camera.calibration, timestamp_s=1.0 + elapsed
        )
        frame += 1
        if observation is None:
            continue

        accepted = list(observation.marker_ids)
        combined = np.concatenate([surveyed[marker_id] for marker_id in accepted])
        unoccluded: bool | None = None
        if check_visibility:
            camera_pose = trajectory.camera_pose_at(route_s_m)
            origin = (camera_pose.x_m, camera_pose.y_m, camera_pose.z_m)
            unoccluded = True
            for marker_id in accepted:
                corners = surveyed[marker_id]
                for target in [corners.mean(axis=0), *corners]:
                    if any(
                        _segment_hits_triangle(origin, tuple(target), triangle) is not None
                        for triangle in triangles
                    ):
                        unoccluded = False
                        break
                if not unoccluded:
                    break

        samples.append(
            {
                "commanded_route_s_m": round(route_s_m, 6),
                "commanded_x_m": round(commanded_x_m, 6),
                "estimated_x_m": round(observation.station_m, 6),
                "station_error_m": round(abs(observation.station_m - commanded_x_m), 6),
                "reprojection_rmse_px": round(observation.reprojection_rmse_px, 4),
                "accepted_marker_ids": accepted,
                "correspondence_rank": _rank(combined),
                "all_accepted_unoccluded": unoccluded,
            }
        )
        for measurement, violation in pipeline.update(observation):
            measurements.append(
                {
                    "station_m": measurement.station_m,
                    "gate_from_id": measurement.gate_from_id,
                    "gate_to_id": measurement.gate_to_id,
                    "speed_mps": round(measurement.speed_mps, 6),
                    "speed_error_mps": round(abs(measurement.speed_mps - truth_speed_mps), 6),
                    "speed_stddev_mps": round(measurement.speed_stddev_mps, 6),
                    "corridor_width_m": round(measurement.corridor_width_m, 6),
                    "speed_limit_mps": measurement.speed_limit_mps,
                }
            )
            if violation is not None:
                events.append(
                    {
                        "event_id": violation.event_id,
                        "station_m": measurement.station_m,
                        "exceedance_mps": round(violation.exceedance_mps, 6),
                        "speed_limit_mps": measurement.speed_limit_mps,
                    }
                )

    station_errors = [sample["station_error_m"] for sample in samples]
    speed_errors = [entry["speed_error_mps"] for entry in measurements]
    ranks = [sample["correspondence_rank"] for sample in samples]
    visibility = [sample["all_accepted_unoccluded"] for sample in samples]
    return {
        "path_speed_mps": truth_speed_mps,
        "frames": frames,
        "accepted_frames": len(samples),
        "gates_measured_m": [entry["station_m"] for entry in measurements],
        "max_gate_speed_error_mps": max(speed_errors) if speed_errors else None,
        "max_station_error_m": max(station_errors) if station_errors else None,
        "minimum_correspondence_rank": min(ranks) if ranks else None,
        "all_accepted_unoccluded": (
            all(entry is True for entry in visibility) if check_visibility else None
        ),
        "violations": events,
        "gate_measurements": measurements,
        "samples": samples,
    }


def run_report(
    scratch_dir: Path,
    schedule: Schedule | None = None,
    profiles: tuple[str, ...] = PROFILE_NAMES,
    check_visibility: bool = True,
) -> dict[str, Any]:
    """Build a scene, drive the declared schedule, and return the summary.

    ``scratch_dir`` receives the generated stage, manifest and marker PNGs. It
    is deliberately *not* where the summary goes: pointing ``--out`` at
    ``docs/evidence/`` used to drop build output into the curated tree, which
    the comment in ``main`` claimed could not happen.
    """

    scratch_dir.mkdir(parents=True, exist_ok=True)
    # The AUTHORED scene, explicitly: this report is the v1 camera program and
    # its schedule windows are authored metres. Following the default to the
    # robot-scale scenario would sample a corridor a third the length while
    # still asking for (6.0, 3.0), which resolve_profiles appends as a new
    # profile rather than refusing.
    stage_path, manifest_path = build_scene(
        authored_config_path(), scratch_dir / "corridor.usda", 6.0, 3.0
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    schedule = schedule or Schedule(
        rate_hz=float(manifest["camera"]["rate_hz"]),
        speeds_mps=SCHEDULE_SPEEDS_MPS,
        window_x_m=SCHEDULE_WINDOW_X_M,
    )

    per_profile: dict[str, Any] = {}
    for profile_name in profiles:
        runs = [
            _run_one(
                manifest_path,
                stage_path,
                profile_name,
                speed,
                schedule,
                check_visibility,
            )
            for speed in schedule.speeds_mps
        ]
        speed_errors = [run["max_gate_speed_error_mps"] for run in runs if run["gates_measured_m"]]
        station_errors = [run["max_station_error_m"] for run in runs if run["accepted_frames"]]
        ranks = [
            run["minimum_correspondence_rank"]
            for run in runs
            if run["minimum_correspondence_rank"] is not None
        ]
        per_profile[profile_name] = {
            "max_gate_speed_error_mps": max(speed_errors) if speed_errors else None,
            "max_station_error_m": max(station_errors) if station_errors else None,
            "minimum_correspondence_rank": min(ranks) if ranks else None,
            "gates_measured_m": sorted(
                {station for run in runs for station in run["gates_measured_m"]}
            ),
            "all_accepted_unoccluded": (
                all(run["all_accepted_unoccluded"] is True for run in runs)
                if check_visibility
                else None
            ),
            "runs": runs,
        }

    enforcement_gates = sorted(
        {
            float(marker["station_m"])
            for marker in manifest["profiles"][profiles[0]]["markers"]
            if marker.get("role", "gate") == "gate"
        }
    )
    # A speed needs two gate crossings, so the *first* gate arms the estimator
    # and can never carry a measurement of its own. Comparing the measured set
    # against every authored gate therefore reported False on every run of every
    # profile while coverage was in fact complete -- and, worse, could not move
    # if a gate were genuinely lost. Coverage is judged against the gates that
    # can carry a speed.
    measurable_gates = enforcement_gates[1:]
    overall_speed = [
        block["max_gate_speed_error_mps"]
        for block in per_profile.values()
        if block["max_gate_speed_error_mps"] is not None
    ]
    overall_station = [
        block["max_station_error_m"]
        for block in per_profile.values()
        if block["max_station_error_m"] is not None
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_utc": datetime.now(UTC).isoformat(timespec="seconds"),
        "provenance": {
            "tool": "tools/synthetic_observer_report.py",
            "commit": _git_commit(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opencv": cv2.__version__,
            "scene": {"m": 6.0, "n": 3.0, "config": "src/corridor_scene/config/corridor.yaml"},
            "manifest_schema_version": manifest["schema_version"],
            "renderer": (
                "synthetic pinhole projection of the surveyed plates; no GPU, no Isaac. "
                "This measures the estimator against the survey, not the RTX renderer."
            ),
        },
        "schedule": schedule.as_dict(),
        "profiles": per_profile,
        "summary": {
            "max_gate_speed_error_mps": max(overall_speed) if overall_speed else None,
            "max_station_error_m": max(overall_station) if overall_station else None,
            "enforcement_gates_m": enforcement_gates,
            "measurable_gates_m": measurable_gates,
            "every_measurable_gate_measured": all(
                block["gates_measured_m"] == measurable_gates for block in per_profile.values()
            ),
            "minimum_correspondence_rank": min(
                block["minimum_correspondence_rank"]
                for block in per_profile.values()
                if block["minimum_correspondence_rank"] is not None
            ),
            "all_accepted_unoccluded": (
                all(block["all_accepted_unoccluded"] is True for block in per_profile.values())
                if check_visibility
                else None
            ),
        },
    }


def _git_commit() -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (subprocess.CalledProcessError, OSError):  # pragma: no cover - not a git checkout
        return "unknown"


def arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"summary JSON path (default {DEFAULT_OUT})",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="overwrite an existing summary; without it an existing file is an error",
    )
    parser.add_argument(
        "--skip-visibility",
        action="store_true",
        help="skip the composed-stage raycast audit of accepted markers",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = arguments(argv)
    output = args.out.resolve()
    # Evidence is promoted into docs/ by hand after review, never written there
    # by a tool, so a silent overwrite of a published artifact is not possible.
    if output.exists() and not args.force:
        raise FileExistsError(f"{output} exists; pass --force to replace it")
    # The scene the reporter builds is an implementation detail of the
    # measurement, not part of the evidence, so it goes to a temporary
    # directory rather than next to the artifact.
    with tempfile.TemporaryDirectory(prefix="synthetic-observer-") as scratch:
        report = run_report(Path(scratch), check_visibility=not args.skip_visibility)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    summary = report["summary"]
    print(f"wrote {output}")
    print(f"  max gate-derived speed error : {summary['max_gate_speed_error_mps']:.4f} m/s")
    print(f"  max per-frame station error  : {summary['max_station_error_m']:.4f} m  (secondary)")
    print(
        f"  every measurable gate measured: {summary['every_measurable_gate_measured']}"
        f"  {summary['measurable_gates_m']}"
    )
    print(f"  minimum correspondence rank  : {summary['minimum_correspondence_rank']}")
    print(f"  all accepted markers unoccluded: {summary['all_accepted_unoccluded']}")
    for name, block in report["profiles"].items():
        print(
            f"  {name:22s} speed {block['max_gate_speed_error_mps']:.4f} m/s"
            f"  station {block['max_station_error_m']:.4f} m"
        )
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
