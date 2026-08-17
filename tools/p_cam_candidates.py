#!/usr/bin/env python3
"""Where should P's enforcement camera stand? Geometry first, opinions later.

    python3 tools/p_cam_candidates.py --manifest out/corridor.manifest.json \
        --profile nominal_m6_n3 --out docs/evidence/p_cam_candidates/geometry.json

ADR 0021 moved the single render product from A's front camera to P's roadside
instrument, and nothing has yet decided where P's instrument looks. This
computes the geometry for candidate poses so that decision is made on measured
numbers rather than on a viewport that looked nice.

It CHOOSES NOTHING. It reports.

WHAT THE CAMERA HAS TO SEE, and it is not the wall plates
---------------------------------------------------------
In v1 the surveyed ArUco plates were what A's OWN camera measured its station
against. In v2 the camera is P's and the subject is **A itself** (ADR 0021), so
what matters is whether a candidate pose can see the ROUTE at each enforcement
station -- the place A will be when its speed is evaluated. The plates are
reported alongside, because the ArUco-on-A baseline still needs a plate-sized
target to be resolvable at that range.

WHAT IS MEASURED, per candidate and per enforcement station
-----------------------------------------------------------
* **Line of sight** — does the segment from the camera to the marker cross an
  authored wall? Tested in 2-D against the manifest's own wall polygons, the
  same source the scan filter's model comes from.
* **Distance** — how far the marker is. A detector's pixel error becomes metric
  error in proportion to it.
* **Incidence** — the angle between the camera ray and the marker's own normal.
  A plate seen edge-on is a plate the detector cannot measure: at 90 deg it has
  no width left. The gate plates are canted 35 deg off the wall for exactly this
  reason.
* **In frustum** — bearing inside the declared horizontal FOV, and the marker
  above the horizon of a camera at P's mount height.

The camera contract (640x360, 75 deg, mount height) is read from the manifest
rather than restated, so a re-scaled scenario moves the answer with it.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _segments(walls: dict) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    out = []
    for corners in walls.values():
        points = [(float(x), float(y)) for x, y in corners]
        for start, end in zip(points, points[1:] + points[:1], strict=True):
            out.append((start, end))
    return out


def _crosses(a, b, c, d) -> bool:
    """Do segments ab and cd properly intersect?"""

    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1, d2 = side(c, d, a), side(c, d, b)
    d3, d4 = side(a, b, c), side(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def blocked(eye, target, segments, skip_near_m: float = 0.05) -> bool:
    """Line of sight, ignoring the wall the target is mounted ON.

    A plate sits in the plane of its own wall, so the segment to it grazes that
    wall by construction; `skip_near_m` pulls the endpoint back so the mounting
    surface is not counted as its own occluder.
    """

    dx, dy = target[0] - eye[0], target[1] - eye[1]
    length = math.hypot(dx, dy)
    if length <= skip_near_m:
        return False
    shortened = (
        eye[0] + dx * (1.0 - skip_near_m / length),
        eye[1] + dy * (1.0 - skip_near_m / length),
    )
    return any(_crosses(eye, shortened, c, d) for c, d in segments)


def route_point(entry: dict, station_m: float) -> tuple[float, float] | None:
    """Where A is at a given station, along the manifest's approach leg.

    The enforcement stations all sit on the approach, which is a straight leg
    from A's spawn along `approach_heading`, so this is exact rather than a
    resampling of the arc.
    """

    trajectory = entry["delivery_trajectory"]
    if station_m > trajectory["approach_length_m"]:
        return None
    start = trajectory["start_xyz_m"]
    heading = trajectory["approach_heading"]
    return (start[0] + heading[0] * station_m, start[1] + heading[1] * station_m)


def evaluate(manifest: dict, profile: str, candidates: list[dict]) -> dict:
    entry = manifest["profiles"][profile]
    segments = _segments(entry["walls"])
    camera = manifest["camera"]
    half_fov = math.radians(camera["horizontal_fov_deg"]) / 2.0

    stations = {}
    for marker in entry["markers"]:
        if marker["role"] != "gate":
            continue
        stations.setdefault(round(marker["station_m"], 3), []).append(marker)

    results = []
    for candidate in candidates:
        eye = tuple(candidate["eye_xyz_m"])
        look = tuple(candidate["look_at_xyz_m"])
        heading = math.atan2(look[1] - eye[1], look[0] - eye[0])
        # THE SUBJECT: A on its route. `route` maps station -> the point A
        # occupies there, taken from the manifest's own trajectory.
        route_rows = []
        for station in sorted(stations):
            point = route_point(entry, station)
            if point is None:
                continue
            dx, dy = point[0] - eye[0], point[1] - eye[1]
            distance = math.hypot(dx, dy)
            bearing = (math.atan2(dy, dx) - heading + math.pi) % (2 * math.pi) - math.pi
            route_rows.append({
                "station_m": station,
                "subject_xy_m": [round(point[0], 3), round(point[1], 3)],
                "distance_m": round(distance, 3),
                "bearing_deg": round(math.degrees(bearing), 2),
                "in_frustum": abs(bearing) <= half_fov,
                "line_of_sight": not blocked(eye[:2], (point[0], point[1]), segments),
            })
            route_rows[-1]["usable"] = (
                route_rows[-1]["in_frustum"] and route_rows[-1]["line_of_sight"]
            )

        rows = []
        for station in sorted(stations):
            # One row per station, taking the marker the camera sees best.
            best = None
            for marker in stations[station]:
                centre = [
                    sum(corner[axis] for corner in marker["corners_xyz_m"]) / 4.0
                    for axis in range(3)
                ]
                dx, dy = centre[0] - eye[0], centre[1] - eye[1]
                distance = math.hypot(dx, dy)
                bearing = (math.atan2(dy, dx) - heading + math.pi) % (2 * math.pi) - math.pi
                normal = marker["normal_xyz"]
                # Incidence: 0 deg is square-on, 90 deg is edge-on and useless.
                ray = (-dx / distance, -dy / distance)
                cosine = max(-1.0, min(1.0, ray[0] * normal[0] + ray[1] * normal[1]))
                incidence = math.degrees(math.acos(abs(cosine)))
                row = {
                    "station_m": station,
                    "marker_id": marker["id"],
                    "side": marker["side"],
                    "distance_m": round(distance, 3),
                    "bearing_deg": round(math.degrees(bearing), 2),
                    "incidence_deg": round(incidence, 2),
                    "in_frustum": abs(bearing) <= half_fov,
                    "line_of_sight": not blocked(eye[:2], (centre[0], centre[1]), segments),
                }
                row["usable"] = row["in_frustum"] and row["line_of_sight"]
                if best is None or (row["usable"], -row["incidence_deg"]) > (
                    best["usable"], -best["incidence_deg"]
                ):
                    best = row
            rows.append(best)

        usable = [r for r in rows if r["usable"]]
        route_usable = [r for r in route_rows if r["usable"]]
        results.append({
            **candidate,
            "heading_deg": round(math.degrees(heading), 2),
            # THE HEADLINE: how much of A's route this pose can actually watch.
            "route_stations": route_rows,
            "route_usable": len(route_usable),
            "route_total": len(route_rows),
            "route_distance_range_m": (
                [min(r["distance_m"] for r in route_usable),
                 max(r["distance_m"] for r in route_usable)]
                if route_usable else None
            ),
            "stations": rows,
            "usable_stations": len(usable),
            "total_stations": len(rows),
            "distance_range_m": (
                [min(r["distance_m"] for r in usable), max(r["distance_m"] for r in usable)]
                if usable else None
            ),
            "worst_incidence_deg": (
                max(r["incidence_deg"] for r in usable) if usable else None
            ),
        })

    return {
        "profile": profile,
        "camera_contract": camera,
        "station_count": len(stations),
        "candidates": results,
        "decision": "NONE -- this tool reports; the choice is Alexander's",
    }


def default_candidates(manifest: dict, profile: str) -> list[dict]:
    """Three poses worth measuring, all of them P's own instrument.

    P stands at the corner (ADR 0019), so the camera goes where P is or on the
    surfaces P owns. Each trades the same two things against each other:
    distance to the far gates, and incidence on the near ones.
    """

    entry = manifest["profiles"][profile]
    low = entry["police_bounds_min_xyz_m"]
    high = entry["police_bounds_max_xyz_m"]
    p_xy = ((low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0)
    mount = manifest["camera"]["mount_height_m"]
    length = manifest["corridor_length_m"]
    north_y = high[1]

    return [
        {
            "name": "at_P_down_the_corridor",
            "why": "P's own position, looking west along the corridor axis. The "
                   "roadside-enforcement pose: the camera is where the officer is.",
            "eye_xyz_m": [p_xy[0], p_xy[1], mount],
            "look_at_xyz_m": [0.0, p_xy[1], mount],
        },
        {
            "name": "at_P_raised",
            "why": "Same footprint, mounted higher -- a pole rather than a "
                   "shoulder. Buys depression angle over the near gates without "
                   "moving P.",
            "eye_xyz_m": [p_xy[0], p_xy[1], mount * 3.0],
            "look_at_xyz_m": [0.0, p_xy[1], 0.0],
        },
        {
            "name": "north_wall_before_the_screen",
            "why": "On the north wall just WEST of ADR 0019's corner screen, "
                   "looking back down the corridor. The first pose whose sight "
                   "line does not start behind the screen.",
            "eye_xyz_m": [length - 0.3, north_y, mount * 2.0],
            "look_at_xyz_m": [0.0, 0.0, mount],
        },
        {
            "name": "corner_mast_over_the_screen",
            "why": "P's own footprint on a mast taller than the screen. THE 2-D "
                   "TEST BELOW CANNOT JUDGE THIS ONE -- it reports blocked "
                   "because the screen is in the way in plan, which is exactly "
                   "what a mast is for. Flagged for the 3-D check.",
            "eye_xyz_m": [p_xy[0], p_xy[1], 1.5],
            "look_at_xyz_m": [0.0, 0.0, 0.0],
        },
        {
            "name": "north_wall_midpoint",
            "why": "Mounted on the north wall halfway along, looking back east "
                   "toward the corner. Halves the distance to the far gates; "
                   "gives up the head-on view of the approach.",
            "eye_xyz_m": [length / 2.0, north_y, mount * 2.0],
            "look_at_xyz_m": [p_xy[0], p_xy[1], mount],
        },
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--profile", default="nominal_m6_n3")
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()

    manifest = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    report = evaluate(
        manifest, arguments.profile, default_candidates(manifest, arguments.profile)
    )
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"profile {report['profile']}, {report['station_count']} enforcement stations")
    for candidate in report["candidates"]:
        print(f"\n  {candidate['name']}  eye {candidate['eye_xyz_m']} "
              f"heading {candidate['heading_deg']} deg")
        print(f"    ROUTE  usable {candidate['route_usable']}/{candidate['route_total']}"
              f"   distance {candidate['route_distance_range_m']}")
        for row in candidate["route_stations"]:
            flag = "ok " if row["usable"] else "NO "
            print(f"      {flag} A at station {row['station_m']:.2f} m -> "
                  f"{row['subject_xy_m']}  {row['distance_m']:5.2f} m  "
                  f"bearing {row['bearing_deg']:7.2f}  "
                  f"{'los' if row['line_of_sight'] else 'BLOCKED'}"
                  f"{'' if row['in_frustum'] else '  out of frustum'}")
        print(f"    PLATES usable {candidate['usable_stations']}/{candidate['total_stations']}"
              f"   distance {candidate['distance_range_m']}"
              f"   worst incidence {candidate['worst_incidence_deg']} deg")
        for row in candidate["stations"]:
            flag = "ok " if row["usable"] else "NO "
            print(f"      {flag} station {row['station_m']:.2f} m  id {row['marker_id']:3d}  "
                  f"{row['distance_m']:5.2f} m  bearing {row['bearing_deg']:7.2f}  "
                  f"incidence {row['incidence_deg']:5.1f}  "
                  f"{'los' if row['line_of_sight'] else 'BLOCKED'}")
    print(f"\nwritten: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
