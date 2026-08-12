#!/usr/bin/env python3
"""Does the arena the simulator will load hold the scenario the run will plan?

    python3 tools/check_arena_matches_manifest.py \
        --arena out/arena_corridor_robot1_nominal_m6_n3.usd \
        --manifest out/corridor.manifest.json --profile nominal_m6_n3 \
        --json out/evidence/robot-a-gate/<run>/arena-check.json

WHY
---
On 2026-08-12 every corridor run since the rescale drove a **0.30-scale plan
inside a 1.0-scale arena**. The arenas on disk were the unscaled 12 m scene --
corridor x from -2.0 to 18.5, B at (16.79, -8.0), and no landmark post in the
stage at all -- while `corridor_nav_gate` planned from the 0.30-scale manifest
and put the goal at (4.11, -2.93).

Nothing failed. The goal was accepted, Nav2 drove to it, reported SUCCEEDED, and
the run recorded a 5.754 m world-frame delivery error and a landmark "confirmed"
at 1.06 m in a stage that contains no post. Every artifact looked like a robot
problem.

Two paths defaulting to the same stale file is how it happened, and correcting
those defaults is not enough on its own: the next drift will come from
somewhere else. This is the check that makes the class of fault visible instead
of the instance.

WHAT IT COMPARES
----------------
Positions the manifest states outright, read back out of the composed stage:
B, B's landmark post, and the corridor's own extent. Those three disagree the
moment the arena and the manifest are different scenarios, and they agree under
any change that is genuinely the same scene.

It does NOT re-derive geometry. Re-deriving the walls here would be a second
implementation of the authoring rules, which is the thing this repository keeps
one copy of on purpose.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from pxr import Usd, UsdGeom

#: How far a rebuilt arena may sit from the manifest and still be the same
#: scenario. Authoring is deterministic, so honest disagreement is at the
#: rounding level; the failure this exists to catch was off by TWELVE METRES.
TOLERANCE_M = 0.05

CORRIDOR_PATH = "/World/Environment/Corridor"
ACTORS_PATH = "/World/Actors"


def _bounds(stage: Usd.Stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    low, high = box.GetMin(), box.GetMax()
    # An empty range comes back inverted with float-max sentinels.
    if low[0] > 1e37:
        return None
    return [round(v, 4) for v in low], [round(v, 4) for v in high]


def _centre(bounds) -> list[float] | None:
    if bounds is None:
        return None
    low, high = bounds
    return [round((a + b) / 2.0, 4) for a, b in zip(low, high, strict=True)]


def compare(arena: Path, manifest_path: Path, profile: str) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    # `Usd.Stage.Open` RAISES on a missing file rather than returning None, and
    # a precondition that dies with a traceback is a precondition nobody reads.
    try:
        stage = Usd.Stage.Open(str(arena))
    except Exception:  # noqa: BLE001 - any open failure is the same verdict here
        stage = None
    if stage is None:
        return {
            "arena": str(arena), "manifest": str(manifest_path), "profile": profile,
            "pass": False, "failures": [f"arena will not open: {arena}"], "checks": [],
        }

    actors = manifest["actors"]
    checks: list[dict] = []

    def planar(name: str, expected, measured) -> None:
        if expected is None or measured is None:
            checks.append({
                "name": name, "expected": expected, "measured": measured,
                "error_m": None, "pass": False,
                "note": "absent from the arena" if measured is None else "absent from the manifest",
            })
            return
        error = max(abs(expected[0] - measured[0]), abs(expected[1] - measured[1]))
        checks.append({
            "name": name,
            "expected": [round(expected[0], 4), round(expected[1], 4)],
            "measured": [round(measured[0], 4), round(measured[1], 4)],
            "error_m": round(error, 4),
            "pass": error <= TOLERANCE_M,
        })

    planar("B", actors.get("b_xyz_m"), _centre(_bounds(stage, f"{ACTORS_PATH}/B")))
    planar(
        "B's landmark post",
        actors.get("landmark_xyz_m"),
        _centre(_bounds(stage, f"{ACTORS_PATH}/BLandmark")),
    )

    corridor = _bounds(stage, CORRIDOR_PATH)
    if corridor is None:
        checks.append({"name": "corridor extent", "pass": False,
                       "note": f"{CORRIDOR_PATH} is absent from the arena"})
    else:
        low, high = corridor
        # The east kerb and the street's far end are stated by the manifest, so
        # they are compared rather than re-derived.
        street = manifest["next_street"]
        planar("corridor east/south extent", [street["east_x_m"], street["south_y_m"]],
               [high[0] - manifest["wall_thickness_m"], low[1]])

    failures = [
        f"{check['name']}: expected {check.get('expected')} but the arena has "
        f"{check.get('measured')}"
        + (f" ({check['note']})" if check.get("note") else "")
        for check in checks if not check["pass"]
    ]
    return {
        "arena": str(arena),
        "manifest": str(manifest_path),
        "profile": profile,
        "tolerance_m": TOLERANCE_M,
        "checks": checks,
        "failures": failures,
        "pass": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--arena", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--json", type=Path, help="where to write the artifact")
    arguments = parser.parse_args()

    report = compare(arguments.arena, arguments.manifest, arguments.profile)
    if arguments.json:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    for check in report["checks"]:
        verdict = "ok  " if check["pass"] else "FAIL"
        print(f"  {verdict} {check['name']}: manifest {check.get('expected')} "
              f"arena {check.get('measured')} (err {check.get('error_m')} m)")
    if report["pass"]:
        print(f"arena and manifest agree within {TOLERANCE_M} m")
        return 0
    print("\n**THE ARENA IS NOT THE SCENARIO THE RUN WOULD PLAN**", file=sys.stderr)
    for failure in report["failures"]:
        print(f"  {failure}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
