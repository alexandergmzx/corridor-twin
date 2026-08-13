#!/usr/bin/env python3
"""Summarise a batch of corridor runs into the four numbers that decide things.

    python3 tools/diagnostics/batch_summary.py [<run-dir> ...]

With no arguments it reads every run directory recorded today. Each run
contributes one row:

* **docked on** -- B, the `EastWallStub` decoy, or nothing, decided by
  comparing A's true final position against the detected range at DOCKED. This
  is the decoy study's open question and the only way to get a rate is to count.
* **closest approach** -- world-frame, to the delivery standoff. The criterion
  ADR 0033 proposes, measured whether or not it is pinned yet.
* **walked away** -- how far A left after its closest approach. The overshoot,
  in one number.
* **refinements** -- zero means the machine declared arrival without ever using
  the mechanism that closes the last stretch.

Truth is evaluation-plane only, exactly as in the gate's own artifacts.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

EVIDENCE = Path("out/evidence/robot-a-gate")

#: The stub's west face centre at the committed scale, from
#: `east_wall_stub_bounds`. See NOTES-the-eastwallstub-decoy-20260813.md.
STUB_XY = (4.56534, -1.926)
#: A detection is "on" an object if the range agrees with the truth distance to
#: it this closely. Generous: the question is which of two objects 0.6 m apart.
ATTRIBUTION_M = 0.15


def classify(run: Path) -> dict | None:
    gate_path, nav_path = run / "gate.json", run / "nav.json"
    if not gate_path.exists() or not nav_path.exists():
        return None
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    nav = json.loads(nav_path.read_text(encoding="utf-8"))

    delivery = gate.get("world_frame_delivery") or {}
    dock = nav.get("docking") or {}
    row = {
        "run": run.name,
        "profile": nav.get("profile", "?"),
        "state": dock.get("state", "-"),
        "refinements": dock.get("refinements"),
        "closest_m": delivery.get("closest_approach_m"),
        "walked_m": delivery.get("walked_away_m"),
        "map_error": next(
            (f for f in nav.get("failures", []) if "map-frame" in f), None
        ),
        "docked_on": "-",
    }

    final = delivery.get("final_position_m")
    docked = next(
        (h for h in dock.get("history", []) if h.get("event") == "docked"), None
    )
    if final and docked:
        manifest = json.loads(
            Path("out/corridor.manifest.json").read_text(encoding="utf-8")
        )
        b_x, b_y, _ = manifest["actors"]["b_xyz_m"]
        detected = docked["range_m"]
        to_b = math.dist(final, (b_x, b_y))
        to_stub = math.dist(final, STUB_XY)
        # Attribute by which object's TRUE distance the laser's range matches.
        candidates = {"B": abs(detected - to_b), "stub": abs(detected - to_stub)}
        best = min(candidates, key=candidates.get)
        row["docked_on"] = (
            best if candidates[best] <= ATTRIBUTION_M else f"?({detected:.2f}m)"
        )
        row["detected_m"] = detected
        row["truth_to_b_m"] = round(to_b, 4)
    return row


def main() -> int:
    if len(sys.argv) > 1:
        runs = sorted(Path(a) for a in sys.argv[1:])
    else:
        runs = sorted(p for p in EVIDENCE.glob("20260813-*") if p.is_dir())

    rows = [row for run in runs if (row := classify(run))]
    if not rows:
        print("no runs with both gate.json and nav.json")
        return 1

    print(f"{'run':<44} {'state':<9} {'ref':>3} {'closest':>8} {'walked':>7} "
          f"{'docked on':>10}")
    print("-" * 88)
    for row in rows:
        closest = f"{row['closest_m']:.4f}" if row["closest_m"] is not None else "-"
        walked = f"{row['walked_m']:.3f}" if row["walked_m"] is not None else "-"
        print(f"{row['run']:<44} {row['state']:<9} {str(row['refinements']):>3} "
              f"{closest:>8} {walked:>7} {row['docked_on']:>10}")

    docked = [r for r in rows if r["state"] == "DOCKED"]
    on_b = [r for r in docked if r["docked_on"] == "B"]
    on_stub = [r for r in docked if r["docked_on"] == "stub"]
    closest = [r["closest_m"] for r in rows if r["closest_m"] is not None]
    stayed = [r for r in rows if (r["walked_m"] or 0.0) < 0.3]
    within = [c for c in closest if c <= 0.15]

    print(f"\n  runs                     {len(rows)}")
    print(f"  reached DOCKED           {len(docked)}")
    print(f"    ...on B                {len(on_b)}")
    print(f"    ...on the stub decoy   {len(on_stub)}")
    print(f"  stayed (walked < 0.3 m)  {len(stayed)}")
    print(f"  closest approach <=0.15  {len(within)} of {len(closest)}")
    if closest:
        ordered = sorted(closest)
        print(f"  closest approach         min {ordered[0]:.4f}  "
              f"median {ordered[len(ordered) // 2]:.4f}  max {ordered[-1]:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
