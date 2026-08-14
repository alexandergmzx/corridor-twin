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
    record_path = run / "run.json"
    record = (
        json.loads(record_path.read_text(encoding="utf-8"))
        if record_path.exists() else {}
    )
    # The runner's own vocabulary, checked rather than guessed: it writes
    # "result", "rerun" or "crash". An earlier version of this filter tested
    # for "infrastructure", which the runner never writes, so it excluded
    # nothing and the runs it was meant to set aside simply vanished from the
    # table instead -- worse than counting them, because nothing showed they
    # were missing.
    classification = record.get("classification")
    if classification and classification != "result":
        # A session that never got far enough to answer the question. Gate
        # discipline: a rerun, never a result, and it must not silently join
        # the tallies as a bad outcome.
        return {
            "run": run.name, "profile": record.get("profile", "?"),
            "kind": classification, "state": "-", "refinements": None,
            "closest_m": None, "walked_m": None, "docked_on": "-",
            "note": record.get("classification_cause", "infrastructure"),
        }
    if not gate_path.exists() or not nav_path.exists():
        return None
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    nav = json.loads(nav_path.read_text(encoding="utf-8"))

    # EVIDENCE INTEGRITY. `world_frame_delivery` is scored against the
    # evaluator's own ground-truth subscription, and on 2026-08-13 17:46 that
    # stream died after 0.81 s. The evaluator then scored the run against A's
    # SPAWN and reported a 5.0072 m delivery error -- for a run that actually
    # drove to (4.456, -2.251), 0.150 m from the standoff, and docked on the
    # real B at 0.616 m against a true 0.601 m. A dead sensor on the
    # EVALUATION plane is not a robot failure, and scoring it as one is how a
    # good run gets recorded as the worst of the day.
    #
    # Every other run in the population reports 5.4-10.7 m here, so near-zero
    # is unambiguous rather than a judgement call.
    # ...but near-zero truth is AMBIGUOUS, and the first version of this guard
    # got it exactly backwards on the second case it met. It means either
    #
    #   (a) the evaluator died  -- run 174631: odom travelled 4.925 m while
    #       truth recorded 0.000, and the bag shows A drove the full route and
    #       delivered to 0.150 m. Not evidence about the robot; exclude.
    #   (b) the robot did not MOVE -- run 195321: odom travelled 0.000 and
    #       truth recorded 0.008, and the bag holds 2386 truth samples over a
    #       0.128 m path. The evaluator was fine. That is a real failure and
    #       excluding it HIDES the thing worth seeing.
    #
    # A guard built after being fooled one way must not fool the reader the
    # other way. The gate's own odometry separates them: truth silent while
    # odometry reports metres can only be the evaluator.
    truth_distance = gate.get("ground_truth_distance_m")
    odom_travelled = nav.get("travelled_m")
    evaluator_died = (
        truth_distance is not None and truth_distance < 0.5
        and odom_travelled is not None and odom_travelled > 1.0
    )
    if evaluator_died:
        return {
            "run": run.name, "profile": nav.get("profile", "?"),
            "kind": "truth-dead", "state": "-", "refinements": None,
            "closest_m": None, "walked_m": None, "docked_on": "-",
            "note": (f"evaluator truth stream died: odom drove {odom_travelled} m, "
                     f"truth recorded {truth_distance} m"),
        }

    delivery = gate.get("world_frame_delivery") or {}
    dock = nav.get("docking") or {}
    row = {
        "run": run.name,
        "kind": "result",
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

    # Docking ENABLED but absent means the gate returned before its dock loop
    # -- on 2026-08-13 17:17 because bt_navigator's acceptance response was
    # lost while the robot drove 10.709 m anyway. The transit is a result; the
    # DOCKING is not, and counting it as a docking failure would understate
    # every docking change measured against it.
    if nav.get("docking") is None:
        row["kind"] = "dock-n/a"
        row["note"] = nav.get("failure", "docking never ran")

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
        if row["kind"] != "result":
            print(f"{row['run']:<44} {'EXCLUDED':<9} {row['kind']:>3}  "
                  f"{row.get('note', '')[:40]}")
            continue
        closest = f"{row['closest_m']:.4f}" if row["closest_m"] is not None else "-"
        walked = f"{row['walked_m']:.3f}" if row["walked_m"] is not None else "-"
        print(f"{row['run']:<44} {row['state']:<9} {str(row['refinements']):>3} "
              f"{closest:>8} {walked:>7} {row['docked_on']:>10}")

    infra = [r for r in rows if r["kind"] in ("rerun", "crash")]
    no_dock = [r for r in rows if r["kind"] == "dock-n/a"]
    truth_dead = [r for r in rows if r["kind"] == "truth-dead"]
    rows = [r for r in rows if r["kind"] == "result"]
    if not rows:
        print("\n  no scoreable runs")
        return 1
    docked = [r for r in rows if r["state"] == "DOCKED"]
    on_b = [r for r in docked if r["docked_on"] == "B"]
    on_stub = [r for r in docked if r["docked_on"] == "stub"]
    closest = [r["closest_m"] for r in rows if r["closest_m"] is not None]
    stayed = [r for r in rows if (r["walked_m"] or 0.0) < 0.3]
    within = [c for c in closest if c <= 0.15]

    print(f"\n  scoreable runs           {len(rows)}")
    print(f"  excluded: rerun / crash  {len(infra)}")
    print(f"  excluded: dock never ran {len(no_dock)}")
    print(f"  excluded: truth stream   {len(truth_dead)}")
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
