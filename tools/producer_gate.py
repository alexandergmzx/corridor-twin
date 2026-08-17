#!/usr/bin/env python3
"""Fill in the producer gate from the adapter's own schedule, after it exits.

    python3 tools/producer_gate.py \
        --schedule out/evidence/crossing/drive-schedule-640x360.json \
        --crossing out/evidence/crossing/crossing-640x360.json

Separate from `crossing_measure.py` for a timing reason, not a design one: the
adapter writes its `--drive-out` schedule when the drive finishes, which is
after the capture window has closed. So the producer gate cannot be computed in
the same pass that measures the crossing, and pretending otherwise would mean
either shortening the capture or reading a file that is not there yet.

WHY THE SCHEDULE AND NOT A SUBSCRIBER
-------------------------------------
The producer gate asks how fast the adapter RENDERED, and a subscriber cannot
answer that: it can only report how many frames reached it, which on a
best-effort transport carrying ~691 kB images is a lower bound that moves with
message size. That confusion is exactly what the rung-1 analysis found -- image
arrivals were being read as the publisher's rate, while CameraInfo from the same
render product on the same tick said something 1.5 Hz different.

The schedule has no DDS in it at all. It is the adapter's per-update record of
simulation time, written by the process that owns the render loop, so the rate
derived from it is the publisher's own number.

It measures INTENT, not emission: a graph that silently failed to publish a
rendered frame would still appear here. The CameraInfo crossing ratio in the
same artifact is what covers that gap, which is why both are reported and
neither is quoted alone.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

SIMULATION_HZ = 60.0


def producer_rate(schedule: dict, declared_hz: float) -> dict:
    samples = schedule.get("samples", [])
    if len(samples) < 2:
        return {"error": "schedule has fewer than two samples", "samples": len(samples)}
    divider = max(1, round(SIMULATION_HZ / declared_hz))
    sim_span = float(samples[-1]["sim_time_s"]) - float(samples[0]["sim_time_s"])
    rendered = len(samples) // divider
    return {
        "updates_completed": len(samples),
        "update_divider": divider,
        "frames_the_adapter_rendered": rendered,
        "adapter_sim_span_s": round(sim_span, 3),
        "adapter_rate_hz_sim_basis": round(rendered / sim_span, 3) if sim_span else None,
        "path_speed_mps": schedule.get("path_speed_mps"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--schedule", required=True)
    parser.add_argument("--crossing", required=True)
    parser.add_argument("--declared-hz", type=float, default=15.0)
    parser.add_argument("--floor", type=float, default=0.95)
    arguments = parser.parse_args()

    schedule_path = Path(arguments.schedule)
    crossing_path = Path(arguments.crossing)
    if not schedule_path.is_file():
        print(f"**no adapter schedule at {schedule_path}** -- producer gate NOT MEASURED")
        return 1
    if not crossing_path.is_file():
        print(f"**no crossing artifact at {crossing_path}**")
        return 1

    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    crossing = json.loads(crossing_path.read_text(encoding="utf-8"))
    measured = producer_rate(schedule, arguments.declared_hz)
    if "error" in measured:
        print(f"**{measured['error']}**")
        return 1

    rate = measured["adapter_rate_hz_sim_basis"]
    ratio = rate / arguments.declared_hz if rate else None

    crossing.setdefault("gates", {})["producer"] = {
        "definition": (
            "frames the adapter rendered per simulation second, from its own "
            "schedule, vs the declared rate"
        ),
        **measured,
        "declared_hz": arguments.declared_hz,
        "ratio": round(ratio, 4) if ratio else None,
        "floor": arguments.floor,
        "pass": bool(ratio and ratio >= arguments.floor),
        "evidence": str(schedule_path),
    }
    crossing["producer"] = crossing["gates"]["producer"]
    crossing.setdefault("checks", {})["producer_gate"] = crossing["gates"]["producer"]["pass"]
    crossing["pass"] = all(crossing["checks"].values())

    crossing_path.write_text(json.dumps(crossing, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(crossing["gates"], indent=2))
    print(f"\nupdated: {crossing_path}")
    print("PRODUCER GATE:", "PASS" if crossing["gates"]["producer"]["pass"] else "**FAIL**")
    print(f"  {rate} Hz rendered vs {arguments.declared_hz} declared (ratio {ratio:.4f})")
    return 0 if crossing["gates"]["producer"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
