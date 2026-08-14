#!/usr/bin/env python3
"""Derive a robot-scale corridor config from the authored one.

    python3 tools/scale_scenario.py --factor 0.3 \\
        --out src/corridor_scene/config/corridor-robot-scale.yaml

WHY THIS EXISTS
---------------
`docs/ROBO_TASK.pdf` labels the corridor widths only as the symbols m and n and
carries no scale bar. Every metric length in `corridor.yaml` is therefore an
explicit demo choice, never a surveyed value -- the scenario source fixes the
TOPOLOGY, not the dimensions.

Those demo choices were made before a robot existed. They give a 12 m corridor,
6 m wide at the entry, for a robot 0.20 m long and 0.16 m wide. That is a
corridor about 37 times the robot's width, and it is the direct cause of the
degeneracy measured in docs/degeneracy-study.md: the walls sit 3 m away, the end
wall 11.5 m away, and robot1's MS200 cannot range it at all until the robot is
3.5 m in. A scan matcher given nothing but two distant parallel walls has
nothing to lock onto along the corridor axis.

Scaling the WORLD toward the robot fixes that at the source, and it is faithful
to the drawing in a way that picking new arbitrary widths would not be: every
length moves by one factor, so every ratio in the scenario -- and therefore
every geometric argument built on it -- is preserved exactly.

WHAT SCALES AND WHAT DOES NOT
-----------------------------
Scaled: every length. Keys ending in `_m`, which the scenario uses consistently
for metres, plus the width thresholds of the speed policy, which are keyed to
corridor width and would otherwise stop meaning anything.

NOT scaled, deliberately:
  * `_deg`, `_px`, `_hz`, counts, fractions, ids -- not lengths.
  * `limit_mps`. Speeds are a POLICY choice, not a dimension, so they are
    PINNED here rather than multiplied -- see `PINNED_LIMITS_MPS` and ADR 0038.
    ADR 0023 said the table comes from a measured profile run; it was measured
    on 2026-08-14 over six runs and the numbers are that measurement's, not
    this factor's. Scaling v1's speeds by 0.30 would put every tier above what
    the robot can physically reach, so no violation could ever arise.
    `--v1-limits` keeps the source's un-pinned values for a v1 comparison.
  * The ROBOT. It is a real 0.20 x 0.16 m machine, and nav2's `robot_radius`
    describes it, not the world. The whole point is that the world shrinks
    toward a fixed robot.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

#: Keys whose values are lengths in metres. `_m` is the scenario's own
#: convention; the two explicit entries are lengths that do not carry the
#: suffix.
LENGTH_SUFFIX = "_m"
EXPLICIT_LENGTH_KEYS = {"maximum_width_m"}

#: Keys that end in `_m` by coincidence of another unit, or that are lengths
#: this tool must not touch.
#:
#: `b_radius_m` is here for the same reason the ROBOT is not scaled: it is a
#: physical dimension sized for a physical SENSOR, not a dimension of the
#: scenario. Scaling it broke a run outright. At factor 0.30 B's detectable
#: radius shrank to 0.045 m -- a 9 cm object, on which an MS200 puts 1.7 beams
#: at 3 m against the 4 the fitter needs -- so it became undetectable at range
#: while staying small enough that ordinary corner geometry fits a circle that
#: size. The detector confirmed a phantom at 0.910 m, docking re-aimed the
#: mission at it, and Nav2 drove half a metre and reported "Reached the goal!".
#:
#: A detectable radius follows the lidar's angular resolution, and that does
#: not change when the world does.
#:
#: `b_height_m` is deliberately NOT here: it describes a person, and a person
#: scales with the scenario. Since ADR 0031 B is one cylinder, so its two
#: dimensions genuinely answer to two different authorities -- which is exactly
#: why the split is named here rather than left to a reader to infer.
NOT_LENGTHS = {
    "limit_mps",
    "b_radius_m",
}

#: **The pinned v2 speed policy, in metres per second, widest tier first.**
#:
#: ADR 0023 left the width->limit table `[to pin after first profile run]` and
#: said the numbers come from a measured profile, not from the scale factor.
#: The profile was measured on 2026-08-14 across six runs and the table is
#: pinned here, under ADR 0038, ratified by the owner as ADR 0007 requires.
#:
#: These do NOT scale, and that is why they are a pin rather than a
#: multiplication. Scaling v1's 0.8/1.2/1.5 by 0.30 gives 0.24/0.36/0.45, and
#: robot1's measured band is 0.056-0.207 m/s -- entirely below all three. A
#: geometrically scaled policy is one no fleet robot can ever violate, so the
#: demonstration's central claim could not arise.
#:
#: Each tier is set from the measured band with margin, by one rule applied
#: three times: a permissive tier sits ABOVE the fastest measurement in its
#: zone, a strict tier BELOW the slowest.
#:
#:     wide   1.5 m <  width         0.207 max measured -> 0.30   (+45%)
#:     mid    1.2 m <  width <= 1.5  0.202 max measured -> 0.25   (+24%)
#:     strict          width <= 1.2  0.056 MIN measured -> 0.04   (-39%)
#:
#: Widths are the SCALED thresholds; only the speeds are pinned. The zone
#: boundaries are ADR 0016's and are untouched.
PINNED_LIMITS_MPS = (0.30, 0.25, 0.04)


def is_length_key(key: str) -> bool:
    if key in NOT_LENGTHS:
        return False
    if key in EXPLICIT_LENGTH_KEYS:
        return True
    return key.endswith(LENGTH_SUFFIX) and not key.endswith("_mps")


def scale_node(node: object, factor: float, key: str | None = None) -> object:
    """Recursively scale every length, leaving everything else untouched."""

    if isinstance(node, dict):
        return {name: scale_node(value, factor, name) for name, value in node.items()}
    if isinstance(node, list):
        return [scale_node(item, factor, key) for item in node]
    if isinstance(node, bool) or node is None:
        return node
    if isinstance(node, (int, float)) and key is not None and is_length_key(key):
        return round(float(node) * factor, 6)
    return node


def pin_limits(scaled: dict, limits=PINNED_LIMITS_MPS) -> list[float]:
    """Replace the v1 limits with the pinned v2 policy. -> the values written.

    Widest tier first, matched to the rules by ascending width threshold, so
    the pin cannot silently attach a strict speed to a wide zone if the rule
    order in the source ever changes.

    **Refuses rather than truncates** on a count mismatch. A source with four
    tiers and a three-value pin would otherwise leave one tier carrying a v1
    speed no fleet robot can reach, and the run would look compliant for a
    reason nobody could see.
    """

    rules = scaled["speed_policy"]["rules"]
    if len(rules) != len(limits):
        raise SystemExit(
            f"the pinned policy has {len(limits)} tiers and the scenario has "
            f"{len(rules)}; pin every tier or none (ADR 0038)")

    widest_first = sorted(rules, key=lambda rule: -rule["maximum_width_m"])
    for rule, limit in zip(widest_first, limits, strict=True):
        rule["limit_mps"] = limit
    # Read back rather than echoing the input, and in the order the header
    # claims, so a wrong pairing shows up in the generated file's own banner.
    return [rule["limit_mps"] for rule in widest_first]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default="src/corridor_scene/config/corridor.yaml")
    parser.add_argument("--factor", type=float, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--v1-limits", action="store_true",
        help="keep the source's un-pinned v1 limits instead of the ADR 0038 pin")
    arguments = parser.parse_args()

    if not 0.0 < arguments.factor <= 1.0:
        raise SystemExit("--factor must be in (0, 1]")

    source = Path(arguments.config)
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    scaled = scale_node(document, arguments.factor)
    pinned = None if arguments.v1_limits else pin_limits(scaled)

    header = (
        f"# DERIVED FILE -- do not hand-edit.\n"
        f"# Generated by tools/scale_scenario.py --factor {arguments.factor}\n"
        f"# from {source.as_posix()}.\n"
        f"#\n"
        f"# Every length is multiplied by {arguments.factor}; every ratio in the\n"
        f"# scenario is therefore unchanged, and so is every geometric argument\n"
        f"# built on it. The supplied drawing has no scale bar, so the authored\n"
        f"# metres were always demo choices -- this simply chooses them for a\n"
        f"# 0.20 x 0.16 m robot instead of for nothing in particular.\n"
        f"#\n"
        f"# NOT scaled: limit_mps, angles, pixels, rates, counts, and the\n"
        f"# robot itself.\n"
    )
    if pinned is not None:
        header += (
            f"#\n"
            f"# limit_mps is PINNED rather than scaled: {pinned} m/s, widest\n"
            f"# tier first, from robot1's profile measured over six runs on\n"
            f"# 2026-08-14 (ADRs 0023 and 0038). Scaling v1's speeds by\n"
            f"# {arguments.factor} would put every tier above what the robot can\n"
            f"# reach, so no violation could ever arise. Change them in\n"
            f"# tools/scale_scenario.py and regenerate; never here.\n"
        )
    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        header + yaml.safe_dump(scaled, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )

    geometry = scaled["geometry"]
    print(f"scaled by {arguments.factor} -> {destination}")
    if pinned is not None:
        print("  speed limits PINNED (not scaled): "
              + ", ".join(f"width<={rule['maximum_width_m']} -> {rule['limit_mps']} m/s"
                          for rule in scaled["speed_policy"]["rules"]))
    print(f"  corridor length {geometry['corridor_length_m']} m")
    for name, entry in geometry["profiles"].items():
        print(f"  {name:22} entry {entry['entry_width_m']} m  corner {entry['corner_width_m']} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
