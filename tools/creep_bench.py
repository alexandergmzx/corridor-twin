#!/usr/bin/env python3
"""Closed-loop terminal-docking bench. No ROS, no Isaac, no GPU, milliseconds.

    source .venv/bin/activate
    python tools/creep_bench.py --scenario all
    python tools/creep_bench.py --scenario cone_leak --json out/evidence/creep-bench/cone.json

WHY THIS EXISTS
---------------
On 2026-08-13 nine Isaac runs produced zero completed deliveries. Three creep
bugs were found, at roughly twenty-five minutes of wall clock each, and every
one of them was catchable offline in milliseconds:

  * a loop that exited on the goal it had just cancelled;
  * a creep that drove a tangent because it steered nothing;
  * a creep starved of ticks behind a TF lookup it never needed.

None of those needed a renderer. What they needed was the real detector, the
real state machine and the real safety filter, wired to each other in a loop,
with a robot that moves.

WHAT IS REAL HERE AND WHAT IS NOT
---------------------------------
REAL, imported and executed, never reimplemented:

  * `landmark_detector.LandmarkDetector`   -- the shape/convexity/persistence tests
  * `corridor_dock.DockingMachine`         -- arming, handoff, creep, stall
  * `yahboomcar_safety.governor`           -- `forward_min_range` and `decide`
  * `yahboomcar_sim.arena.raycast`         -- the fleet's own MS200-shaped raycaster
  * `export_scan_walls.wall_segments`      -- the corridor, from the manifest

SIMULATED, and deliberately crude:

  * unicycle kinematics, one Euler step per tick, no wheel dynamics
  * contact: A's front face against B's surface stops translation dead
  * slip: the same, except the ENCODERS keep reporting the commanded speed

**B IS A 32-GON, NOT A BEAM.** This is the whole point. Every docking test in
the fleet's `test_governor.py` models the target as a single return at a single
bearing (`_ahead()`), and a one-beam target cannot leak outside an angular
mask. That fixture blindness is exactly how a green unit test coexisted with a
robot that would not move: the in-process proof on 2026-08-13 reported 29 of 31
ticks moving with the mask declared, while the real robot pinned at 0.35 m. A
32-gon has 0.6 mm of radial error and leaks precisely as the real cylinder does.

WHAT THIS BENCH CANNOT TELL YOU
-------------------------------
Physics. Whether PhysX actually stops A against B's collider, whether the
wheels slip, and by how much, are Isaac questions -- the bench takes slip as an
input and asks what the software does about it, which is a different question
and the only one it can answer honestly.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

# NO `.resolve()`, NO `realpath`, ANYWHERE ON THIS PATH.
#
# The first draft of this file opened with `Path(__file__).resolve()`, two lines
# under a docstring warning against exactly that, and the bench then reported
# the fleet layout missing at /home/alexmint/Development/yahboomcar-ros2 -- the
# non-fleet sibling that resolving the symlink lands you in. D5 again, third
# time in two sessions.
#
# `build_corridor_arena` already owns the sanctioned resolver, is pure stdlib at
# module scope (no `pxr`), and is pinned by `test_corridor_arena_layout.py`
# including a negative control that a realpath implementation MUST fail. Import
# it rather than keep a second copy that can drift -- a second copy is how this
# went wrong in the first place.
_TOOLS = os.path.dirname(os.path.abspath(sys.argv[0])) if sys.argv[0] else "tools"
sys.path.insert(0, _TOOLS)

import build_corridor_arena as _layout  # noqa: E402

REPO = Path(_layout.logical_abspath(os.path.join(_layout.this_file(), "..", "..")))
sys.path.insert(0, str(REPO / "tools"))

from corridor_dock import (  # noqa: E402
    DockingMachine,
    final_approach_m,
)
from export_scan_walls import wall_segments  # noqa: E402
from landmark_detector import LandmarkDetector  # noqa: E402

#: The tick rate the live gate spins at (`corridor_nav_gate` spin_once 0.1 s).
TICK_HZ = 10.0
DT = 1.0 / TICK_HZ

#: The MS200 on the twin. 12 Hz against a 10 Hz control loop means roughly one
#: fresh scan per tick; the bench scans every tick, which is the optimistic
#: case and therefore the one that must still fail when the geometry says fail.
LIDAR_BEAMS = 360
LIDAR_R_MIN = 0.12
LIDAR_R_MAX = 8.0

#: B as a polygon. 32 sides puts the maximum radial error at
#: r*(1-cos(pi/32)) = 0.6 mm, two orders below anything that matters here.
B_FACETS = 32


def _fleet_src() -> Path:
    """The fleet `src/` directory, delegated to the composer's own resolver.

    Env-override-first, then a LOGICAL walk. Not reimplemented here: the
    composer's `fleet_src_root()` is the one copy, and it carries the unit test
    that fails a realpath-based implementation.
    """

    return Path(_layout.fleet_src_root())


def _import_fleet():
    """`raycast` and the governor, from the fleet, without writing bytecode.

    Nothing may drop a `__pycache__` into a sibling checkout, which is why
    `build_corridor_arena.py:455-460` does the same dance.
    """

    src = _fleet_src()
    paths = [
        src / "yahboomcar-ros2" / "yahboomcar_sim",
        src / "yahboomcar-ros2" / "yahboomcar_safety",
    ]
    missing = [str(p) for p in paths if not p.is_dir()]
    if missing:
        raise SystemExit(
            "fleet layout incomplete, missing:\n  " + "\n  ".join(missing)
            + "\n  set CORRIDOR_FLEET_SRC to a fleet src/ directory, or run from the"
              " symlinked path (robot-fleet/src/corridor-twin)"
        )
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        for path in paths:
            if str(path) not in sys.path:
                sys.path.insert(0, str(path))
        from yahboomcar_safety import governor as governor_module
        from yahboomcar_sim.arena import raycast
    finally:
        sys.dont_write_bytecode = previous
    return raycast, governor_module


def b_polygon(centre, radius, facets=B_FACETS):
    """B's silhouette as closed segments, so the segment raycaster sees a disc."""

    cx, cy = centre
    points = [
        (cx + radius * math.cos(2 * math.pi * i / facets),
         cy + radius * math.sin(2 * math.pi * i / facets))
        for i in range(facets)
    ]
    return [(points[i], points[(i + 1) % facets]) for i in range(facets)]


@dataclass
class World:
    """The corridor, B, and the two bodies' dimensions -- all from the manifest."""

    segments: list
    b_xy: tuple
    b_radius: float
    a_length: float

    @property
    def contact_range_m(self) -> float:
        return self.a_length / 2.0 + self.b_radius

    @classmethod
    def from_manifest(cls, manifest: dict, profile: str, with_b: bool = True):
        walls = wall_segments(manifest, profile)
        actors = manifest["actors"]
        b_xy = tuple(actors["b_xyz_m"][:2])
        b_radius = float(actors["b_radius_m"])
        segments = list(walls)
        if with_b:
            segments += b_polygon(b_xy, b_radius)
        return cls(segments, b_xy, b_radius,
                   float(actors["a_size_xyz_m"][0]))


@dataclass
class Robot:
    """Unicycle. `odom_*` is what the ENCODERS believe, which slip divorces."""

    x: float
    y: float
    yaw: float
    slip: bool = False
    odom_travel_m: float = 0.0
    measured_vx: float = 0.0
    contacted: bool = False

    def step(self, vx: float, wz: float, world: World) -> None:
        self.yaw += wz * DT
        blocked = self._would_touch(vx, world)
        if blocked:
            self.contacted = True
        moved = 0.0 if blocked else vx * DT
        self.x += moved * math.cos(self.yaw)
        self.y += moved * math.sin(self.yaw)
        self.odom_travel_m += abs(moved)
        # THE SLIP CASE. The wheels turn, the encoders integrate, and the robot
        # does not move. The twin authors rear friction at 0.1 and fuses wheel
        # twist only, so this is the twin's own documented behaviour, not a
        # pessimistic invention.
        self.measured_vx = vx if (blocked and self.slip) else (moved / DT)

    def _would_touch(self, vx: float, world: World) -> bool:
        if vx <= 0.0:
            return False
        ahead_x = self.x + vx * DT * math.cos(self.yaw)
        ahead_y = self.y + vx * DT * math.sin(self.yaw)
        return math.dist((ahead_x, ahead_y), world.b_xy) <= world.contact_range_m

    @property
    def range_to_b_centre(self):
        return None


def run_scenario(
    *,
    world: World,
    robot: Robot,
    governor_module,
    raycast,
    mask_mode: str,
    slow_zone_exempt: bool,
    max_seconds: float,
    start_state: str = "DOCKING",
    stall_witness: bool = True,
) -> dict:
    """One closed-loop creep. Returns a trace and a verdict.

    `mask_mode` is 'cone' (the shipped +/-15 deg mask), 'disc' (the silhouette
    proposal) or 'none'. Keeping all three lets the bench REPRODUCE the failure
    before it is trusted to certify the fix.
    """

    cfg = governor_module.GovernorConfig()
    machine = DockingMachine(
        nominal_goal=(0.0, 0.0),
        standoff_m=final_approach_m(world.b_radius, world.a_length),
        expected_radius_m=world.b_radius,
        a_length_m=world.a_length,
    )
    machine.state = start_state
    detector = LandmarkDetector(world.b_radius)

    trace: list[dict] = []
    duty_by_range: list[tuple[float, float]] = []
    ticks = int(max_seconds * TICK_HZ)
    for tick in range(ticks):
        ranges = raycast((robot.x, robot.y), robot.yaw, world.segments,
                         n_beams=LIDAR_BEAMS, r_min=LIDAR_R_MIN, r_max=LIDAR_R_MAX)
        verdict = detector.feed(list(ranges), -math.pi, 2 * math.pi / LIDAR_BEAMS,
                                LIDAR_R_MIN, LIDAR_R_MAX)

        now = tick * DT
        # `measured_vx=None` reads as "moving" in the machine, which disables
        # the stall path. Used by the reproductions that need to show where the
        # robot PINS -- otherwise the false stall below terminates them first
        # and hides the geometry under test.
        witness = robot.measured_vx if stall_witness else None
        command = machine.creep(verdict, witness, now)
        if command is None:
            break

        declared = command.get("approach")
        docking = _build_mask(governor_module, declared, mask_mode, world)
        min_range = governor_module.forward_min_range(
            list(ranges), -math.pi, 2 * math.pi / LIDAR_BEAMS, cfg, docking=docking
        )
        decision = _decide(governor_module, command, min_range, cfg, docking,
                           slow_zone_exempt)

        true_r = math.dist((robot.x, robot.y), world.b_xy)
        duty_by_range.append((true_r, decision.vx))
        trace.append({
            "t": round(now, 2), "true_range_m": round(true_r, 4),
            "commanded_vx": round(command["vx"], 4),
            "governed_vx": round(decision.vx, 4),
            "unmasked_min_range_m": (None if math.isinf(min_range)
                                     else round(min_range, 4)),
            "reason": decision.reason,
            "state": machine.state,
        })

        robot.step(decision.vx, decision.wz, world)
        if machine.state in machine.TERMINAL:
            break

    final_r = math.dist((robot.x, robot.y), world.b_xy)
    return {
        "mask_mode": mask_mode,
        "slow_zone_exempt": slow_zone_exempt,
        "slip": robot.slip,
        "state": machine.state,
        "contacted": robot.contacted,
        "final_true_range_m": round(final_r, 4),
        "contact_range_m": round(world.contact_range_m, 4),
        "elapsed_s": round(len(trace) * DT, 2),
        "duty": _duty_bands(duty_by_range),
        "history": machine.report().get("state"),
        "trace_tail": trace[-6:],
        "trace_len": len(trace),
    }


def _build_mask(governor_module, declared, mask_mode, world):
    """The declaration, as whichever mask shape is under test."""

    if declared is None or mask_mode == "none":
        return None
    if mask_mode == "cone":
        return governor_module.DockingApproach(
            bearing_rad=declared["bearing_rad"],
            range_m=declared["range_m"],
            margin_m=declared.get("margin_m", 0.10),
        )
    if mask_mode == "disc":
        disc = getattr(governor_module, "DockingDisc", None)
        if disc is None:
            raise SystemExit(
                "governor has no DockingDisc yet -- run the cone scenarios, or "
                "implement A1 first"
            )
        return disc(
            bearing_rad=declared["bearing_rad"],
            range_m=declared["range_m"],
            target_radius_m=world.b_radius,
            margin_m=declared.get("margin_m", 0.10),
        )
    raise SystemExit(f"unknown mask mode {mask_mode}")


def _decide(governor_module, command, min_range, cfg, docking, slow_zone_exempt):
    """`decide`, with the A2 exemption expressed as the caller's flag until it lands."""

    kwargs = {}
    if slow_zone_exempt:
        import inspect
        if "creep_exempt_slow_zone" in inspect.signature(
                governor_module.decide).parameters:
            kwargs["creep_exempt_slow_zone"] = True
    return governor_module.decide(
        command["vx"], 0.0, command["wz"], min_range, 0.05, 0.05, cfg,
        docking=docking, **kwargs,
    )


def _duty_bands(samples):
    """Fraction of ticks that MOVED, per range band.

    This is the statistic the 20260814-003844 bag replay produced, and the one
    the cone reproduction has to match: 98% moving while B closes to 0.42 m,
    then 28%, then 12%, then nothing.
    """

    bands = [(0.70, 0.42), (0.42, 0.38), (0.38, 0.35), (0.35, 0.0)]
    out = {}
    for high, low in bands:
        inside = [vx for r, vx in samples if low < r <= high]
        key = f"{high:.2f}-{low:.2f}"
        out[key] = (None if not inside
                    else round(sum(1 for v in inside if v > 1e-6) / len(inside), 3))
    return out


SCENARIOS: dict = {}


def scenario(name):
    def register(fn):
        SCENARIOS[name] = fn
        return fn
    return register


def _world_and_robot(manifest, profile, *, start_range, start_bearing_deg,
                     slip=False):
    """A placed `start_range` from B's centre, `start_bearing_deg` off the line to B.

    Approaching from the west, which is where the lane is and where every
    recorded run approached from.
    """

    world = World.from_manifest(manifest, profile)
    bx, by = world.b_xy
    x = bx - start_range
    y = by
    yaw = math.radians(start_bearing_deg)
    return world, Robot(x=x, y=y, yaw=yaw, slip=slip)


@scenario("cone_leak")
def _cone_leak(manifest, profile):
    """**THE REPRODUCTION.** The shipped +/-15 deg cone, run 003844's geometry.

    Must pin near 0.35 m and must NOT contact. If this scenario ever passes,
    the bench has stopped modelling the thing that actually happened and every
    other verdict it gives is worthless.

    The stall witness is OFF here. With it on, the run ends at 1.1 s for an
    unrelated reason -- see `slow_zone_false_stall` -- and the pin under test
    never gets a chance to happen.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0)
    return run_scenario(world=world, robot=robot, mask_mode="cone",
                        slow_zone_exempt=False, max_seconds=120.0,
                        stall_witness=False, **_fleet_kwargs())


@scenario("slow_zone_false_stall")
def _slow_zone_false_stall(manifest, profile):
    """**Found by this bench in 1.1 s, on its first run, and previously unnamed.**

    The stub's south face holds the creep in the slow zone at roughly 8.7 mm/s.
    `STALL_SPEED_MPS` is 10 mm/s. So a perfectly healthy throttled creep reads
    as stationary, the one-second debounce elapses, and the machine calls it an
    arrival -- about 0.39 m short of contact, while the robot is still moving.

    Under the shipped code this terminates in ~1.1 s. It must not, once A2
    exempts the clamped creep from the slow zone and the threshold is derived
    against the speed the creep actually commands.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0)
    result = run_scenario(world=world, robot=robot, mask_mode="cone",
                          slow_zone_exempt=False, max_seconds=25.0,
                          stall_witness=True, **_fleet_kwargs())
    result["false_stall"] = (result["elapsed_s"] < 5.0
                             and not result["contacted"])
    return result


@scenario("disc")
def _disc(manifest, profile):
    """A1 alone: silhouette mask, slow zone still active.

    Expected to reach contact but SLOWLY -- the stub's south face throttles the
    creep to ~8.7 mm/s, so this is the scenario that shows why A2 is needed.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0)
    return run_scenario(world=world, robot=robot, mask_mode="disc",
                        slow_zone_exempt=False, max_seconds=90.0,
                        **_fleet_kwargs())


@scenario("disc_exempt")
def _disc_exempt(manifest, profile):
    """A1 + A2: the shipping configuration. Contact in ~8-12 s."""

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0)
    return run_scenario(world=world, robot=robot, mask_mode="disc",
                        slow_zone_exempt=True, max_seconds=40.0,
                        **_fleet_kwargs())


@scenario("misaligned")
def _misaligned(manifest, profile):
    """A arrives mid-turn, 65 deg off -- the measured arrival band is 51-79.

    The tangent bug (run 003034) lived here: pure forward motion drove past B
    and the range GREW. Contact must still happen.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=65.0)
    return run_scenario(world=world, robot=robot, mask_mode="disc",
                        slow_zone_exempt=True, max_seconds=40.0,
                        **_fleet_kwargs())


@scenario("slip")
def _slip(manifest, profile):
    """Contact happens and the wheels keep spinning.

    The twin authors rear friction at 0.1 and the EKF fuses wheel twist only,
    so the encoders report motion through the bump. Under the encoder-only
    witness this must NOT reach DELIVERED_CONFIRMED -- that is the bug. Under
    A3 it must.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0, slip=True)
    return run_scenario(world=world, robot=robot, mask_mode="disc",
                        slow_zone_exempt=True, max_seconds=40.0,
                        **_fleet_kwargs())


@scenario("forgery")
def _forgery(manifest, profile):
    """**The dangerous one.** A governor stop, mistaken for a bump.

    The cone leak pins A at 0.31-0.35 m, INSIDE the 0.39 m sighting ceiling,
    with the encoders reading zero because the governor zeroed the output. One
    solid second of that forges DELIVERED_CONFIRMED about 0.13 m short of
    contact. It nearly fired in run 003844.

    Under the shipped code this is expected to REPORT A DELIVERY IT DID NOT
    MAKE. Under A3 it must refuse.
    """

    world, robot = _world_and_robot(manifest, profile, start_range=0.62,
                                    start_bearing_deg=0.0)
    result = run_scenario(world=world, robot=robot, mask_mode="cone",
                          slow_zone_exempt=False, max_seconds=25.0,
                          **_fleet_kwargs())
    result["forged"] = (result["state"] == "DELIVERED_CONFIRMED"
                        and not result["contacted"])
    return result


def _fleet_kwargs():
    raycast, governor_module = _import_fleet()
    return {"raycast": raycast, "governor_module": governor_module}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", default="all",
                        choices=["all", *sorted(SCENARIOS)])
    parser.add_argument("--profile", default="nominal_m6_n3")
    parser.add_argument("--manifest", default="out/corridor.manifest.json")
    parser.add_argument("--json", type=Path)
    arguments = parser.parse_args()

    manifest = json.loads(Path(arguments.manifest).read_text(encoding="utf-8"))
    names = sorted(SCENARIOS) if arguments.scenario == "all" else [arguments.scenario]

    results = {}
    for name in names:
        result = SCENARIOS[name](manifest, arguments.profile)
        results[name] = result
        contact = "CONTACT" if result["contacted"] else "no contact"
        print(f"  {name:14} {result['state']:20} {contact:11} "
              f"final r={result['final_true_range_m']:.4f} "
              f"(contact {result['contact_range_m']:.4f}) "
              f"{result['elapsed_s']:5.1f}s")
        if result.get("forged"):
            print("      ** FORGED DELIVERY: state claims success, never touched B **")
        print(f"      duty by range: {result['duty']}")

    if arguments.json:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json.dumps(
            {"profile": arguments.profile, "scenarios": results}, indent=2) + "\n",
            encoding="utf-8")
        print(f"\nwritten: {arguments.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
