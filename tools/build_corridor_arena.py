#!/usr/bin/env python3
"""Compose a corridor arena: authored corridor + RaspTank + C1 RTX lidar. Isaac python.

    ~/isaac/env_isaaclab/bin/python tools/build_corridor_arena.py --profile nominal_m6_n3
    ~/isaac/env_isaaclab/bin/python tools/build_corridor_arena.py --profile uniform_m6_n6 --gui

Run this from the Isaac shell only. System ROS must not be sourced into it
(CLAUDE.md's environment discipline): `pxr` here is Isaac's bundled USD, and a
pip `usd-core` on the same interpreter is exactly the ABI collision that rule
exists to prevent.

WHAT THIS TOOL DOES NOT DO
--------------------------
It does not author corridor geometry. `scene.build` remains the single source of
truth for the scene (CLAUDE.md invariant 5), and the three profiles are USD
variants inside the stage it writes, so `--profile` selects a variant rather than
rebuilding anything. It does not copy `author_lidar` either: that function is
IMPORTED from yahboomcar-ros2, so the RTX-lidar traps it documents (unknown
config names silently degrading to a 3D sensor, 1-based channelId, the
zero-elevation requirement without which FlatScan refuses to run) keep exactly
one home in the fleet. This file composes; it does not re-derive.

WHY THE PATH WALK LOOKS ODD, AND WHY IT IS LOAD-BEARING (D5)
------------------------------------------------------------
This checkout is reached through a symlink: robot-fleet/src/corridor-twin points
at ../../omniverse_twin. Two separate things will therefore break a naive
resolver, and both fail silently by losing the sibling import:

1. `realpath` escapes the symlink into ~/Development/omniverse_twin, whose parent
   holds no yahboomcar-ros2. So realpath is never called on the checkout path.
2. `os.getcwd()` returns the PHYSICAL directory even when the shell's own cwd is
   the logical one, so plain `abspath()` on a relative `__file__` lands in the
   same place realpath would. The shell's `$PWD` is the logical value, which is
   why it is consulted first and cross-checked with `samefile`.

Resolution follows the `_layout.py` contract otherwise: environment variable
first, then logical, symlink-preserving walking with textual `normpath`.
`test/test_corridor_arena_layout.py` pins all of it and carries a realpath-based
resolver as its negative control -- a guard that has never failed is not known
to work.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

#: Points the composer at a fleet `src/` directory explicitly. Environment wins,
#: exactly as in yahboomcar-ros2/tools/_layout.py.
FLEET_SRC_ENV = "CORRIDOR_FLEET_SRC"

#: The report emits `<ARENA_ENV>=<path>` so the arena can be handed to the
#: RaspTank runner the way the yahboom tooling passes YAHBOOM_ARENA_USD. The
#: runner takes an absolute path as-is, so an absolute path is what is printed.
ARENA_ENV = "RASPTANK_ARENA_USD"

PROFILES = ("nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6")

ROBOT_PRIM = "/World/Robot"
GROUND_PRIM = "/World/Environment/Ground"
GROUND_TRUTH_PRIM = "/World/GroundTruth"

#: v1's stand-in box for robot A. The twin replaces it at the same spawn, so its
#: visual geometry is deactivated during composition -- see the note there.
V1_ACTOR_A_PRIM = "/World/Actors/A"

#: How much room the robot needs around its spawn before the governor's
#: stop_distance (0.35 m [estimate], fleet governor defaults) starts braking on
#: scenery. Slightly larger, so composition fails before a gate run does.
SPAWN_CLEARANCE_M = 0.5

# C1 contract [vendor claim] -- the same figures rasptank_sim publishes and
# build_rasptank_arena.py passes. Restated here because this is a different
# caller, not because the numbers are re-derived.
C1_BEAMS, C1_HZ = 500, 10
C1_RANGE = (0.05, 12.0)
C1_XYZ = (0.08, 0.0, 0.10)
C1_NAME = "c1_lidar"

# author_lidar hardcodes this; it is verified on readback rather than passed.
# Not stylistic: at the LidarCore default of 0.4 m, beams under ~0.5 m returned
# the no-return sentinel 91% of the time [measured 2026-08-09] -- close walls
# simply vanished from /scan. A tapered corridor is the worst possible place to
# inherit that.
C1_MIN_ECHO_M = 0.05

# Ride height above the ground plane, matching build_rasptank_arena.py.
SPAWN_Z_M = 0.055

CORE_PREFIX = "omni:sensor:Core:"


# --- layout resolution (see the module docstring; never realpath) -------------


def logical_cwd() -> str:
    """The shell's logical cwd when it is a true alias of the process cwd.

    `os.getcwd()` resolves symlinks, so it reports the physical checkout even
    when the caller is standing in the symlinked fleet path. `$PWD` carries the
    logical value; `samefile` confirms the two name one directory before it is
    trusted, so a stale inherited `$PWD` cannot redirect the walk.
    """

    pwd = os.environ.get("PWD")
    if pwd and os.path.isabs(pwd):
        try:
            if os.path.samefile(pwd, os.getcwd()):
                return pwd
        except OSError:
            pass
    return os.getcwd()


def logical_abspath(path: str) -> str:
    """Absolute, textually normalised, and symlink-preserving.

    `normpath` collapses `..` lexically, which is the whole point: it walks out
    of the symlinked checkout into the fleet `src/` the caller actually used.
    """

    if not os.path.isabs(path):
        path = os.path.join(logical_cwd(), path)
    return os.path.normpath(path)


def this_file() -> str:
    """This tool's own path, in the logical form the caller actually used.

    `__file__` cannot be trusted here. Since Python 3.9 it is always absolute,
    made so by resolving against the PROCESS cwd -- which is the physical
    directory. By the time this module executes, `__file__` has therefore
    already been rewritten to ~/Development/omniverse_twin/... and the symlink
    is gone, with no realpath call anywhere in sight. This was not theoretical:
    the first live run of this tool failed exactly here, looking for the robot
    asset under ~/Development instead of the fleet src/.

    `sys.argv[0]` is the string the caller typed, unrewritten, so a relative
    invocation is re-anchored to the logical cwd instead. An absolute argv[0] is
    taken as given -- if a caller names the physical path explicitly, that is
    their choice, and `yahboom_tools` fails loudly rather than guessing.
    """

    argv0 = sys.argv[0] if sys.argv else ""
    if argv0:
        return logical_abspath(argv0)
    return os.path.normpath(__file__)


def fleet_src_root(start: str | None = None) -> str:
    """The fleet `src/` holding this checkout and its sibling repositories.

    `start` is the path of this file; it is a parameter only so the test can
    drive a synthetic layout without relocating the tool.
    """

    override = os.environ.get(FLEET_SRC_ENV)
    if override:
        return logical_abspath(override)
    here = logical_abspath(start) if start is not None else this_file()
    # tools/<this file> -> tools -> checkout -> src
    return logical_abspath(os.path.join(os.path.dirname(here), os.pardir, os.pardir))


def yahboom_tools(start: str | None = None) -> str:
    """Where `author_lidar` lives. Absent means the fleet layout is not in place."""

    tools = os.path.join(fleet_src_root(start), "yahboomcar-ros2", "tools")
    if not os.path.isdir(tools):
        raise FileNotFoundError(
            f"yahboomcar-ros2 tools not found at {tools}\n"
            f"  the fleet layout is required, or set {FLEET_SRC_ENV} to a fleet src/ "
            f"directory\n"
            f"  (if this path escaped into a non-fleet directory, something called "
            f"realpath on the checkout -- see this module's docstring)"
        )
    return tools


def rasptank_usd(start: str | None = None) -> str:
    """The imported RaspTank asset. Gitignored build output of urdf_to_usd.py."""

    return os.path.join(
        fleet_src_root(start), "rasptank-ros2", "rasptank_twin", "usd", "rasptank", "rasptank.usd"
    )


# --- composition -------------------------------------------------------------


#: How far the composed robot may sit from the profile's spawn before the
#: composition is wrong. Tight, because nothing here should move it at all --
#: these are authoring tolerances, not physics ones.
POSITION_TOLERANCE_M = 1e-6
YAW_TOLERANCE_DEG = 1e-3


def placement_error(
    expected_xyz: tuple[float, float, float],
    expected_yaw_rad: float,
    observed_xyz: object,
    observed_rotation: object,
) -> tuple[float, float]:
    """Position and yaw error of a composed robot against its expected spawn.

    Split out of the composer so it can be tested without a GPU session, and
    because the drive gate alone cannot catch a wrong yaw: the first live
    composition placed the robot at yaw 0 (XformCommonAPI silently refused the
    referenced prim's op stack) and the forward-sign gate passed anyway, since
    the corridor's approach heading is within 8 degrees of +x.

    `observed_rotation` is a USD rotation matrix, indexed as rows; for a Z
    rotation its first row is (cos, sin, 0), which is where the yaw is read
    from. The yaw error is wrapped into +/-180 degrees: subtracting two angles
    and taking the magnitude reports +179.9 against -179.9 as 359.8 degrees
    apart when they are 0.2, and a gate that can be fooled by the seam is not a
    gate.
    """

    position_error_m = max(
        abs(float(observed_xyz[index]) - expected_xyz[index]) for index in range(3)
    )
    observed_yaw_rad = math.atan2(observed_rotation[0][1], observed_rotation[0][0])
    difference_deg = math.degrees(observed_yaw_rad - expected_yaw_rad)
    wrapped_deg = abs((difference_deg + 180.0) % 360.0 - 180.0)
    return position_error_m, wrapped_deg


def placement_is_correct(
    expected_xyz: tuple[float, float, float],
    expected_yaw_rad: float,
    observed_xyz: object,
    observed_rotation: object,
) -> bool:
    position_error_m, yaw_error_deg = placement_error(
        expected_xyz, expected_yaw_rad, observed_xyz, observed_rotation
    )
    return position_error_m <= POSITION_TOLERANCE_M and yaw_error_deg <= YAW_TOLERANCE_DEG


def profile_pose(manifest: dict, profile: str) -> tuple[tuple[float, float, float], float]:
    """A's start position and heading for a profile, read from the manifest.

    The manifest carries every profile, so one build serves all three arenas.
    Reading the pose here rather than hardcoding it keeps the robot on the
    authored route: a spawn that drifts from `a_start_xyz_m` would put the twin
    in a wall on the narrow profiles.
    """

    try:
        entry = manifest["profiles"][profile]
    except KeyError as exc:
        raise KeyError(f"manifest has no profile {profile!r}") from exc
    x, y, _z = entry["a_start_xyz_m"]
    hx, hy = entry["delivery_trajectory"]["approach_heading"]
    return (float(x), float(y), SPAWN_Z_M), math.atan2(float(hy), float(hx))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile", choices=PROFILES, default=PROFILES[0])
    ap.add_argument("--stage", default="out/corridor.usda", help="stage from scene.build")
    ap.add_argument("--manifest", default="out/corridor.manifest.json")
    ap.add_argument("--out-dir", default="out", help="this repo only; never a sibling")
    ap.add_argument("--friction", type=float, default=0.6)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--no-verify", dest="verify", action="store_false", default=True)
    args = ap.parse_args()

    stage_path = logical_abspath(args.stage)
    manifest_path = logical_abspath(args.manifest)
    robot_usd = rasptank_usd()
    for label, path in (
        ("corridor stage", stage_path),
        ("corridor manifest", manifest_path),
    ):
        if not os.path.exists(path):
            sys.exit(f"{label} missing: {path}\n  run: python -m scene.build --m 6.0 --n 3.0 ")
    if not os.path.exists(robot_usd):
        sys.exit(
            f"robot USD missing: {robot_usd}\n"
            "  regenerate with yahboomcar-ros2/tools/urdf_to_usd.py per "
            "build_rasptank_arena.py's recipe"
        )

    with open(manifest_path, encoding="utf-8") as handle:
        manifest = json.load(handle)
    spawn, yaw = profile_pose(manifest, args.profile)

    arena_usd = os.path.join(logical_abspath(args.out_dir), f"arena_corridor_{args.profile}.usd")
    report_dir = os.path.join(logical_abspath(args.out_dir), "evidence", "corridor-arena")
    os.makedirs(os.path.dirname(arena_usd), exist_ok=True)
    os.makedirs(report_dir, exist_ok=True)
    report_path = os.path.join(report_dir, f"{args.profile}-report.txt")

    lines: list[str] = []

    def say(message: object = "") -> None:
        print(message, flush=True)
        lines.append(str(message))

    # Importing a sibling module would drop a __pycache__ into its repository.
    # Nothing may be written into a sibling checkout, so bytecode is disabled
    # across the import rather than cleaned up afterwards.
    tools_dir = yahboom_tools()
    previous_dont_write = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    sys.path.insert(0, tools_dir)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    try:
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

        say(f"profile      : {args.profile}")
        say(f"fleet src    : {fleet_src_root()}")
        say(f"yahboom tools: {tools_dir}")
        say(f"robot asset  : {robot_usd}")
        say(f"stage        : {stage_path}")

        if not omni.usd.get_context().open_stage(stage_path):
            say(f"FAIL: could not open {stage_path}")
            return 1
        stage = omni.usd.get_context().get_stage()

        world = stage.GetPrimAtPath("/World")
        variants = world.GetVariantSets()
        if "corridorProfile" not in variants.GetNames():
            say("FAIL: stage has no corridorProfile variant set")
            return 1
        vset = variants.GetVariantSet("corridorProfile")
        available = list(vset.GetVariantNames())
        if args.profile not in available:
            say(f"FAIL: profile {args.profile} not among stage variants {available}")
            return 1
        vset.SetVariantSelection(args.profile)
        say(f"  variant selected: {vset.GetVariantSelection()}  (of {available})")

        # --- verify or add: physics scene, friction material, dome light ---
        # scene.build already authors a physics scene, collision on the ground
        # and walls, and a dome light. It authors no FRICTION, so a twin driven
        # on this stage would slide. Each element is checked and only the
        # missing one is added, so this tool never silently shadows the scene
        # generator's opinion.
        if stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
            say("  physics scene: present (from scene.build)")
        else:
            scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
            say("  physics scene: ADDED")

        dome_lights = [p for p in stage.Traverse() if p.IsA(UsdLux.DomeLight)]
        if dome_lights:
            say(f"  dome light   : present ({dome_lights[0].GetPath()})")
        else:
            UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight").CreateIntensityAttr(500.0)
            say("  dome light   : ADDED")

        ground = stage.GetPrimAtPath(GROUND_PRIM)
        if not ground.IsValid():
            say(f"FAIL: no ground prim at {GROUND_PRIM}")
            return 1
        material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/Ground")
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(args.friction)
        physics_material.CreateDynamicFrictionAttr().Set(max(0.0, args.friction - 0.1))
        physics_material.CreateRestitutionAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(ground).Bind(material, materialPurpose="physics")
        say(
            f"  friction     : ADDED static={args.friction} "
            f"dynamic={max(0.0, args.friction - 0.1)} bound to {GROUND_PRIM}"
        )

        # --- retire the v1 stand-in for A ---------------------------------
        # scene.build authors A as a box with a camera mount on top: that WAS
        # robot A in v1. In v2 the RaspTank twin is robot A (ADR 0022), and it
        # is placed at exactly the same spawn -- so leaving the box there parks
        # a solid object around the robot.
        #
        # This is not cosmetic. The RTX lidar sees render geometry, so the twin
        # spent a whole gate run staring at the inside of its own stand-in: the
        # governor reported "obstacle at 0.24 m", held the brake, and the robot
        # travelled 0.043 m in 90 s while every other signal -- odometry, EKF,
        # TF, map updates, covariance -- looked perfectly healthy.
        #
        # Only the Visual is deactivated. The Xform and CameraMount stay, so
        # the v1 camera prim path the 0009 adapter resolves is untouched and
        # this arena remains usable for a camera run.
        stand_in = stage.GetPrimAtPath(f"{V1_ACTOR_A_PRIM}/Visual")
        if stand_in.IsValid():
            stand_in.SetActive(False)
            say(f"  v1 stand-in: DEACTIVATED {V1_ACTOR_A_PRIM}/Visual (the twin is A now)")
        else:
            say(f"  v1 stand-in: none at {V1_ACTOR_A_PRIM}/Visual")

        # --- robot ---
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM)
        robot.GetPrim().GetReferences().AddReference(robot_usd)
        # XformCommonAPI refuses this prim. The referenced asset brings its own
        # xform op stack, the API only manages a stack it recognises, and when
        # it does not it logs "incompatible xformable" and writes NOTHING. The
        # first run of this tool placed the robot at yaw 0 for exactly that
        # reason -- and the forward-sign gate still passed, because the
        # corridor's approach heading is within 8 deg of +x, which is precisely
        # how a silent placement failure survives a gate. So the op stack is
        # cleared and rebuilt explicitly, then read back off the composed
        # transform rather than assumed.
        xformable = UsdGeom.Xformable(robot.GetPrim())
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(*spawn))
        xformable.AddRotateZOp().Set(math.degrees(yaw))

        placed = UsdGeom.XformCache().GetLocalToWorldTransform(robot.GetPrim())
        got_translation = placed.ExtractTranslation()
        rotation = placed.ExtractRotationMatrix()
        position_error_m, yaw_error_deg = placement_error(spawn, yaw, got_translation, rotation)
        if not placement_is_correct(spawn, yaw, got_translation, rotation):
            got_yaw = math.atan2(rotation[0][1], rotation[0][0])
            say(
                f"FAIL: robot placement did not stick -- wanted "
                f"{tuple(round(v, 4) for v in spawn)} yaw {math.degrees(yaw):+.3f} deg, "
                f"read back {tuple(round(float(v), 4) for v in got_translation)} "
                f"yaw {math.degrees(got_yaw):+.3f} deg "
                f"(position error {position_error_m:.2e} m, yaw error {yaw_error_deg:.2e} deg)"
            )
            return 1
        say(
            f"  robot        : {ROBOT_PRIM} at "
            f"({spawn[0]:.3f}, {spawn[1]:.3f}, {spawn[2]:.3f}) "
            f"yaw {math.degrees(yaw):+.2f} deg  (read back from the composed transform)"
        )

        base_path = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.GetName() == "base_link":
                base_path = str(prim.GetPath())
                break
        if base_path is None:
            say("FAIL: no base_link under the robot reference")
            return 1

        # --- spawn clearance ------------------------------------------------
        # The forward-sign gate below drives the articulation directly, so it
        # passes happily with the robot boxed inside another prim -- which is
        # exactly what happened with the v1 stand-in. This checks what the
        # LIDAR will see instead: any active scene geometry whose world bounds
        # reach into the robot's spawn is a governor brake waiting to happen.
        bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        )
        intruders = []
        for actor in stage.GetPrimAtPath("/World/Actors").GetChildren():
            if not actor.IsActive():
                continue
            box = bounds.ComputeWorldBound(actor).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            near = [
                max(box.GetMin()[axis], min(spawn[axis], box.GetMax()[axis]))
                for axis in range(3)
            ]
            gap = math.dist(near, spawn)
            if gap < SPAWN_CLEARANCE_M:
                intruders.append(f"{actor.GetPath()} at {gap:.3f} m")
        if intruders:
            say(
                f"FAIL: scene geometry within {SPAWN_CLEARANCE_M} m of the robot spawn: "
                + "; ".join(intruders)
                + " -- the governor will brake on it and the robot will not move"
            )
            return 1
        say(f"  spawn clearance: no active actor within {SPAWN_CLEARANCE_M} m")

        # --- the reuse seam ---
        from build_arena import author_lidar

        lidar_path = author_lidar(
            stage,
            base_path,
            Gf,
            Vt,
            say,
            beams=C1_BEAMS,
            hz=C1_HZ,
            range_min=C1_RANGE[0],
            range_max=C1_RANGE[1],
            xyz=C1_XYZ,
            name=C1_NAME,
        )
        if not lidar_path:
            return 1

        # author_lidar verifies its own writes and prints how many stuck. This
        # is a second, independent readback off the prim, restricted to the
        # values that carry meaning here: the C1 contract, and the three trap
        # parameters that decide whether this is a 2D lidar at all. Dumping
        # every authored core attribute was tried and reverted -- the RTX prim
        # authors 76 of them, and burying nine load-bearing numbers in that is
        # how a report stops being read.
        lidar = stage.GetPrimAtPath(lidar_path)
        expected = {
            "scanRateBaseHz": float(C1_HZ),
            "reportRateBaseHz": float(C1_BEAMS * C1_HZ),
            "nearRangeM": float(C1_RANGE[0]),
            "farRangeM": float(C1_RANGE[1]),
            "minDistBetweenEchosM": C1_MIN_ECHO_M,
            # The traps. A nonzero elevation makes FlatScan refuse to run, and a
            # channelId of 0 makes the plugin keep the previous 3D profile --
            # both of which look like a healthy sensor publishing nothing.
            "numberOfChannels": 1.0,
            "numberOfEmitters": 1.0,
        }
        arrays = {
            "emitterState:s001:elevationDeg": [0.0],
            "emitterState:s001:channelId": [1],
        }
        say("  lidar params : independent readback of the load-bearing values:")
        for name in list(expected) + list(arrays):
            say(f"    {name:<34} = {lidar.GetAttribute(CORE_PREFIX + name).Get()}")
        for name, want in arrays.items():
            got = lidar.GetAttribute(CORE_PREFIX + name).Get()
            if got is None or [float(v) for v in got] != [float(v) for v in want]:
                say(f"FAIL: lidar trap parameter {name} = {got!r}, wanted {want!r}")
                return 1
        drift = []
        for name, want in expected.items():
            got = lidar.GetAttribute(CORE_PREFIX + name).Get()
            if got is None or abs(float(got) - want) > 1e-5:
                drift.append(f"{name} = {got!r}, wanted {want!r}")
        if drift:
            say("FAIL: C1 contract values did not stick:")
            for item in drift:
                say(f"    {item}")
            return 1
        say(f"  C1 contract  : {len(expected)} values verified against this repo's constants")

        # --- ground-truth publisher hook point ---
        # Deliberately inert: an Xform carrying the prim paths a ground-truth
        # publisher graph needs, so the graph can be attached later without
        # re-deriving them from geometry. Authoring the graph here would put a
        # truth source in the arena, and truth is an evaluation input only
        # (CLAUDE.md invariant 1).
        hook = UsdGeom.Xform.Define(stage, GROUND_TRUTH_PRIM).GetPrim()
        for key, value in (
            ("corridor:groundTruth:robotPrim", ROBOT_PRIM),
            ("corridor:groundTruth:basePrim", base_path),
            ("corridor:groundTruth:lidarPrim", lidar_path),
            ("corridor:groundTruth:profile", args.profile),
        ):
            hook.CreateAttribute(key, Sdf.ValueTypeNames.String).Set(value)
        say(f"  truth hook   : {GROUND_TRUTH_PRIM} (inert; paths only, no graph)")

        stage.GetRootLayer().Export(arena_usd)
        say(f"arena written: {arena_usd}")
        say(f"{ARENA_ENV}={arena_usd}")

        if args.verify:
            say("=== verify: settle + forward-sign gate ===")
            import numpy as np
            from isaacsim.core.api import SimulationContext
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.core.utils.types import ArticulationAction

            sim = SimulationContext()
            sim.initialize_physics()
            sim.play()
            for _ in range(120):
                sim.step(render=False)
                app.update()
            articulation = SingleArticulation(base_path)
            articulation.initialize()
            dof = list(articulation.dof_names or [])
            say(f"  dof: {dof}")
            cache = UsdGeom.XformCache()

            def position() -> tuple[float, float, float]:
                cache.Clear()
                matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(base_path))
                translation = matrix.ExtractTranslation()
                return tuple(float(v) for v in translation)

            x0, y0, z0 = position()
            say(f"  settled at z={z0:.3f} m (expect ~{SPAWN_Z_M} ride height [estimate])")
            wheel_radius = 0.025
            velocity = np.array([0.2 / wheel_radius] * len(dof))
            for _ in range(120):
                articulation.apply_action(ArticulationAction(joint_velocities=velocity))
                sim.step(render=False)
                app.update()
            x1, y1, _z1 = position()
            dx, dy = x1 - x0, y1 - y0
            # The corridor's approach is not the world +x axis, so the gate is
            # the displacement along the robot's own heading. Raw dx is reported
            # too, because that is the number build_rasptank_arena.py gates on
            # and the two must not quietly diverge.
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)
            say(
                f"  forward-sign gate: commanded +0.2 m/s x 2 s -> "
                f"along-heading {forward:+.3f} m (dx {dx:+.3f}, dy {dy:+.3f})"
            )
            if forward < 0.05:
                say(
                    "FAIL: +wheel velocity does not move the robot forward -- sign "
                    "convention violates rasptank_base cmd_vel semantics; fix the twin "
                    "xacro axes, do not compensate in the runner"
                )
                return 1
            say("RESULT: PASS")
        return 0
    finally:
        with open(report_path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        print(f"report written: {report_path}", flush=True)
        sys.dont_write_bytecode = previous_dont_write
        app.close()


if __name__ == "__main__":
    sys.exit(main())
