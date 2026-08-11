#!/usr/bin/env python3
"""Publish the corridor camera and simulation clock with installed Isaac Sim 5.1."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

from isaac_gpu import gpu_memory_snapshot
from renderer_contract import RenderState, is_path_tracing, render_state_violations
from viewpoints import CHASE_VIEW, VIEW_NAMES, chase_pose, format_vec3, parse_vec3, resolve

_ENV_MARKER = "CORRIDOR_ISAAC_ROS_ENV"


def _bootstrap_bundled_jazzy() -> None:
    """Re-exec once with Isaac's Python and bundled Jazzy libraries isolated."""
    if os.environ.get(_ENV_MARKER) == "1":
        return
    bridge_root = (
        Path(sys.prefix)
        / "lib"
        / f"python{sys.version_info.major}.{sys.version_info.minor}"
        / "site-packages"
        / "isaacsim"
        / "exts"
        / "isaacsim.ros2.bridge"
    )
    bridge_lib = bridge_root / "jazzy" / "lib"
    if not bridge_lib.is_dir():
        raise RuntimeError(f"Isaac bundled Jazzy library directory not found: {bridge_lib}")
    environment = os.environ.copy()
    previous_library_path = environment.get("LD_LIBRARY_PATH", "")
    environment.update(
        {
            _ENV_MARKER: "1",
            "ROS_DISTRO": "jazzy",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "PYTHONNOUSERSITE": "1",
            "PYTHONPATH": "",
            "LD_LIBRARY_PATH": (
                f"{bridge_lib}:{previous_library_path}"
                if previous_library_path
                else str(bridge_lib)
            ),
        }
    )
    environment.pop("OLD_PYTHONPATH", None)
    environment.pop("AMENT_PREFIX_PATH", None)
    os.execve(sys.executable, [sys.executable, *sys.argv], environment)


_bootstrap_bundled_jazzy()

# SimulationApp must start before omni, pxr, usdrt, or Isaac extensions are imported.
from isaacsim import SimulationApp  # noqa: E402

# This literal is inspected by ordinary pytest without importing Isaac Sim.
ADAPTER_CONTRACT = {
    "camera_prim": "/World/Actors/A/CameraMount/FrontCamera",
    "image_topic": "/p_cam/image_raw",
    "camera_info_topic": "/p_cam/camera_info",
    "clock_topic": "/clock",
    "frame_id": "p_cam_optical_frame",
    "width": 640,
    "height": 360,
    "simulation_hz": 60,
    "camera_hz": 15,
    "render_products": 1,
    "render_mode": "RaytracedLighting",
    "anti_aliasing": 3,
}

ROBOT_PRIM = "/World/Actors/A"
RENDER_PRODUCT_WARMUP_UPDATES = 12

# Kit's own viewport camera. Moving it is a transform write on a prim the GUI
# already owns: it creates no prim, no sensor and no render product, so the
# one-camera budget enforced in _validate_stage_and_graph is untouched.
VIEWPORT_CAMERA_PRIM = "/OmniverseKit_Persp"

# Chase updates every Nth app update rather than every one. Each call writes USD
# attributes and syncs the viewport, and the delivered camera rate already sags
# under host contention; at 60 Hz simulation this is 15 Hz of camera motion for
# a quarter of the cost.
CHASE_UPDATE_INTERVAL = 4

# Settings keys that hold the renderer state actually in force. The installed
# SimulationApp._set_render_settings(default=...) writes "/rtx-defaults" when
# called with default=True and "/rtx" otherwise, and reset_render_settings()
# takes the default=False path, so the active tree is the acceptance value and
# the defaults tree is lifecycle evidence.
ACTIVE_RENDER_MODE_KEY = "/rtx/rendermode"
DEFAULT_RENDER_MODE_KEY = "/rtx-defaults/rendermode"
ACTIVE_ANTI_ALIASING_KEY = "/rtx/post/aa/op"
DEFAULT_ANTI_ALIASING_KEY = "/rtx-defaults/post/aa/op"


def _read_render_state(settings) -> RenderState:
    """Read the renderer state in force. Never derived from the request.

    This is the only part of the renderer check that needs a running Kit
    application. The acceptance policy lives in ``renderer_contract`` so its
    rejection branches stay testable without a GPU.
    """

    return RenderState(
        active_render_mode=settings.get_as_string(ACTIVE_RENDER_MODE_KEY),
        default_render_mode=settings.get_as_string(DEFAULT_RENDER_MODE_KEY),
        active_anti_aliasing=settings.get_as_int(ACTIVE_ANTI_ALIASING_KEY),
        default_anti_aliasing=settings.get_as_int(DEFAULT_ANTI_ALIASING_KEY),
    )


def _render_state_violations(state: RenderState) -> list[str]:
    """Apply the portable acceptance policy to one readback."""

    return render_state_violations(
        state,
        expected_render_mode=ADAPTER_CONTRACT["render_mode"],
        expected_anti_aliasing=ADAPTER_CONTRACT["anti_aliasing"],
        active_render_mode_key=ACTIVE_RENDER_MODE_KEY,
    )


STATIC_PROBE_DEFAULTS = {
    "stations_x_m": (0.5, 1.5, 3.0, 5.0, 7.0),
    "settle_updates": 12,
    "capture_updates": 36,
    "output": "out/evidence/static-fiducials/static-truth.json",
}

NODE_TYPES = {
    "tick": "omni.graph.action.OnPlaybackTick",
    "run_one_frame": "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
    "render_product": "isaacsim.core.nodes.IsaacCreateRenderProduct",
    "ros_context": "isaacsim.ros2.bridge.ROS2Context",
    "camera_qos": "isaacsim.ros2.bridge.ROS2QoSProfile",
    "clock_qos": "isaacsim.ros2.bridge.ROS2QoSProfile",
    "rgb": "isaacsim.ros2.bridge.ROS2CameraHelper",
    "camera_info": "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
    "simulation_time": "isaacsim.core.nodes.IsaacReadSimulationTime",
    "clock": "isaacsim.ros2.bridge.ROS2PublishClock",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", type=Path)
    parser.add_argument("--updates", type=int, default=900)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--profile")
    parser.add_argument(
        "--static-probe-out",
        type=Path,
        help="Hold A at fixed approach stations and write evaluator-only pose intervals.",
    )
    parser.add_argument(
        "--static-stations-x",
        type=float,
        nargs="+",
        default=list(STATIC_PROBE_DEFAULTS["stations_x_m"]),
        metavar="X_M",
    )
    parser.add_argument(
        "--static-settle-updates",
        type=int,
        default=STATIC_PROBE_DEFAULTS["settle_updates"],
    )
    parser.add_argument(
        "--static-capture-updates",
        type=int,
        default=STATIC_PROBE_DEFAULTS["capture_updates"],
    )
    parser.add_argument(
        "--drive-speed-mps",
        type=float,
        help=(
            "Drive A continuously along the delivery trajectory at this path speed. "
            "Mutually exclusive with --static-probe-out."
        ),
    )
    parser.add_argument(
        "--drive-out",
        type=Path,
        help="Write the commanded pose schedule. Simulator truth: evaluator input only.",
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--view",
        choices=VIEW_NAMES,
        default="rviz",
        help=(
            "Viewport perspective. Ignored without --gui. 'rviz' matches the "
            "angle the RViz config shows; 'chase' follows A along the route."
        ),
    )
    parser.add_argument(
        "--view-eye",
        type=parse_vec3,
        metavar="X,Y,Z",
        help="Explicit viewport camera position in world metres; overrides --view.",
    )
    parser.add_argument(
        "--view-target",
        type=parse_vec3,
        metavar="X,Y,Z",
        help="Explicit viewport look-at point in world metres; overrides --view.",
    )
    parser.add_argument("--report-gpu-memory", action="store_true")
    return parser.parse_args()


def _validate_environment() -> None:
    if os.environ.get(_ENV_MARKER) != "1":
        raise RuntimeError("Isaac ROS environment bootstrap did not complete")
    if os.environ.get("ROS_DISTRO") != "jazzy":
        raise RuntimeError("bundled ROS_DISTRO must be jazzy")
    middleware = os.environ.get("RMW_IMPLEMENTATION")
    if middleware not in {None, "rmw_fastrtps_cpp"}:
        raise RuntimeError(
            "Isaac validation expects the default Fast DDS middleware; "
            f"found RMW_IMPLEMENTATION={middleware!r}"
        )


def _select_profile(stage, requested: str | None) -> str:
    world = stage.GetPrimAtPath("/World")
    variants = world.GetVariantSets().GetVariantSet("corridorProfile")
    if not variants or not variants.GetVariantNames():
        raise RuntimeError("missing /World corridorProfile variant set")
    selected = requested or variants.GetVariantSelection()
    if selected not in variants.GetVariantNames():
        raise ValueError(f"unknown corridor profile {selected!r}")
    if not variants.SetVariantSelection(selected):
        raise RuntimeError(f"failed to select corridor profile {selected!r}")
    return selected


def _set_actor_pose(stage, pose) -> None:
    from pxr import Gf, UsdGeom

    actor = stage.GetPrimAtPath(ROBOT_PRIM)
    if not actor:
        raise RuntimeError(f"missing robot prim {ROBOT_PRIM}")
    operations = UsdGeom.Xformable(actor).GetOrderedXformOps()
    translate = next(
        (
            operation
            for operation in operations
            if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ),
        None,
    )
    rotate_z = next(
        (
            operation
            for operation in operations
            if operation.GetOpType() == UsdGeom.XformOp.TypeRotateZ
        ),
        None,
    )
    if translate is None or rotate_z is None:
        raise RuntimeError("robot prim must carry translate and rotateZ xform ops")
    translate.Set(Gf.Vec3d(pose.x_m, pose.y_m, pose.z_m))
    rotate_z.Set(math.degrees(pose.yaw_rad))


def _manifest_path(args: argparse.Namespace, stage_path: Path) -> Path:
    path = args.manifest or stage_path.with_suffix(".manifest.json")
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


def _run_drive(
    app,
    stage,
    args: argparse.Namespace,
    stage_path: Path,
    profile: str,
) -> int:
    """Drive A continuously along the authored route from simulation time.

    This is the demonstration mode. It reuses the same pieces the static probe
    already exercises on the GPU -- the manifest trajectory, ``_set_actor_pose``
    and ``app.update()`` -- and differs only in deriving the route station from
    the simulation clock instead of stepping through a list of dwells.

    Simulation time, never wall time, sets the station. Wall time may describe
    how fast the app happens to run, but making it the motion input would
    couple the demonstrated speed to host load, and the observer differentiates
    message stamps that come from this same clock.

    Whether the pose written here is composed into the frame that
    ``app.update()`` renders, or into the one after it, is not measured. No
    offset is applied to compensate for an unmeasured latency; that measurement
    is its own piece of work.
    """

    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes

    scene_source = Path(__file__).resolve().parents[1] / "src/corridor_scene"
    if str(scene_source) not in sys.path:
        sys.path.insert(0, str(scene_source))
    from scene.trajectory import trajectory_from_manifest

    manifest_path = _manifest_path(args, stage_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if profile not in manifest["profiles"]:
        raise ValueError(f"manifest does not contain selected profile {profile!r}")
    trajectory = trajectory_from_manifest(manifest["profiles"][profile]["delivery_trajectory"])

    speed_mps = float(args.drive_speed_mps)
    if speed_mps <= 0.0:
        raise ValueError("--drive-speed-mps must be positive")
    route_length_m = trajectory.length_m

    clock = _isaacsim_core_nodes.acquire_interface()
    epoch_s = float(clock.get_sim_time())
    # Without a viewport there is nothing to chase, and the per-update cost
    # would be paid for a camera nobody is looking through.
    chase_viewport = args.gui and args.view == CHASE_VIEW
    samples: list[dict[str, object]] = []
    completed = 0
    route_s_m = 0.0
    # --updates is a safety cap so a stalled clock cannot spin forever; the
    # route end is the intended stopping condition.
    for index in range(args.updates):
        sim_time_s = float(clock.get_sim_time())
        route_s_m = min(speed_mps * (sim_time_s - epoch_s), route_length_m)
        pose = trajectory.pose_at(route_s_m)
        _set_actor_pose(stage, pose)
        if chase_viewport and index % CHASE_UPDATE_INTERVAL == 0:
            eye, target = chase_pose(pose.x_m, pose.y_m, pose.yaw_rad)
            _set_viewport_camera(eye, target)
        app.update()
        completed = index + 1
        samples.append(
            {
                "update_index": index,
                "sim_time_s": sim_time_s,
                "route_s_m": route_s_m,
                "x_m": pose.x_m,
                "y_m": pose.y_m,
                "yaw_rad": pose.yaw_rad,
            }
        )
        if route_s_m >= route_length_m:
            break

    print(
        "ISAAC_ROS_CAMERA_DRIVE",
        f"speed_mps={speed_mps}",
        f"route_s_m={route_s_m:.3f}",
        f"route_length_m={route_length_m:.3f}",
        f"reached_end={route_s_m >= route_length_m}",
        f"updates={completed}",
        f"sim_span_s={samples[-1]['sim_time_s'] - epoch_s:.3f}" if samples else "sim_span_s=0",
        flush=True,
    )
    if args.drive_out is not None:
        output_path = args.drive_out.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(
                {
                    "schema_version": "1.0",
                    "kind": "evaluator_only_commanded_pose_schedule",
                    "stage": str(stage_path),
                    "manifest": str(manifest_path),
                    "profile": profile,
                    "robot_prim": ROBOT_PRIM,
                    "path_speed_mps": speed_mps,
                    "route_length_m": route_length_m,
                    "sim_time_epoch_s": epoch_s,
                    "pose_update_order": "set_actor_pose_then_app_update; latency unmeasured",
                    "samples": samples,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print("ISAAC_DRIVE_SCHEDULE_WRITTEN", f"path={output_path}", flush=True)
    return completed


def _run_static_probe(
    app,
    stage,
    args: argparse.Namespace,
    stage_path: Path,
    profile: str,
    render_warmup_reset_events: int,
) -> int:
    """Hold A at fixed poses while the existing graph publishes production pixels."""

    # Installed reference: isaacsim.core.nodes/tests/test_core_nodes.py imports
    # this binding and calls acquire_interface().get_sim_time() around timeline updates.
    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes

    scene_source = Path(__file__).resolve().parents[1] / "src/corridor_scene"
    if str(scene_source) not in sys.path:
        sys.path.insert(0, str(scene_source))
    from scene.trajectory import trajectory_from_manifest

    manifest_path = _manifest_path(args, stage_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if profile not in manifest["profiles"]:
        raise ValueError(f"manifest does not contain selected profile {profile!r}")
    trajectory = trajectory_from_manifest(
        manifest["profiles"][profile]["delivery_trajectory"]
    )
    if args.static_settle_updates < 1 or args.static_capture_updates < 1:
        raise ValueError("static settle and capture update counts must be positive")
    stations_x_m = tuple(float(value) for value in args.static_stations_x)
    if not stations_x_m:
        raise ValueError("at least one static world-X station is required")
    if any(
        later <= earlier
        for earlier, later in zip(stations_x_m, stations_x_m[1:], strict=False)
    ):
        raise ValueError("static world-X stations must be strictly increasing")

    import carb

    settings = carb.settings.get_settings()
    clock = _isaacsim_core_nodes.acquire_interface()
    dwells: list[dict[str, object]] = []
    observed_active_anti_aliasing: set[int] = set()
    observed_default_anti_aliasing: set[int] = set()
    observed_active_render_modes: set[str] = set()
    observed_default_render_modes: set[str] = set()
    total_updates = 0
    for index, station_x_m in enumerate(stations_x_m):
        route_s_m = trajectory.approach_s_at_x(station_x_m)
        actor_pose = trajectory.pose_at(route_s_m)
        camera_pose = trajectory.camera_pose_at(route_s_m)
        _set_actor_pose(stage, actor_pose)
        start_s = float(clock.get_sim_time())
        for _ in range(args.static_settle_updates):
            app.update()
        render_state = _read_render_state(settings)
        observed_active_anti_aliasing.add(render_state.active_anti_aliasing)
        observed_default_anti_aliasing.add(render_state.default_anti_aliasing)
        observed_active_render_modes.add(str(render_state.active_render_mode))
        observed_default_render_modes.add(str(render_state.default_render_mode))
        violations = _render_state_violations(render_state)
        if violations:
            raise RuntimeError(
                f"renderer contract violated before dwell {index}: " + "; ".join(violations)
            )
        settled_start_s = float(clock.get_sim_time())
        for _ in range(args.static_capture_updates):
            app.update()
        end_s = float(clock.get_sim_time())
        total_updates += args.static_settle_updates + args.static_capture_updates
        if not start_s <= settled_start_s < end_s:
            raise RuntimeError(
                f"simulation time did not advance during static dwell {index}: "
                f"{start_s}, {settled_start_s}, {end_s}"
            )
        dwells.append(
            {
                "index": index,
                "required_for_estimation": True,
                "route_s_m": route_s_m,
                "expected_station_x_m": camera_pose.x_m,
                "actor_pose": {
                    "x_m": actor_pose.x_m,
                    "y_m": actor_pose.y_m,
                    "z_m": actor_pose.z_m,
                    "yaw_rad": actor_pose.yaw_rad,
                },
                "camera_pose": {
                    "x_m": camera_pose.x_m,
                    "y_m": camera_pose.y_m,
                    "z_m": camera_pose.z_m,
                    "yaw_rad": camera_pose.yaw_rad,
                },
                "sim_start_s": start_s,
                "settled_start_s": settled_start_s,
                "sim_end_s": end_s,
            }
        )
        print(
            "ISAAC_STATIC_DWELL",
            f"index={index}",
            f"route_s_m={route_s_m:.6f}",
            f"expected_station_x_m={camera_pose.x_m:.6f}",
            f"settled_start_s={settled_start_s:.6f}",
            f"end_s={end_s:.6f}",
            flush=True,
        )

    output_path = args.static_probe_out.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "kind": "evaluator_only_static_pose_schedule",
                "stage": str(stage_path),
                "manifest": str(manifest_path),
                "profile": profile,
                "robot_prim": ROBOT_PRIM,
                "camera_prim": ADAPTER_CONTRACT["camera_prim"],
                "simulation_hz": ADAPTER_CONTRACT["simulation_hz"],
                "camera_hz": ADAPTER_CONTRACT["camera_hz"],
                "render_product_warmup_updates": RENDER_PRODUCT_WARMUP_UPDATES,
                "render_warmup_reset_events": render_warmup_reset_events,
                # Every "observed_" field below is a readback. The requested
                # values are recorded separately and under names that cannot be
                # mistaken for measurements.
                "render_settings": {
                    "requested_render_mode": ADAPTER_CONTRACT["render_mode"],
                    "requested_anti_aliasing": ADAPTER_CONTRACT["anti_aliasing"],
                    "observed_active_render_modes": sorted(observed_active_render_modes),
                    "observed_default_render_modes": sorted(observed_default_render_modes),
                    "observed_active_anti_aliasing": sorted(
                        observed_active_anti_aliasing
                    ),
                    "observed_default_anti_aliasing": sorted(
                        observed_default_anti_aliasing
                    ),
                    "path_tracing": any(
                        is_path_tracing(mode) for mode in observed_active_render_modes
                    ),
                },
                "settle_updates": args.static_settle_updates,
                "capture_updates": args.static_capture_updates,
                "dwells": dwells,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        "ISAAC_STATIC_SCHEDULE_WRITTEN",
        f"path={output_path}",
        f"dwells={len(dwells)}",
        f"updates={total_updates}",
        flush=True,
    )
    return total_updates


def _configure_camera_model() -> None:
    """Apply the installed 5.1 OpenCV pinhole schema for CameraInfo."""
    from isaacsim.sensors.camera import Camera

    contract = ADAPTER_CONTRACT
    camera = Camera(
        prim_path=contract["camera_prim"],
        name="corridor_front_camera",
        resolution=(contract["width"], contract["height"]),
    )
    width, height = camera.get_resolution()
    focal_length = camera.get_focal_length()
    fx = width * focal_length / camera.get_horizontal_aperture()
    fy = height * focal_length / camera.get_vertical_aperture()
    camera.set_opencv_pinhole_properties(
        cx=width / 2.0,
        cy=height / 2.0,
        fx=fx,
        fy=fy,
        pinhole=[0.0] * 12,
    )


def _set_viewport_camera(eye, target) -> bool:
    """Point Kit's viewport camera at the scene. Returns whether it took effect.

    Verified against the installed
    ``isaacsim/exts/isaacsim.core.utils/isaacsim/core/utils/viewports.py``, not
    reconstructed: ``set_camera_view`` imports ``omni.kit.viewport.utility``
    lazily and returns after a warning when no viewport exists, which is the
    headless case. The deprecated ``omni.isaac.core.utils`` copy in the same
    tree is deliberately not used.

    This never touches the sensor. It writes ``/OmniverseKit_Persp``; the ROS
    camera remains ADAPTER_CONTRACT["camera_prim"].
    """

    from isaacsim.core.utils.prims import is_prim_path_valid
    from isaacsim.core.utils.viewports import set_camera_view

    if not is_prim_path_valid(VIEWPORT_CAMERA_PRIM):
        return False
    set_camera_view(
        eye=eye,
        target=target,
        camera_prim_path=VIEWPORT_CAMERA_PRIM,
    )
    return True


def _create_graph() -> None:
    import omni.graph.core as og
    import usdrt.Sdf

    contract = ADAPTER_CONTRACT
    camera_skip_count = contract["simulation_hz"] // contract["camera_hz"] - 1
    namespace, image_name = contract["image_topic"].rsplit("/", maxsplit=1)
    _, info_name = contract["camera_info_topic"].rsplit("/", maxsplit=1)
    clock_name = contract["clock_topic"].lstrip("/")
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS_CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                (name, node_type) for name, node_type in NODE_TYPES.items()
            ],
            keys.SET_VALUES: [
                (
                    "render_product.inputs:cameraPrim",
                    [usdrt.Sdf.Path(contract["camera_prim"])],
                ),
                ("render_product.inputs:width", contract["width"]),
                ("render_product.inputs:height", contract["height"]),
                ("camera_qos.inputs:createProfile", "Custom"),
                ("camera_qos.inputs:history", "keepLast"),
                ("camera_qos.inputs:depth", 5),
                ("camera_qos.inputs:reliability", "bestEffort"),
                ("camera_qos.inputs:durability", "volatile"),
                ("clock_qos.inputs:createProfile", "Custom"),
                ("clock_qos.inputs:history", "keepLast"),
                ("clock_qos.inputs:depth", 1),
                ("clock_qos.inputs:reliability", "bestEffort"),
                ("clock_qos.inputs:durability", "volatile"),
                ("rgb.inputs:nodeNamespace", namespace),
                ("rgb.inputs:topicName", image_name),
                ("rgb.inputs:frameId", contract["frame_id"]),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:frameSkipCount", camera_skip_count),
                ("rgb.inputs:resetSimulationTimeOnStop", True),
                ("rgb.inputs:useSystemTime", False),
                ("camera_info.inputs:nodeNamespace", namespace),
                ("camera_info.inputs:topicName", info_name),
                ("camera_info.inputs:frameId", contract["frame_id"]),
                ("camera_info.inputs:frameSkipCount", camera_skip_count),
                ("camera_info.inputs:resetSimulationTimeOnStop", True),
                ("camera_info.inputs:useSystemTime", False),
                ("simulation_time.inputs:resetOnStop", True),
                ("clock.inputs:topicName", clock_name),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "run_one_frame.inputs:execIn"),
                ("run_one_frame.outputs:step", "render_product.inputs:execIn"),
                ("render_product.outputs:execOut", "rgb.inputs:execIn"),
                ("render_product.outputs:execOut", "camera_info.inputs:execIn"),
                ("render_product.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                (
                    "render_product.outputs:renderProductPath",
                    "camera_info.inputs:renderProductPath",
                ),
                ("ros_context.outputs:context", "rgb.inputs:context"),
                ("ros_context.outputs:context", "camera_info.inputs:context"),
                ("camera_qos.outputs:qosProfile", "rgb.inputs:qosProfile"),
                ("camera_qos.outputs:qosProfile", "camera_info.inputs:qosProfile"),
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("simulation_time.outputs:simulationTime", "clock.inputs:timeStamp"),
                ("ros_context.outputs:context", "clock.inputs:context"),
                ("clock_qos.outputs:qosProfile", "clock.inputs:qosProfile"),
            ],
        },
    )


def _validate_stage_and_graph() -> None:
    import omni.graph.core as og
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac did not open the corridor stage")
    camera = stage.GetPrimAtPath(ADAPTER_CONTRACT["camera_prim"])
    if not camera or not camera.IsA(UsdGeom.Camera):
        raise RuntimeError(f"missing camera prim {ADAPTER_CONTRACT['camera_prim']}")
    graph = og.get_graph_by_path("/World/ROS_CameraGraph")
    if graph is None:
        raise RuntimeError("camera graph was not created")
    actual_types = sorted(node.get_type_name() for node in graph.get_nodes())
    expected_types = sorted(NODE_TYPES.values())
    if actual_types != expected_types:
        raise RuntimeError(f"unexpected graph node types: {actual_types}")
    render_products = sum(
        node_type == "isaacsim.core.nodes.IsaacCreateRenderProduct"
        for node_type in actual_types
    )
    if render_products != ADAPTER_CONTRACT["render_products"]:
        raise RuntimeError(f"expected one render product; found {render_products}")


def main() -> int:
    args = arguments()
    stage_path = args.stage.resolve()
    if not stage_path.is_file():
        raise FileNotFoundError(stage_path)
    if args.updates < 1:
        raise ValueError("--updates must be positive")
    if args.static_probe_out is not None and args.drive_speed_mps is not None:
        raise ValueError("--static-probe-out and --drive-speed-mps are mutually exclusive")
    if args.drive_speed_mps is not None and args.drive_speed_mps <= 0.0:
        raise ValueError("--drive-speed-mps must be positive")
    if args.drive_out is not None and args.drive_speed_mps is None:
        raise ValueError("--drive-out requires --drive-speed-mps")
    # Reject a bad viewpoint before paying for a GPU app start, not after.
    static_view = resolve(args.view, args.view_eye, args.view_target)
    if args.view == CHASE_VIEW and args.drive_speed_mps is None:
        raise ValueError("--view chase requires --drive-speed-mps; there is nothing to follow")
    # Fail on a missing manifest before paying for a GPU app start.
    if args.static_probe_out is not None or args.drive_speed_mps is not None:
        _manifest_path(args, stage_path)
    _validate_environment()
    sys.argv = [sys.argv[0]]
    print(
        "ISAAC_ROS_ENV",
        "distro=jazzy",
        "middleware=rmw_fastrtps_cpp",
        "python_path=isolated",
        flush=True,
    )
    print("ISAAC_ROS_CAMERA_START", f"stage={stage_path}", flush=True)
    app = SimulationApp(
        {
            "headless": not args.gui,
            "width": ADAPTER_CONTRACT["width"],
            "height": ADAPTER_CONTRACT["height"],
            "renderer": ADAPTER_CONTRACT["render_mode"],
            "anti_aliasing": ADAPTER_CONTRACT["anti_aliasing"],
            "create_new_stage": False,
            "disable_viewport_updates": False,
            "fast_shutdown": True,
            "multi_gpu": False,
            "open_usd": str(stage_path),
            "extra_args": [
                "--enable",
                "isaacsim.ros2.bridge",
                "--/app/player/useFixedTimeStepping=true",
                "--/renderer/multiGpu/enabled=false",
            ],
        }
    )
    try:
        import carb
        import omni.timeline
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        stage.SetTimeCodesPerSecond(ADAPTER_CONTRACT["simulation_hz"])
        profile = _select_profile(stage, args.profile)
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(ADAPTER_CONTRACT["simulation_hz"])
        settings = carb.settings.get_settings()
        settings.set_bool("/app/player/useFixedTimeStepping", True)
        _configure_camera_model()
        _create_graph()
        _validate_stage_and_graph()
        # The viewpoint is GUI-only decoration. Reporting which one a run used
        # keeps a recorded run self-describing, and reporting that it was not
        # applied stops a headless log implying a viewport nobody saw.
        if not args.gui:
            print("ISAAC_ROS_CAMERA_VIEW", "view=none", "reason=headless", flush=True)
        elif static_view is not None:
            eye, target = static_view
            applied = _set_viewport_camera(eye, target)
            print(
                "ISAAC_ROS_CAMERA_VIEW",
                f"view={args.view}",
                f"eye={format_vec3(eye)}",
                f"target={format_vec3(target)}",
                f"applied={applied}",
                flush=True,
            )
        else:
            print(
                "ISAAC_ROS_CAMERA_VIEW",
                f"view={args.view}",
                f"update_interval={CHASE_UPDATE_INTERVAL}",
                flush=True,
            )
        print(
            "ISAAC_ROS_CAMERA_GRAPH_READY",
            f"image={ADAPTER_CONTRACT['image_topic']}",
            f"camera_info={ADAPTER_CONTRACT['camera_info_topic']}",
            f"clock={ADAPTER_CONTRACT['clock_topic']}",
            f"resolution={ADAPTER_CONTRACT['width']}x{ADAPTER_CONTRACT['height']}",
            f"rate_hz={ADAPTER_CONTRACT['camera_hz']}",
            f"anti_aliasing={ADAPTER_CONTRACT['anti_aliasing']}",
            "render_products=1",
            flush=True,
        )
        timeline.play()
        # IsaacCreateRenderProduct constructs its Hydra product during the
        # first playback updates. Discard those renders and verify the active
        # mode after every warm-up update before admitting any static dwell.
        render_warmup_reset_events = 0
        stable_updates = 0
        for _ in range(RENDER_PRODUCT_WARMUP_UPDATES):
            app.update()
            if _render_state_violations(_read_render_state(settings)):
                render_warmup_reset_events += 1
                stable_updates = 0
            else:
                stable_updates += 1
        render_state = _read_render_state(settings)
        violations = _render_state_violations(render_state)
        if violations:
            raise RuntimeError(
                "post-create renderer state does not match the contract: "
                + "; ".join(violations)
            )
        if stable_updates < 3:
            raise RuntimeError(
                "renderer state did not remain stable for three "
                f"warm-up updates; stable_updates={stable_updates}"
            )
        print(
            "ISAAC_ROS_CAMERA_RENDER_READY",
            f"warmup_updates={RENDER_PRODUCT_WARMUP_UPDATES}",
            f"reset_events={render_warmup_reset_events}",
            f"stable_updates={stable_updates}",
            f"active_render_mode={render_state.active_render_mode!r}",
            f"default_render_mode={render_state.default_render_mode!r}",
            f"active_anti_aliasing={render_state.active_anti_aliasing}",
            f"default_anti_aliasing={render_state.default_anti_aliasing}",
            flush=True,
        )
        if args.static_probe_out is not None:
            completed_updates = _run_static_probe(
                app,
                stage,
                args,
                stage_path,
                profile,
                render_warmup_reset_events,
            )
        elif args.drive_speed_mps is not None:
            completed_updates = _run_drive(app, stage, args, stage_path, profile)
        else:
            for _ in range(args.updates):
                app.update()
            completed_updates = args.updates
        if args.report_gpu_memory:
            gpu_name, used_mib, total_mib = gpu_memory_snapshot()
            print(
                "ISAAC_ROS_CAMERA_GPU",
                f"name={gpu_name}",
                f"used_mib={used_mib}",
                f"total_mib={total_mib}",
                flush=True,
            )
        timeline.stop()
        app.update()
        print(
            "ISAAC_ROS_CAMERA_PASS",
            f"updates={completed_updates}",
            f"profile={profile}",
            f"static_probe={args.static_probe_out is not None}",
            f"drive={args.drive_speed_mps is not None}",
            "render_products=1",
            flush=True,
        )
    except Exception as exc:
        print(
            "ISAAC_ROS_CAMERA_FAIL",
            f"error={type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        os._exit(1)
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
