#!/usr/bin/env python3
"""Publish the corridor camera and simulation clock with installed Isaac Sim 5.1."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

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
    "image_topic": "/robot/front_camera/image_raw",
    "camera_info_topic": "/robot/front_camera/camera_info",
    "clock_topic": "/clock",
    "frame_id": "robot_front_camera_optical_frame",
    "width": 640,
    "height": 360,
    "simulation_hz": 60,
    "camera_hz": 15,
    "render_products": 1,
    "render_mode": "RaytracedLighting",
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
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--report-gpu-memory", action="store_true")
    return parser.parse_args()


def gpu_memory_snapshot() -> tuple[str, int, int]:
    output = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.used,memory.total",
            "--format=csv,noheader,nounits",
            "--id=0",
        ],
        text=True,
    ).strip()
    name, used_mib, total_mib = (item.strip() for item in output.split(","))
    return name, int(used_mib), int(total_mib)


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
            "anti_aliasing": 1,
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
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(ADAPTER_CONTRACT["simulation_hz"])
        carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)
        _configure_camera_model()
        _create_graph()
        _validate_stage_and_graph()
        print(
            "ISAAC_ROS_CAMERA_GRAPH_READY",
            f"image={ADAPTER_CONTRACT['image_topic']}",
            f"camera_info={ADAPTER_CONTRACT['camera_info_topic']}",
            f"clock={ADAPTER_CONTRACT['clock_topic']}",
            f"resolution={ADAPTER_CONTRACT['width']}x{ADAPTER_CONTRACT['height']}",
            f"rate_hz={ADAPTER_CONTRACT['camera_hz']}",
            "render_products=1",
            flush=True,
        )
        timeline.play()
        for _ in range(args.updates):
            app.update()
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
            f"updates={args.updates}",
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
