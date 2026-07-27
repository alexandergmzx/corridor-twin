import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "tools/isaac_5_1_ros_camera.py"
PROBE = ROOT / "tools/ros_camera_contract_probe.py"


def literal_assignment(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            targets = [target.id for target in node.targets if isinstance(target, ast.Name)]
            if name in targets:
                return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {path}")


def test_adapter_has_one_small_rgb_camera_and_clock() -> None:
    contract = literal_assignment(ADAPTER, "ADAPTER_CONTRACT")
    assert contract == {
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


def test_adapter_graph_exposes_no_truth_or_pose_shortcut() -> None:
    node_types = literal_assignment(ADAPTER, "NODE_TYPES")
    assert list(node_types.values()).count("isaacsim.core.nodes.IsaacCreateRenderProduct") == 1
    assert list(node_types.values()).count("isaacsim.ros2.bridge.ROS2CameraHelper") == 1
    serialized = " ".join(node_types.values()).lower()
    assert all(word not in serialized for word in ("odometry", "transform", "pose", "tf"))


def test_adapter_self_isolates_isaac_python_from_system_ros() -> None:
    source = ADAPTER.read_text(encoding="utf-8")
    assert '"PYTHONPATH": ""' in source
    assert '"ROS_DISTRO": "jazzy"' in source
    assert '"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"' in source
    assert '"isaacsim.ros2.bridge"' in source
    assert 'bridge_root / "jazzy" / "lib"' in source


def test_live_probe_uses_only_permitted_feed_topics() -> None:
    values = {
        name: literal_assignment(PROBE, name)
        for name in ("IMAGE_TOPIC", "CAMERA_INFO_TOPIC", "CLOCK_TOPIC")
    }
    assert values == {
        "IMAGE_TOPIC": "/robot/front_camera/image_raw",
        "CAMERA_INFO_TOPIC": "/robot/front_camera/camera_info",
        "CLOCK_TOPIC": "/clock",
    }
    source = PROBE.read_text(encoding="utf-8").lower()
    assert "ground_truth" not in source
    assert "odometry" not in source
