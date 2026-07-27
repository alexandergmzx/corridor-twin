import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "tools/isaac_5_1_ros_camera.py"
PROBE = ROOT / "tools/ros_camera_contract_probe.py"
ARUCO_CAPTURE = ROOT / "tools/ros_aruco_capture.py"
ARUCO_GATE = ROOT / "tools/aruco_render_gate.py"
GPU_HELPER = ROOT / "tools/isaac_gpu.py"


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
        "anti_aliasing": 3,
    }


def test_adapter_graph_exposes_no_truth_or_pose_shortcut() -> None:
    node_types = literal_assignment(ADAPTER, "NODE_TYPES")
    assert list(node_types.values()).count("isaacsim.core.nodes.IsaacCreateRenderProduct") == 1
    assert list(node_types.values()).count("isaacsim.ros2.bridge.ROS2CameraHelper") == 1
    serialized = " ".join(node_types.values()).lower()
    assert all(word not in serialized for word in ("odometry", "transform", "pose", "tf"))


def test_static_probe_reuses_the_adapter_product_and_records_both_station_coordinates() -> None:
    defaults = literal_assignment(ADAPTER, "STATIC_PROBE_DEFAULTS")
    assert defaults == {
        "stations_x_m": (0.5, 1.5, 3.0, 5.0, 7.0),
        "settle_updates": 12,
        "capture_updates": 36,
        "output": "out/evidence/static-fiducials/static-truth.json",
    }
    source = ADAPTER.read_text(encoding="utf-8")
    assert "trajectory.approach_s_at_x(station_x_m)" in source
    assert '"route_s_m": route_s_m' in source
    assert '"expected_station_x_m": camera_pose.x_m' in source
    assert "rep.create.render_product" not in source
    assert "ROS2CameraHelper" in source
    assert "settings.get_as_int(ACTIVE_ANTI_ALIASING_KEY)" in source
    assert literal_assignment(ADAPTER, "ACTIVE_ANTI_ALIASING_KEY") == "/rtx/post/aa/op"
    assert literal_assignment(ADAPTER, "DEFAULT_ANTI_ALIASING_KEY") == "/rtx-defaults/post/aa/op"
    assert '"observed_active_anti_aliasing": sorted(' in source
    assert '"observed_default_anti_aliasing": sorted(' in source
    assert "IsaacCreateRenderProduct constructs its Hydra product" in source
    assert '"render_product_warmup_updates": RENDER_PRODUCT_WARMUP_UPDATES' in source


def _function_def(path: Path, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in {path}")


def _dict_containing(node: ast.AST, key: str) -> ast.Dict:
    for candidate in ast.walk(node):
        if isinstance(candidate, ast.Dict):
            keys = [item.value for item in candidate.keys if isinstance(item, ast.Constant)]
            if key in keys:
                return candidate
    raise AssertionError(f"no dictionary literal carrying {key!r}")


def _entries(node: ast.Dict) -> dict[str, ast.AST]:
    return {
        key.value: value
        for key, value in zip(node.keys, node.values, strict=True)
        if isinstance(key, ast.Constant)
    }


def _reads_contract_render_mode(node: ast.AST) -> bool:
    for candidate in ast.walk(node):
        if (
            isinstance(candidate, ast.Subscript)
            and isinstance(candidate.value, ast.Name)
            and candidate.value.id == "ADAPTER_CONTRACT"
            and isinstance(candidate.slice, ast.Constant)
            and candidate.slice.value == "render_mode"
        ):
            return True
    return False


def test_adapter_reads_both_render_mode_keys() -> None:
    source = ADAPTER.read_text(encoding="utf-8")
    assert 'settings.get_as_string(ACTIVE_RENDER_MODE_KEY)' in source
    assert 'settings.get_as_string(DEFAULT_RENDER_MODE_KEY)' in source
    assert literal_assignment(ADAPTER, "ACTIVE_RENDER_MODE_KEY") == "/rtx/rendermode"
    assert literal_assignment(ADAPTER, "DEFAULT_RENDER_MODE_KEY") == "/rtx-defaults/rendermode"


def test_static_schedule_never_sources_renderer_evidence_from_the_request() -> None:
    """Pin the original defect.

    The requested render mode was echoed into the truth schedule under a
    measured-sounding name. Finding a getter somewhere in the file does not
    disprove that, so inspect the serialized dictionary itself: no key that
    reports an observation may be reachable from ADAPTER_CONTRACT["render_mode"].
    """

    probe = _function_def(ADAPTER, "_run_static_probe")
    settings = _entries(_dict_containing(probe, "observed_active_render_modes"))

    observed = {name: node for name, node in settings.items() if name.startswith("observed_")}
    assert {
        "observed_active_render_modes",
        "observed_default_render_modes",
        "observed_active_anti_aliasing",
        "observed_default_anti_aliasing",
    } <= set(observed)
    for name, node in observed.items():
        assert not _reads_contract_render_mode(node), f"{name} is sourced from the request"
        assert isinstance(node, ast.Call) and node.func.id == "sorted", (
            f"{name} must serialize a readback accumulator deterministically"
        )

    # The request may still be recorded, but only under a name that says so.
    assert "render_mode" not in settings
    assert _reads_contract_render_mode(settings["requested_render_mode"])

    # path_tracing must be derived from the readback, not asserted.
    assert not isinstance(settings["path_tracing"], ast.Constant)
    assert not _reads_contract_render_mode(settings["path_tracing"])


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


def test_aruco_capture_has_only_the_production_camera_contract() -> None:
    contract = literal_assignment(ARUCO_CAPTURE, "CAPTURE_CONTRACT")
    assert contract == {
        "image_topic": "/robot/front_camera/image_raw",
        "camera_info_topic": "/robot/front_camera/camera_info",
        "clock_topic": "/clock",
        "frame_id": "robot_front_camera_optical_frame",
        "width": 640,
        "height": 360,
    }
    source = ARUCO_CAPTURE.read_text(encoding="utf-8").lower()
    assert all(word not in source for word in ("ground_truth", "odometry", "cmd_vel"))


def test_render_gate_separates_pixel_analysis_from_truth_comparison() -> None:
    tree = ast.parse(ARUCO_GATE.read_text(encoding="utf-8"), filename=str(ARUCO_GATE))
    functions = {
        node.name: [argument.arg for argument in node.args.args]
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }
    assert functions["analyse_capture"] == [
        "capture_path",
        "manifest_path",
        "profile_name",
        "image_transform",
    ]
    assert "truth" not in functions["analyse_capture"]
    assert functions["compare_to_truth"] == ["analysis", "truth", "manifest"]


def test_both_isaac_tools_reuse_one_gpu_snapshot_helper() -> None:
    assert GPU_HELPER.is_file()
    for path in (ADAPTER, ROOT / "tools/isaac_5_1_smoke.py"):
        source = path.read_text(encoding="utf-8")
        assert "from isaac_gpu import gpu_memory_snapshot" in source
        assert '"--query-gpu=' not in source
