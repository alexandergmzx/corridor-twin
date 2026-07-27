"""The one-command demonstration must keep the two Python ABIs apart.

Nothing here starts a simulator. These are the rules that are easy to break by
editing a launch file and only discover on the day: system ROS leaking into the
Isaac process, a second camera appearing in the display, or the observer being
put back on wall time while the adapter stamps from the simulation clock.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "tools/run_demo.sh"
LIVE_LAUNCH = ROOT / "src/police_observer/launch/live_demo.launch.py"
SYNTHETIC_LAUNCH = ROOT / "src/police_observer/launch/synthetic_demo.launch.py"
RVIZ = ROOT / "src/police_observer/rviz/corridor_twin.rviz"


def test_demo_script_is_executable_and_documented() -> None:
    assert DEMO.is_file()
    assert DEMO.stat().st_mode & 0o111, "run_demo.sh must be executable"
    text = DEMO.read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    assert "--headless" in text and "--record" in text


def test_isaac_process_is_started_without_system_ros_on_its_path() -> None:
    """The adapter re-execs into Isaac's bundled Jazzy and rejects leaked paths.

    Sourcing system ROS in that shell does not merge two ROS installations, it
    aborts the run -- and mixing the two ABIs is exactly what the environment
    discipline in CLAUDE.md exists to prevent.
    """

    text = DEMO.read_text(encoding="utf-8")
    isaac_invocation = text.split("isaac_5_1_ros_camera.py")[0].rsplit("# --- Isaac side", 1)[-1]
    for leaked in ("AMENT_PREFIX_PATH", "PYTHONPATH", "ROS_DISTRO", "CMAKE_PREFIX_PATH"):
        assert f"-u {leaked}" in isaac_invocation, f"{leaked} must be unset for the Isaac process"
    assert "source /opt/ros/jazzy/setup.bash" not in isaac_invocation


def test_ros_side_starts_before_the_publisher() -> None:
    """docs/ACTIVATION.md validates consumer-first ordering; keep it."""

    text = DEMO.read_text(encoding="utf-8")
    assert text.index("live_demo.launch.py") < text.index("isaac_5_1_ros_camera.py")


def test_live_launch_starts_only_system_jazzy_nodes() -> None:
    """The adapter cannot run inside a system-Jazzy launch description.

    Checked structurally rather than by searching the text for "isaac": this
    file's docstring explains at length why the adapter is absent, and an audit
    that forbids the word would forbid the explanation.
    """

    tree = ast.parse(LIVE_LAUNCH.read_text(encoding="utf-8"))
    launched, raw_processes = [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id == "Node":
            launched += [
                keyword.value.value
                for keyword in node.keywords
                if keyword.arg == "package" and isinstance(keyword.value, ast.Constant)
            ]
        elif node.func.id in {"ExecuteProcess", "ExecuteLocal"}:
            raw_processes.append(node.func.id)

    assert sorted(launched) == ["police_observer", "police_observer", "rviz2"]
    assert raw_processes == [], "a raw process could start the adapter in the wrong environment"


def test_live_launch_defaults_to_simulation_time() -> None:
    """The adapter stamps from /clock and the observer differentiates stamps."""

    text = LIVE_LAUNCH.read_text(encoding="utf-8")
    assert re.search(r'DeclareLaunchArgument\("use_sim_time",\s*default_value="true"\)', text)
    assert text.count('"use_sim_time": use_sim_time') == 2  # observer and display


def test_display_shows_one_camera_and_no_other_sensor() -> None:
    config = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    topics = [
        display["Topic"]["Value"] for display in displays if isinstance(display.get("Topic"), dict)
    ]
    assert topics == ["/robot/front_camera/image_raw", "/police/enforcement_view"]
    image_displays = [d for d in displays if d["Class"] == "rviz_default_plugins/Image"]
    assert len(image_displays) == 1, "one camera means one image display"
    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "world"


def test_the_simulator_free_fallback_is_still_available() -> None:
    """The recorded fallback the interview plan depends on must keep working."""

    assert SYNTHETIC_LAUNCH.is_file()
    text = SYNTHETIC_LAUNCH.read_text(encoding="utf-8")
    assert "synthetic-camera-publisher" in text
    assert "police-observer" in text
    # run_demo.sh points at it when Isaac is unavailable.
    assert "synthetic_demo.launch.py" in DEMO.read_text(encoding="utf-8")
