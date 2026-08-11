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

    The section marker below is the anchor, and it is asserted before it is used:
    when ADR 0020 retitled the section, `rsplit` silently returned the whole file
    instead of the adapter's block, which swept the observer's own `source
    /opt/ros/jazzy/setup.bash` into the text being audited. That failed loudly
    here, but only by luck -- a rename in the other direction would have widened
    the slice to nothing and passed.
    """

    text = DEMO.read_text(encoding="utf-8")
    marker = "# --- A's side: Isaac"
    assert marker in text, "the anchor moved; this guard would audit the wrong lines"
    isaac_invocation = text.split("isaac_5_1_ros_camera.py")[0].rsplit(marker, 1)[-1]
    for leaked in ("AMENT_PREFIX_PATH", "PYTHONPATH", "ROS_DISTRO", "CMAKE_PREFIX_PATH"):
        assert f"-u {leaked}" in isaac_invocation, f"{leaked} must be unset for the Isaac process"
    assert "source /opt/ros/jazzy/setup.bash" not in isaac_invocation


def test_cleanup_signals_process_groups_not_just_the_launcher() -> None:
    """Observed defect: two rehearsal runs each left three processes alive.

    `ros2 launch` spawns the observer, the display and RViz as its own
    children. Signalling only the launcher leaves every one of them running, so
    a later run finds stale nodes publishing on the same topics and a pile of
    RViz windows. Job control puts each background job in its own process group
    so the trap can take the whole group down.
    """

    text = DEMO.read_text(encoding="utf-8")
    assert re.search(r"^set -m$", text, flags=re.MULTILINE), (
        "job control is what gives each background job its own process group"
    )
    # The negative pid is the process group; a bare `kill "$pid"` is the bug.
    assert 'kill -TERM -- "-$pid"' in text
    assert 'kill -KILL -- "-$pid"' in text
    assert "trap cleanup EXIT INT TERM" in text


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
    assert topics == ["/p_cam/image_raw", "/police/enforcement_view"]
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


def _node_domains(path: Path) -> dict[str, str]:
    """Map each launched node's name to the ``additional_env`` it is pinned with.

    A node with no ``additional_env`` is reported as ``"<unpinned>"`` rather than
    skipped: an unpinned node inherits the ambient domain, which is the exact
    failure this guard exists to catch, and a silent skip would let it pass.
    """

    found: dict[str, str] = {}
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        name = keywords.get("name")
        label = name.value if isinstance(name, ast.Constant) else "<unnamed>"
        environment = keywords.get("additional_env")
        if environment is None:
            found[label] = "<unpinned>"
        elif isinstance(environment, ast.Name):
            found[label] = environment.id
        else:
            # An inline dict or a call the walk cannot resolve is not evidence
            # of correct pinning; name it so the assertion reports it.
            found[label] = ast.dump(environment)
    return found


def test_every_police_node_is_pinned_to_the_police_domain() -> None:
    """ADR 0020: P's processes must not be able to discover A's topics.

    Pinned per node rather than once per launch file on purpose. A launch-wide
    SetEnvironmentVariable applies to whatever is visited after it, so the domain
    would become a property of where a line sits in the action list -- and moving
    a Node above it would silently return that node to the ambient domain.
    """

    assert set(_node_domains(LIVE_LAUNCH).values()) == {"police_side"}


def test_the_fallback_splits_the_two_actors_across_the_two_domains() -> None:
    """The GPU-free path runs the same topology, or it demonstrates a lie.

    This is the only end-to-end run most machines can do, so if it quietly
    collapsed to one domain the isolation would be untested everywhere it can
    actually be observed.
    """

    assert _node_domains(SYNTHETIC_LAUNCH) == {
        "synthetic_camera_publisher": "robot_side",
        "police_observer": "police_side",
        "enforcement_view": "police_side",
        "rviz2": "police_side",
    }
    # The fallback has to carry the crossing too, or P receives nothing at all.
    assert "corridor_gateway" in SYNTHETIC_LAUNCH.read_text(encoding="utf-8")


def test_the_demonstration_refuses_to_run_both_halves_on_one_domain() -> None:
    """Equal domain ids would restore exactly the topology ADR 0020 removed.

    Left unchecked this is silent: everything starts, every topic flows, and the
    isolation claim is simply false while the demonstration looks perfect.
    """

    text = DEMO.read_text(encoding="utf-8")
    assert 'robot_domain="${ROBOT_DOMAIN_ID:-42}"' in text
    assert 'police_domain="${POLICE_DOMAIN_ID:-43}"' in text
    assert '[[ "$robot_domain" == "$police_domain" ]]' in text, (
        "run_demo.sh must reject equal domains rather than silently reuniting the halves"
    )
    # Neither default may be the domain an unconfigured ROS process joins.
    assert ":-0}" not in text

    # Each half is pinned, and the gateway is started to bridge them.
    assert 'export ROS_DOMAIN_ID="$police_domain"' in text
    assert 'ROS_DOMAIN_ID="$robot_domain"' in text
    assert "gateway.launch.py" in text


def test_the_gateway_is_not_confined_to_either_domain() -> None:
    """It is the one participant legitimately in both, so it must stay unpinned.

    Exporting a domain in the gateway's subshell would make it a member of that
    side instead of the boundary between them.
    """

    text = DEMO.read_text(encoding="utf-8")
    gateway_block = text.split("gateway.launch.py")[0].rsplit("# --- The gateway", 1)[-1]
    assert "export ROS_DOMAIN_ID" not in gateway_block
