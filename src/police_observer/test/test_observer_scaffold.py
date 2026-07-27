import ast
from pathlib import Path

from police_observer.synthetic_node import CLOCK_QOS
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_observer_package_has_no_isaac_imports() -> None:
    forbidden = ("import omni", "from omni", "import isaacsim", "from isaacsim")
    violations = []
    for path in (PACKAGE_ROOT / "police_observer").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        if any(token in text for token in forbidden):
            violations.append(path.name)
    assert violations == []


def test_observer_time_mode_is_explicit() -> None:
    config = (PACKAGE_ROOT / "config/observer.yaml").read_text(encoding="utf-8")
    assert "use_sim_time: false" in config


def test_synthetic_clock_qos_matches_jazzy_time_source() -> None:
    assert CLOCK_QOS.history == HistoryPolicy.KEEP_LAST
    assert CLOCK_QOS.depth == 1
    assert CLOCK_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert CLOCK_QOS.durability == DurabilityPolicy.VOLATILE


VIEW_SOURCE = PACKAGE_ROOT / "police_observer" / "viz_node.py"


def _code_without_docstring(path: Path) -> str:
    """Return a module's source with its module docstring removed.

    This module's docstring explains at length which truth sources it refuses
    to read, so auditing raw text would either fail on the explanation or force
    the explanation out of the file. The audit is about code.
    """

    text = path.read_text(encoding="utf-8")
    docstring = ast.get_docstring(ast.parse(text))
    return text.replace(docstring, "", 1) if docstring else text


def test_enforcement_view_reads_no_truth_source() -> None:
    """The audit the observer node carries, applied to the demonstration display.

    A plan view is the easiest place in this project to cheat: drawing A where
    the simulator put it looks identical to drawing A where the camera measured
    it. This is cheap enough to run without the colcon-generated interfaces, so
    it stays outside the behavioural suite that needs them.
    """

    source = _code_without_docstring(VIEW_SOURCE).lower()
    for forbidden in ("ground_truth", "odometry", "cmd_vel", "get_world_pose", "tf2"):
        assert forbidden not in source, f"the display must not read {forbidden}"


def test_enforcement_view_is_a_consumer_and_never_a_sensor() -> None:
    """A node producing or consuming imagery would have to import for it."""

    source = _code_without_docstring(VIEW_SOURCE)
    for forbidden in ("sensor_msgs", "cv_bridge", "render_product"):
        assert forbidden not in source, f"the display must not involve {forbidden}"
