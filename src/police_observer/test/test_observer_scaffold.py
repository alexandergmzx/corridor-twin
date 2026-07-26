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
