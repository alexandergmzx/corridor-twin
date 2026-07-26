from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_scene_package_has_no_isaac_imports() -> None:
    forbidden = ("import omni", "from omni", "import isaacsim", "from isaacsim")
    violations = []
    for path in (PACKAGE_ROOT / "scene").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        if any(token in text for token in forbidden):
            violations.append(path.name)
    assert violations == []


def test_scenario_records_metric_z_up_stage() -> None:
    scenario = (PACKAGE_ROOT / "config/corridor.yaml").read_text(encoding="utf-8")
    assert "up_axis: Z" in scenario
    assert "meters_per_unit: 1.0" in scenario
