import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_required_documents_exist() -> None:
    required = [
        "README.md",
        "docs/README.md",
        "docs/DESIGN.md",
        "docs/DEVELOPMENT.md",
        "docs/SENSOR-FEED.md",
        "docs/adr/README.md",
    ]
    assert all((ROOT / path).is_file() for path in required)


def test_visual_documentation_entry_points_exist() -> None:
    minimum_mermaid_blocks = {
        "README.md": 1,
        "docs/README.md": 2,
        "docs/DESIGN.md": 2,
        "docs/SENSOR-FEED.md": 1,
        "docs/ACTIVATION.md": 1,
        "docs/DEVELOPMENT.md": 1,
        "docs/adr/README.md": 1,
    }
    for relative_path, expected_minimum in minimum_mermaid_blocks.items():
        content = (ROOT / relative_path).read_text(encoding="utf-8")
        blocks = re.findall(r"```mermaid\n.+?\n```", content, flags=re.DOTALL)
        assert len(blocks) >= expected_minimum, relative_path

    project_map = (ROOT / "docs/README.md").read_text(encoding="utf-8")
    assert "## Project growth map" in project_map
    assert "## Capability and evidence matrix" in project_map
    assert "<b>NEXT</b>" in project_map


def test_visual_documentation_local_links_resolve() -> None:
    visual_documents = [
        ROOT / "README.md",
        ROOT / "docs/README.md",
        ROOT / "docs/DESIGN.md",
        ROOT / "docs/SENSOR-FEED.md",
        ROOT / "docs/ACTIVATION.md",
        ROOT / "docs/DEVELOPMENT.md",
        ROOT / "docs/adr/README.md",
    ]
    missing: list[str] = []
    for document in visual_documents:
        content = document.read_text(encoding="utf-8")
        for target in re.findall(r"\[[^]]+\]\(([^)]+)\)", content):
            if "://" in target or target.startswith("#"):
                continue
            local_target = target.split("#", maxsplit=1)[0]
            if not (document.parent / local_target).exists():
                missing.append(f"{document.relative_to(ROOT)} -> {target}")
    assert missing == []


def test_interface_definitions_exist() -> None:
    message_dir = ROOT / "src/corridor_interfaces/msg"
    assert (message_dir / "SpeedEstimate.msg").is_file()
    assert (message_dir / "SpeedViolation.msg").is_file()


def test_robot_side_sources_are_unaware_of_the_police() -> None:
    """A must not detect, model, or react to P.

    This is additive to the geometric visibility gate, never a replacement for
    it: P could be plainly visible in A's pixels even if A's code ignored them.
    """

    forbidden = ("police_bounds", "p_bounds", "speed_violation", "SpeedViolation")
    robot_side = [
        ROOT / "src/corridor_scene/scene/trajectory.py",
        ROOT / "tools/isaac_5_1_ros_camera.py",
        ROOT / "tools/isaac_5_1_smoke.py",
    ]
    violations: list[str] = []
    for path in robot_side:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        violations += [
            f"{path.relative_to(ROOT)} -> {token}" for token in forbidden if token in text
        ]
    assert violations == []


def test_observer_consumes_no_actor_ground_truth() -> None:
    """P reads pixels, calibration, time, and the survey. Nothing else."""

    observer = ROOT / "src/police_observer/police_observer"
    forbidden = ("p_bounds", "police_bounds", "delivery_path", "b_xyz", "a_start")
    violations: list[str] = []
    for path in observer.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        violations += [f"{path.name} -> {token}" for token in forbidden if token in text]
    assert violations == []


def test_phase_one_python_has_no_isaac_dependencies() -> None:
    source_roots = [
        ROOT / "src/corridor_scene",
        ROOT / "src/police_observer",
    ]
    violations: list[str] = []
    for source_root in source_roots:
        for path in source_root.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                modules: list[str] = []
                if isinstance(node, ast.Import):
                    modules = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    modules = [node.module]
                if any(
                    module.split(".", maxsplit=1)[0] in {"omni", "isaacsim"} for module in modules
                ):
                    violations.append(str(path.relative_to(ROOT)))
    assert violations == []
