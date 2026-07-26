import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_required_documents_exist() -> None:
    required = [
        "README.md",
        "docs/DESIGN.md",
        "docs/DEVELOPMENT.md",
        "docs/SENSOR-FEED.md",
        "docs/adr/README.md",
    ]
    assert all((ROOT / path).is_file() for path in required)


def test_interface_definitions_exist() -> None:
    message_dir = ROOT / "src/corridor_interfaces/msg"
    assert (message_dir / "SpeedEstimate.msg").is_file()
    assert (message_dir / "SpeedViolation.msg").is_file()


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
