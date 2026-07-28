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
        "docs/evidence/README.md",
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
        "docs/evidence/README.md": 1,
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
        ROOT / "docs/evidence/README.md",
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


def test_versioned_evidence_topics_have_provenance() -> None:
    evidence_root = ROOT / "docs/evidence"
    missing = [
        str(path.relative_to(ROOT))
        for path in evidence_root.iterdir()
        if path.is_dir() and not (path / "NOTES.md").is_file()
    ]
    assert missing == []


def test_evidence_index_lists_every_recorded_topic() -> None:
    """A topic nobody links to is evidence nobody finds.

    The provenance check above proves each topic directory documents itself, but
    said nothing about whether the index points at it. The live-demo topic sat
    unlisted for four commits because of exactly that gap.
    """

    evidence_root = ROOT / "docs/evidence"
    index = (evidence_root / "README.md").read_text(encoding="utf-8")
    unlisted = [
        path.name
        for path in sorted(evidence_root.iterdir())
        if path.is_dir() and f"{path.name}/NOTES.md" not in index
    ]
    assert unlisted == [], f"add these to docs/evidence/README.md: {unlisted}"


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


# Everything that can influence a published speed estimate. The rule below is
# about what may reach a measurement, so it is enumerated rather than taken as
# "every file in the package": a file added later that feeds the estimate must
# be added here deliberately.
ESTIMATE_PATH_MODULES = (
    "node.py",
    "estimator.py",
    "synthetic.py",
    "synthetic_node.py",
)


def test_observer_consumes_no_actor_ground_truth() -> None:
    """P reads pixels, calibration, time, and the survey. Nothing else."""

    observer = ROOT / "src/police_observer/police_observer"
    forbidden = ("p_bounds", "police_bounds", "delivery_path", "b_xyz", "a_start")
    present = {path.name for path in observer.rglob("*.py")} - {"__init__.py"}
    unclassified = present - set(ESTIMATE_PATH_MODULES) - {"viz_node.py"}
    assert unclassified == set(), (
        f"classify {sorted(unclassified)}: does it feed a published estimate, or only display one?"
    )

    violations: list[str] = []
    for name in ESTIMATE_PATH_MODULES:
        text = (observer / name).read_text(encoding="utf-8")
        violations += [f"{name} -> {token}" for token in forbidden if token in text]
    assert violations == []


# The only messages anything on the estimate path may subscribe to. The camera
# contract is Image + CameraInfo; /clock is handled by rclpy's TimeSource, not
# by a subscription any of these modules constructs.
PERMITTED_SUBSCRIPTION_TYPES = frozenset({"Image", "CameraInfo"})


def _constructed_subscriptions(path: Path) -> list[str]:
    """Return the message type of every subscription a module constructs.

    Covers both forms this package uses: ``create_subscription(TYPE, ...)`` and
    ``message_filters.Subscriber(node, TYPE, ...)`` (``node.py:78,84``).
    """

    found: list[str] = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        name = function.attr if isinstance(function, ast.Attribute) else getattr(function, "id", "")
        if name == "create_subscription" and node.args:
            argument = node.args[0]
        elif name == "Subscriber" and len(node.args) >= 2:
            argument = node.args[1]  # message_filters.Subscriber(node, TYPE, topic)
        else:
            continue
        found.append(
            argument.id if isinstance(argument, ast.Name) else ast.dump(argument)
        )
    return found


def test_estimate_path_subscribes_only_to_the_camera_contract() -> None:
    """Enumerate what the estimate path actually subscribes to, not what it spells.

    A token blocklist was the previous guard, and it had two holes. It named
    only the tokens someone thought of, so `/tf` and `get_world_pose` passed
    freely; and it covered only `node.py`, leaving the rest of the estimate path
    unchecked. Widening the blocklist would also have been wrong: `synthetic
    _node.py` legitimately contains "ground_truth" because it *publishes* the
    evaluator topic `test/ground_truth/speed`, which is the isolation design
    working, not a breach of it.

    Enumerating constructed subscriptions distinguishes reading truth from
    publishing it, and fails on a truth subscription of any type at all --
    including one nobody thought to name.
    """

    observer = ROOT / "src/police_observer/police_observer"
    violations: list[str] = []
    for name in ESTIMATE_PATH_MODULES:
        for message_type in _constructed_subscriptions(observer / name):
            if message_type not in PERMITTED_SUBSCRIPTION_TYPES:
                violations.append(f"{name} subscribes to {message_type}")
    assert violations == [], (
        f"{violations}; the estimate path may subscribe only to "
        f"{sorted(PERMITTED_SUBSCRIPTION_TYPES)}"
    )

    # Guard the guard: if the observer stopped subscribing to the camera
    # entirely, the loop above would pass vacuously.
    assert set(_constructed_subscriptions(observer / "node.py")) == PERMITTED_SUBSCRIPTION_TYPES


def test_display_may_draw_the_scene_but_never_locates_the_robot_from_truth() -> None:
    """The display is held to a different rule than the estimate path, on purpose.

    P knows where P is standing and where the delivery is going; those are
    surveyed scenario facts, not sensor readings, and drawing them is what makes
    the concealment visible. What the display must never do is learn where *A*
    is from anything but a published, camera-derived estimate -- that is the
    claim the whole demonstration rests on.
    """

    view = ROOT / "src/police_observer/police_observer/viz_node.py"
    raw = view.read_text(encoding="utf-8")
    # Audit code, not the docstring that explains which sources it refuses.
    docstring = ast.get_docstring(ast.parse(raw))
    text = raw.replace(docstring, "", 1) if docstring else raw

    # A's authored start pose and any simulator-side pose channel stay out.
    for forbidden in ("a_start", "get_world_pose", "Odometry", "ground_truth", "/tf"):
        assert forbidden not in text, f"the display must not read {forbidden}"

    # A's drawn position must be a function of a subscribed estimate's station.
    assert "estimate.station_m" in text
    assert "approach_s_at_x" in text


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


# Cases the build-side and observer-side policy validators must agree on.
# corridor_scene cannot import police_observer -- the dependency runs the other
# way -- so the invariant is implemented twice. This is what stops the two
# copies drifting apart.
POLICY_CASES = (
    ("empty", [], False),
    ("duplicate threshold", [{"maximum_width_m": 4.0, "limit_mps": 0.8}] * 2, False),
    ("zero limit", [{"maximum_width_m": 4.0, "limit_mps": 0.0}], False),
    ("negative width", [{"maximum_width_m": -1.0, "limit_mps": 0.8}], False),
    ("missing field", [{"maximum_width_m": 4.0}], False),
    (
        "valid but scrambled",
        [
            {"maximum_width_m": 1000.0, "limit_mps": 1.5},
            {"maximum_width_m": 4.0, "limit_mps": 0.8},
            {"maximum_width_m": 5.0, "limit_mps": 1.2},
        ],
        True,
    ),
)


def test_speed_policy_validation_agrees_across_packages() -> None:
    """One invariant, two implementations, and a test that keeps them equal."""

    import sys

    for package in ("src/police_observer", "src/corridor_scene"):
        candidate = str(ROOT / package)
        if candidate not in sys.path:
            sys.path.insert(0, candidate)

    from dataclasses import replace

    from police_observer.estimator import normalized_speed_rules
    from scene.model import _validate_speed_policy, load_scenario

    scenario = load_scenario()
    disagreements = []
    for name, rules, expected_ok in POLICY_CASES:
        candidate = replace(scenario, speed_policy={**scenario.speed_policy, "rules": rules})

        try:
            _validate_speed_policy(candidate)
            build_ok = True
        except ValueError:
            build_ok = False
        try:
            normalized_speed_rules(rules)
            observer_ok = True
        except ValueError:
            observer_ok = False

        if not (build_ok == observer_ok == expected_ok):
            disagreements.append(
                f"{name}: build={build_ok} observer={observer_ok} expected={expected_ok}"
            )
    assert disagreements == []
