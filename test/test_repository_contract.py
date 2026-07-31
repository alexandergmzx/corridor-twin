import ast
import json
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
        "docs/REVIEW-LOG.md",
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
        ROOT / "docs/REVIEW-LOG.md",
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


def test_docs_readme_gpu_figures_stay_labelled_pending_refresh() -> None:
    """A6-M4 (Round 7): live-Isaac and VRAM figures need an explicit caveat.

    docs/README.md once described the R17 plate-relocation's 3,486 MiB
    measurement -- taken weeks before P moved sides under ADR 0019 -- as "the
    live demonstration on the corrected geometry": both the wrong run and the
    wrong side of the correction. ACTIVATION.md and RELEASE-v1.0-interview.md
    already carry an explicit pending-refresh banner for exactly this
    situation; docs/README.md must too, so a reader does not have to compare
    three documents to notice one disagrees.
    """

    content = (ROOT / "docs/README.md").read_text(encoding="utf-8")
    assert "predates the 2026-07-29 police-placement correction" in content
    # The exact wrong claim: a past-tense measurement asserted to already
    # describe the corrected geometry. "fresh GPU requalification on the
    # corrected geometry" (future work, still pending) is fine and expected.
    assert "in the live demonstration on the corrected geometry" not in content


def test_versioned_evidence_topics_have_provenance() -> None:
    evidence_root = ROOT / "docs/evidence"
    missing = [
        str(path.relative_to(ROOT))
        for path in evidence_root.iterdir()
        if path.is_dir() and not (path / "NOTES.md").is_file()
    ]
    assert missing == []


def test_live_run_headline_figures_match_the_recorded_summary() -> None:
    """Documents that quote the live run must quote the run that is recorded.

    `cdb6f79` re-recorded the demonstration on corrected geometry, and seven
    citations in five documents kept the superseded numbers -- two of them
    disagreeing with the notes' own summary. A reader comparing README against
    the evidence would have found two different runs described as the same one.

    This asserts the *current* figures are present rather than listing forbidden
    old ones. The forbidden-list form needed updating on every rerun, in the
    same commit as the documents it was meant to police, which is the failure
    mode inverted rather than fixed. Reading the artifact means a rerun updates
    the documents and nothing else.
    """

    summary = json.loads(
        (ROOT / "docs/evidence/live-demo/summary.json").read_text(encoding="utf-8")
    )
    worst_error = max(entry["speed_error_mps"] for entry in summary["estimates"])
    exceedance = summary["violations"][0]["exceedance_mps"]
    used_mib = summary["gpu"]["used_mib"]

    # Each document quotes at whatever precision suits it, so a figure is
    # satisfied by any of its reasonable spellings.
    def spellings(value: float, places: tuple[int, ...]) -> list[str]:
        return [f"{value:.{place}f}" for place in places]

    required = {
        "README.md": [
            spellings(worst_error, (3, 4)),
            spellings(exceedance, (3, 4)),
            [str(used_mib), f"{used_mib:,}"],
        ],
        "CLAUDE.md": [spellings(worst_error, (3, 4)), [str(used_mib), f"{used_mib:,}"]],
        "docs/README.md": [
            spellings(worst_error, (3, 4)),
            spellings(exceedance, (3, 4)),
            [str(used_mib), f"{used_mib:,}"],
        ],
        "docs/evidence/live-demo/NOTES.md": [
            spellings(worst_error, (3, 4)),
            spellings(exceedance, (3, 4)),
            [str(used_mib), f"{used_mib:,}"],
        ],
    }

    missing: list[str] = []
    for relative_path, figures in required.items():
        content = (ROOT / relative_path).read_text(encoding="utf-8")
        for alternatives in figures:
            if not any(spelling in content for spelling in alternatives):
                missing.append(f"{relative_path} cites none of {alternatives}")
    assert missing == [], f"documents disagree with the recorded run: {missing}"

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


# Helpers that subscribe on the node handed to them, without the calling module
# ever writing `create_subscription`. `TransformListener` is the one that
# matters: it is the ordinary way to consume TF, and TF is named directly in
# CLAUDE.md's truth-isolation invariant. Enumerating call sites cannot see
# inside them, so they are refused by name on the estimate path.
IMPLICIT_SUBSCRIBER_CONSTRUCTORS = frozenset(
    {"TransformListener", "Buffer", "TransformBroadcaster", "MessageFilter"}
)


def _constructed_subscriptions(path: Path) -> list[str]:
    """Return the message type of every subscription a module constructs.

    Covers both forms this package uses -- ``create_subscription(TYPE, ...)``
    and ``message_filters.Subscriber(node, TYPE, ...)`` (``node.py:78,84``) --
    in positional *and* keyword spelling. Keyword form matters because
    ``create_subscription(msg_type=Odometry, ...)`` is ordinary rclpy: reading
    only ``node.args[0]`` returned nothing for it, so the type never reached
    the permitted-set check and a truth subscription passed silently.

    A call the walk cannot resolve to a name is reported as its dump rather
    than skipped, so an unrecognised spelling fails the permitted-set check
    instead of disappearing from it.
    """

    found: list[str] = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        name = function.attr if isinstance(function, ast.Attribute) else getattr(function, "id", "")
        keywords = {word.arg: word.value for word in node.keywords}
        if name == "create_subscription":
            argument = node.args[0] if node.args else keywords.get("msg_type")
        elif name == "Subscriber":
            # message_filters.Subscriber(node, TYPE, topic)
            argument = node.args[1] if len(node.args) >= 2 else keywords.get("msg_type")
        elif name in IMPLICIT_SUBSCRIBER_CONSTRUCTORS:
            found.append(name)
            continue
        else:
            continue
        if argument is None:
            # A subscription whose type could not be located is not evidence of
            # safety. Name the call so the assertion reports it.
            found.append(f"{name}(<unresolved message type>)")
            continue
        found.append(argument.id if isinstance(argument, ast.Name) else ast.dump(argument))
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
    including one nobody thought to name, provided the walk can see it. See
    ``test_the_subscription_walk_sees_every_route_this_package_could_use`` for
    the spellings that must stay visible to it.
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


# Each is a real way to start receiving a message in this package, paired with
# the type an auditor must see reported. The claim being defended is that the
# walk sees the *route*, not that anyone writes it this way today: three of
# these returned an empty list before this test existed, so a truth
# subscription spelled any of those ways passed the guard above in silence.
SUBSCRIPTION_SPELLINGS = (
    ("positional create_subscription", "self.create_subscription(Odometry, '/odom', cb, 10)"),
    (
        "keyword create_subscription",
        "self.create_subscription(msg_type=Odometry, topic='/odom', callback=cb, qos_profile=10)",
    ),
    ("positional message_filters", "message_filters.Subscriber(self, Odometry, '/odom')"),
    ("keyword message_filters", "message_filters.Subscriber(self, msg_type=Odometry)"),
    ("tf2_ros listener", "TransformListener(self.buffer, self)"),
)


def test_the_subscription_walk_sees_every_route_this_package_could_use(
    tmp_path: Path,
) -> None:
    """The enumeration is only as good as the spellings it can parse.

    ``create_subscription(msg_type=...)`` is ordinary rclpy and
    ``TransformListener(buffer, node)`` is the ordinary way to consume TF --
    which CLAUDE.md names directly as forbidden truth. Both used to enumerate
    to nothing, so the permitted-set check never ran on them and a truth
    subscription passed. This pins each route as visible rather than trusting
    that nobody will reach for it.
    """

    for label, source in SUBSCRIPTION_SPELLINGS:
        module = tmp_path / f"{label.replace(' ', '_')}.py"
        module.write_text(f"class N:\n    def build(self):\n        {source}\n", encoding="utf-8")
        seen = _constructed_subscriptions(module)
        assert seen, f"{label}: the walk enumerated nothing, so the guard cannot judge it"
        assert not PERMITTED_SUBSCRIPTION_TYPES.issuperset(seen), (
            f"{label}: enumerated {seen}, which the permitted set would accept"
        )


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
    # Non-finite values are the case the sign tests miss on their own: `nan <=
    # 0` and `inf <= 0` are both False. Before these cases the build side
    # accepted all four while the observer side rejected them, so a policy
    # written with YAML's `.inf` built a scene and a manifest and then stopped
    # the observer from constructing -- the split the agreement test exists to
    # catch. The manifest was not even strictly valid JSON: `json.dump` writes
    # a bare `Infinity` token.
    ("nan threshold", [{"maximum_width_m": float("nan"), "limit_mps": 1.5}], False),
    ("inf threshold", [{"maximum_width_m": float("inf"), "limit_mps": 1.5}], False),
    ("nan limit", [{"maximum_width_m": 4.0, "limit_mps": float("nan")}], False),
    ("inf limit", [{"maximum_width_m": 4.0, "limit_mps": float("inf")}], False),
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
