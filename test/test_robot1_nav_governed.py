"""robot1's Nav2 launch must route every motion writer through the governor.

ADR 0023 makes the governed path law, and this is the one file in the
repository where that law is implemented rather than described. Both vendor
alternatives are ungoverned -- `navigation_dwb_launch.py` has no cmd_vel
remapping at all, and `yahboomcar_multi/navigation_launch.py` remaps into a
velocity_smoother whose output lands on `/cmd_vel` -- so "it launched Nav2 and
the robot moved" is not evidence of anything. It is exactly what running
ungoverned looks like.

The launch is checked as a syntax tree rather than by importing it, so these
tests run without ROS present and without launching anything.

The subtle failure this guards is not a missing remap; it is a remap on the
controller but NOT on the behavior server. Path following would then be
governed while recovery spins and backups went straight to the motors -- and
recoveries fire precisely when Nav2 is already confused, which is the worst
possible moment to bypass a safety layer.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parent.parent
LAUNCH = ROOT / "config/robot1/robot1_nav_corridor_launch.py"
PARAMS = ROOT / "config/robot1/nav2_robot1_corridor.yaml"

#: Every Nav2 node that can emit motion. Both must be governed.
MOTION_WRITERS = {"controller_server", "behavior_server"}


def _launch_tree() -> ast.Module:
    return ast.parse(LAUNCH.read_text(encoding="utf-8"))


def _node_calls() -> dict[str, dict[str, ast.expr]]:
    """Map each `Node(...)` call's `name=` to its keyword arguments."""

    nodes: dict[str, dict[str, ast.expr]] = {}
    for call in (n for n in ast.walk(_launch_tree()) if isinstance(n, ast.Call)):
        if not (isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        keywords = {kw.arg: kw.value for kw in call.keywords if kw.arg}
        name = keywords.get("name")
        if isinstance(name, ast.Constant):
            nodes[name.value] = keywords
    return nodes


def test_every_motion_writer_is_remapped_into_the_governed_pipe() -> None:
    nodes = _node_calls()

    for writer in MOTION_WRITERS:
        assert writer in nodes, f"{writer} is not launched at all"
        remappings = nodes[writer].get("remappings")
        assert remappings is not None, (
            f"{writer} has no remappings: its cmd_vel goes straight to the motors"
        )
        assert isinstance(remappings, ast.Name) and remappings.id == "GOVERNED", (
            f"{writer} does not use the shared GOVERNED remapping"
        )


def test_the_governed_remapping_points_at_the_governor_input() -> None:
    """`cmd_vel` -> `cmd_vel_raw` is what robot1's governor actually listens to.

    `cmd_vel_governor.py:84` subscribes to the absolute `/cmd_vel_raw` and has
    no topic parameter to retarget, so this pair is not a convention -- it is
    the one string that reaches it.
    """

    tree = _launch_tree()
    governed = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "GOVERNED" for t in node.targets)
    )

    assert ast.literal_eval(governed) == [("cmd_vel", "cmd_vel_raw")]


def test_no_node_is_namespaced() -> None:
    """robot1 runs at root, and that is what makes the remapping reach the governor.

    A `namespace=` on any of these nodes would resolve the remapping to
    `/<ns>/cmd_vel_raw`, which nothing subscribes to. Nav2 would then publish
    `/cmd_vel` directly at the motors while every topic name still looked
    plausible.
    """

    offenders = [name for name, keywords in _node_calls().items() if "namespace" in keywords]

    assert offenders == [], (
        f"namespaced nodes would silently bypass robot1's governor: {offenders}"
    )


def test_the_lifecycle_bond_is_disabled() -> None:
    """D-19/OI-14: bond_timeout 30.0 deactivated managed nodes within ~200 ms."""

    source = LAUNCH.read_text(encoding="utf-8")

    assert '"bond_timeout": 0.0' in source


def test_the_param_file_targets_robot1_topics_and_frames() -> None:
    """The five substitutions that robot1's architecture forces.

    Each one fails silently if missed: a wrong odom topic leaves the controller
    waiting on a publisher that does not exist, and a wrong scan topic points
    the costmaps at a stream robot1 never produces.
    """

    text = PARAMS.read_text(encoding="utf-8")
    body = text[text.index("/**:") :]

    assert "odom_topic: /odom" in body
    assert "topic: /scan" in body
    assert "map_topic: /map" in body
    assert "scan_filtered" not in body, "robot1 has no scan_filtered producer"
    assert "robot2/" not in body, "robot2 frames do not exist on robot1"


def test_the_param_file_parses_and_keeps_nav2_under_the_governor_cap() -> None:
    """Nav2 must never ask for more than the governed path allows.

    robot1's governor caps forward speed at 0.35 m/s
    (yahboomcar_safety/governor.py:41-60). A Nav2 max above that would have the
    governor clamping every command, which reads in a log as Nav2 "working"
    while the robot is permanently speed-limited by a safety layer rather than
    by its planner.
    """

    document = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    parameters = document["/**:"] if "/**:" in document else document["/**"]
    velocity = parameters["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_x"]

    assert velocity == pytest.approx(0.22)
    assert velocity < 0.35


# --- U3: the MPPI arm --------------------------------------------------------
# The comparison is only meaningful if the two arms differ in EXACTLY one thing.
# These pin that: same footprint, same inflation, same planner, same governed
# remapping, different controller.


def _load(name):
    import yaml
    path = Path(__file__).parent.parent / "config/robot1" / name
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]


def test_the_two_arms_differ_only_in_the_controller() -> None:
    dwb = _load("nav2_robot1_corridor.yaml")
    mppi = _load("nav2_robot1_corridor_mppi.yaml")

    assert dwb["controller_server"]["ros__parameters"]["FollowPath"]["plugin"] \
        != mppi["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
    for section in ("planner_server", "behavior_server", "local_costmap", "global_costmap"):
        assert dwb[section] == mppi[section], f"{section} differs between the arms"


def test_both_arms_share_the_measured_footprint_and_inflation() -> None:
    """A comparison run on two different robots measures nothing."""

    for name in ("nav2_robot1_corridor.yaml", "nav2_robot1_corridor_mppi.yaml"):
        params = _load(name)
        for scope in ("local_costmap", "global_costmap"):
            costmap = params[scope][scope]["ros__parameters"]
            assert costmap["robot_radius"] == 0.128
            # 0.18 since the corner narrowed to 0.90 m: at 0.30 only 0.30 m of
            # the corner would be uninflated, against 0.54 m at 0.18.
            assert costmap["inflation_layer"]["inflation_radius"] == 0.18


def test_the_mppi_arm_respects_the_governors_near_wall_yaw_cap() -> None:
    """0.4 rad/s, for the same measured reason the DWB arm carries it."""

    mppi = _load("nav2_robot1_corridor_mppi.yaml")["controller_server"]["ros__parameters"]

    assert mppi["FollowPath"]["wz_max"] == 0.4
    assert mppi["FollowPath"]["vx_max"] == 0.22
    assert mppi["FollowPath"]["motion_model"] == "DiffDrive"


def test_loop_closing_is_on_in_the_params_the_runner_actually_launches() -> None:
    """The falsified hypothesis stays reverted, checked at the file that RUNS.

    9c88a93 turned loop closing off on the argument that a single-pass delivery
    has no loop to close. That is wrong -- slam_toolbox also closes against
    recent scan chains, which is how it corrects accumulated drift -- and the
    operator observed SLAM behaving worse. cd6e946 reverted it.

    The corridor-local file that carries `false` is still on disk as the record
    of that experiment, opt-in behind --corridor-slam. So checking the config
    with the corridor's name on it would check the wrong file: this follows the
    runner's default to the params it actually launches.
    """

    import re

    runner = (ROOT / "tools/corridor_profile_run.sh").read_text(encoding="utf-8")
    default = re.search(r'^SLAM_PARAMS="([^"]+)"', runner, re.MULTILINE)
    assert default, "the runner no longer declares a default SLAM params file"

    params = Path(default.group(1))
    if not params.is_file():
        pytest.skip(f"the fleet params file is not in place: {params}")

    body = params.read_text(encoding="utf-8")
    assert re.search(r"^\s*do_loop_closing:\s*true\s*$", body, re.MULTILINE), (
        f"{params} does not have loop closing on; the revert has been undone"
    )

    # And the opt-in record still says what it is, so nobody re-adopts it by
    # reading only the filename.
    record = ROOT / "config/robot1/slam_robot1_corridor.yaml"
    assert "NOT IN USE" in record.read_text(encoding="utf-8")
