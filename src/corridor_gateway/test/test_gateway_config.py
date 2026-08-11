"""The gateway is an allowlist, so the allowlist is what gets tested.

There is no node source in this package to audit -- the relay is the upstream
``domain_bridge`` binary, and a test cannot walk its call graph the way
``test_repository_contract.py`` walks the observer's. What *is* reviewable is the
configuration the binary reads, and that is where every property this package
claims actually lives: which topics cross, in which direction, between which
domains, under which QoS.

So these assertions restate the contract in Python rather than importing it from
the YAML and comparing it to itself. A test that read the topic list out of the
file and asserted the file contained that list would pass no matter what the file
said.
"""

from __future__ import annotations

from pathlib import Path

import yaml

# The workspace-level `pytest -q` runs before `colcon build`, so the install
# space does not exist yet; pyproject.toml's `pythonpath` puts src/ on the path
# for exactly this reason.
from corridor_gateway.domains import POLICE_DOMAIN_ID, RELAYED_TOPICS, ROBOT_DOMAIN_ID

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config" / "corridor_domain_bridge.yaml"

# Mirrors `sensor_qos` and `CLOCK_QOS` in police_observer, and the QoS column of
# docs/SENSOR-FEED.md. Written out rather than derived so a change to either side
# has to be made deliberately in both.
SENSOR_QOS = {
    "reliability": "best_effort",
    "durability": "volatile",
    "history": "keep_last",
    "depth": 5,
}
CLOCK_QOS = {**SENSOR_QOS, "depth": 1}

# Since ADR 0021 the crossing carries P's own enforcement camera, not a sensor on
# A. The names moved with the ownership; the QoS did not, because nothing about
# the transport changed. Resolution and rate stay unpinned here -- they are not
# properties of the allowlist, and ADR 0024 re-measures them.
EXPECTED = {
    "/p_cam/image_raw": ("sensor_msgs/msg/Image", SENSOR_QOS),
    "/p_cam/camera_info": ("sensor_msgs/msg/CameraInfo", SENSOR_QOS),
    "/clock": ("rosgraph_msgs/msg/Clock", CLOCK_QOS),
}


def _config() -> dict:
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_the_bridge_runs_between_the_two_declared_domains() -> None:
    config = _config()
    assert config["from_domain"] == ROBOT_DOMAIN_ID
    assert config["to_domain"] == POLICE_DOMAIN_ID
    # Neither half may sit on the default domain: an unconfigured ROS process
    # joins domain 0, so a fallback there would silently reunite the two halves.
    assert 0 not in {config["from_domain"], config["to_domain"]}


def test_exactly_the_sanctioned_topics_cross_and_nothing_else() -> None:
    """The allowlist is the security property; an extra entry is the whole bug."""

    topics = _config()["topics"]
    assert set(topics) == set(EXPECTED), (
        "the gateway's topic set changed; every entry widens what P can observe"
    )
    # domains.py is what the launch files and documentation quote. If it drifts
    # from the file the bridge actually reads, the documentation describes a
    # boundary that is not the one being enforced.
    assert set(topics) == set(RELAYED_TOPICS)

    for topic, (expected_type, expected_qos) in EXPECTED.items():
        assert topics[topic]["type"] == expected_type, topic
        assert RELAYED_TOPICS[topic] == expected_type, topic
        assert topics[topic]["qos"] == expected_qos, topic


def test_no_topic_is_bridged_back_toward_the_robot() -> None:
    """One-way is a property of this file, so this file is where it is checked.

    domain_bridge is one-way by default, but ``reversed`` and ``bidirectional``
    are per-topic opt-ins. Either one on any entry would let P's domain reach
    A's -- and A observing its own enforcement is precisely what the scenario
    forbids. ``remap`` is refused too: a renamed topic is a topic the observer
    contract and docs/SENSOR-FEED.md no longer describe.
    """

    offenders = [
        f"{topic}: {key}"
        for topic, options in _config()["topics"].items()
        for key in ("reversed", "bidirectional", "remap")
        if key in options
    ]
    assert offenders == [], f"the gateway must stay one-way and unrenamed: {offenders}"


def test_the_clock_is_carried_explicitly() -> None:
    """Regression guard for the failure this package exists to prevent.

    Under ``use_sim_time`` rclpy's TimeSource subscribes to /clock internally, so
    no observer source line mentions it and the AST subscription guard in
    test_repository_contract.py cannot see it. If /clock stops crossing, the
    observer's clock never advances, node.py resets the pipeline on every frame,
    and the demonstration publishes nothing at all while appearing to run.
    """

    assert "/clock" in _config()["topics"]
    assert RELAYED_TOPICS["/clock"] == "rosgraph_msgs/msg/Clock"
