"""Prove the communication-domain boundary, with no GPU and no Isaac.

ADR 0020 claims that P cannot discover, list, or subscribe to anything A
publishes. That claim is about DDS discovery, so it is tested against DDS
discovery rather than against source code: a node is stood up in each domain and
asked what it can see.

Nothing here imports ``domain_bridge``. This file tests the *isolation*, which is
a property of ROS domains and holds whether or not the bridge is installed; the
crossing itself is tested in ``test/integration/``. Keeping them apart means the
guarantee everything else rests on still runs on a machine with nothing extra
installed.

Two design points are load-bearing:

**The positive control skips rather than passes.** A test that only asserts "P
saw nothing" passes perfectly in an environment where discovery is broken for
everyone -- a container without multicast, a misconfigured RMW, a sandbox with no
loopback. Every negative here is therefore paired with a positive control in the
same environment, and if that control cannot see its own publisher the result is
``skip``, never ``pass``. ADR 0011 made the same rule for the visibility gate:
without a visible negative control, a pass means nothing.

**Domains are allocated, not hardcoded.** The demonstration runs on 42 and 43,
but two CI jobs on one network would collide on fixed ids and produce a flaky
cross-talk failure that looks like a real leak. ``domain_coordinator`` reserves
unused ids for the duration of each test; what is under test is that *different*
domains do not see each other, not that 42 and 43 specifically do not.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest
from corridor_gateway.domains import RELAYED_TOPICS
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

# Declared as a test dependency of corridor_gateway, so a correctly provisioned
# machine has it. Imported defensively anyway: a bare `import` here would turn a
# missing package into a collection error that takes down every unrelated test in
# the workspace, which is a much worse failure than skipping this file.
domain_coordinator = pytest.importorskip(
    "domain_coordinator",
    reason="apt install ros-jazzy-domain-coordinator to run the isolation proof",
)

ROOT = Path(__file__).resolve().parents[1]
TOPIC = "/robot/front_camera/image_raw"
SETTLE_S = 5.0
POLL_S = 0.2


def _context(domain_id: int) -> Context:
    context = Context()
    context.init(domain_id=domain_id)
    return context


def _sees_topic(node: Node, topic: str) -> bool:
    return any(name == topic for name, _ in node.get_topic_names_and_types())


def _wait_until_seen(node: Node, topic: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _sees_topic(node, topic):
            return True
        time.sleep(POLL_S)
    return False


class _Fixture:
    """A publisher in one domain and an observer in each of two domains."""

    def __init__(self, robot_domain: int, police_domain: int) -> None:
        self.robot_context = _context(robot_domain)
        self.police_context = _context(police_domain)
        self.publisher_node = Node("robot_camera", context=self.robot_context)
        self.publisher = self.publisher_node.create_publisher(Image, TOPIC, 5)
        self.same_domain = Node("same_domain_probe", context=self.robot_context)
        self.other_domain = Node("police_probe", context=self.police_context)

    def close(self) -> None:
        for node in (self.publisher_node, self.same_domain, self.other_domain):
            node.destroy_node()
        for context in (self.robot_context, self.police_context):
            if context.ok():
                context.shutdown()


@pytest.fixture
def two_domains():
    with domain_coordinator.domain_id() as robot, domain_coordinator.domain_id() as police:
        assert robot != police, "domain_coordinator handed out the same id twice"
        fixture = _Fixture(robot, police)
        try:
            yield fixture
        finally:
            fixture.close()


def test_a_topic_is_discoverable_inside_its_own_domain(two_domains) -> None:
    """The control. If this fails, discovery is broken and no negative means anything."""

    if not _wait_until_seen(two_domains.same_domain, TOPIC, SETTLE_S):
        pytest.skip(
            "DDS discovery is unavailable in this environment, so an isolation "
            "negative would pass vacuously"
        )


def test_the_police_domain_cannot_discover_the_robot_camera(two_domains) -> None:
    """The claim ADR 0020 rests on: P's side cannot even see that the topic exists.

    Note what is asserted. Not that P fails to *receive* images -- that would also
    be true if the publisher were simply idle -- but that the topic never appears
    in P's graph at all. Discovery is the layer the isolation lives at, so it is
    the layer the test reads.
    """

    if not _wait_until_seen(two_domains.same_domain, TOPIC, SETTLE_S):
        pytest.skip("no DDS discovery in this environment; the negative would be vacuous")

    # The control has already proven discovery works here and that this topic is
    # discoverable, so any further wait is time for a leak to show up, not time
    # for the publisher to appear.
    time.sleep(SETTLE_S)
    assert not _sees_topic(two_domains.other_domain, TOPIC), (
        "the police domain discovered a robot topic; the domains are not isolated"
    )


def test_no_message_crosses_between_domains_unbridged(two_domains) -> None:
    """Discovery isolation should also mean no data, and that is worth asserting.

    Discovery and delivery are separate mechanisms. This pins the consequence
    that actually matters to the demonstration -- P receives nothing -- so a
    future RMW whose discovery is scoped but whose delivery is not would fail
    here rather than silently widen the boundary.
    """

    if not _wait_until_seen(two_domains.same_domain, TOPIC, SETTLE_S):
        pytest.skip("no DDS discovery in this environment; the negative would be vacuous")

    received: list[Image] = []
    two_domains.other_domain.create_subscription(
        Image, TOPIC, lambda message: received.append(message), 5
    )
    same_domain_received: list[Image] = []
    two_domains.same_domain.create_subscription(
        Image, TOPIC, lambda message: same_domain_received.append(message), 5
    )

    # Each executor is bound to its own context. `rclpy.spin_once(node)` would
    # reach for the *default* context, which this file never initialises -- the
    # nodes here live in explicitly constructed contexts so that two domains can
    # coexist in one process.
    robot_executor = SingleThreadedExecutor(context=two_domains.robot_context)
    police_executor = SingleThreadedExecutor(context=two_domains.police_context)
    robot_executor.add_node(two_domains.same_domain)
    police_executor.add_node(two_domains.other_domain)

    try:
        deadline = time.monotonic() + SETTLE_S
        while time.monotonic() < deadline:
            message = Image()
            message.height, message.width, message.encoding = 360, 640, "rgb8"
            message.step = 640 * 3
            message.data = bytes(640 * 360 * 3)
            two_domains.publisher.publish(message)
            robot_executor.spin_once(timeout_sec=0.05)
            police_executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
    finally:
        robot_executor.shutdown()
        police_executor.shutdown()

    if not same_domain_received:
        pytest.skip("same-domain delivery failed, so the cross-domain negative is vacuous")
    assert received == [], f"{len(received)} images crossed a domain boundary unbridged"


# --- The sim-time hole -------------------------------------------------------
# Everything above proves the boundary blocks what it should. These prove it
# still passes the one thing that has no visible subscription anywhere in the
# observer's source, and would therefore be the easiest entry to delete by
# accident.

LIVE_LAUNCH = ROOT / "src/police_observer/launch/live_demo.launch.py"
SYNTHETIC_LAUNCH = ROOT / "src/police_observer/launch/synthetic_demo.launch.py"
DEMO = ROOT / "tools/run_demo.sh"


def test_the_clock_is_on_the_sanctioned_crossing() -> None:
    """The observer's dependency on /clock is real but invisible to source audits.

    Under ``use_sim_time`` rclpy's own TimeSource calls
    ``node.create_subscription(..., CLOCK_TOPIC, ...)`` on the observer's behalf.
    No line in police_observer constructs it, so the AST walk in
    ``test_repository_contract.py`` cannot enumerate it -- that file says as much
    in a comment beside its permitted-types set.

    The consequence of dropping the entry is not a crash. The observer's clock
    stays at zero, ``node.py``'s ``timestamp_s <= 0.0`` branch resets the pipeline
    on every frame, and the run publishes no estimates at all while every process
    stays up and every log looks ordinary.
    """

    assert RELAYED_TOPICS["/clock"] == "rosgraph_msgs/msg/Clock"


def test_every_sim_time_launcher_also_starts_the_crossing() -> None:
    """Enabling sim time without a gateway is the silent-failure configuration.

    The live path starts the two in separate subshells of run_demo.sh; the
    fallback includes the gateway in its own launch description. Both are checked
    because they are separately editable, and either one could grow a sim-time
    default without a crossing to carry the clock.
    """

    live = LIVE_LAUNCH.read_text(encoding="utf-8")
    demo = DEMO.read_text(encoding="utf-8")
    synthetic = SYNTHETIC_LAUNCH.read_text(encoding="utf-8")

    # The live launch defaults to sim time and does not start the gateway
    # itself, so its documented launcher must.
    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in live
    assert "live_demo.launch.py" in demo and "gateway.launch.py" in demo, (
        "run_demo.sh brings up the sim-time observer, so it must bring up the gateway too"
    )

    # The fallback can be switched to sim time by argument, so it carries its own.
    assert "corridor_gateway" in synthetic
