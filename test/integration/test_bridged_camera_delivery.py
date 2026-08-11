"""End-to-end: the committed gateway configuration actually carries the feed.

``test/test_domain_isolation.py`` proves the boundary blocks. This proves the one
sanctioned crossing opens, using the same YAML the demonstration ships rather
than a configuration written for the test -- a bridge test against a bespoke
config would prove only that domain_bridge works, which is upstream's job.

The negative and the positive run in one test, in one process, against one
publisher. That ordering is the point: the same images that reach nobody before
the bridge starts reach P after it does, so the crossing is shown to be the
cause. Two separate tests could each pass for unrelated reasons.

Skipped, never failed, when ``domain_bridge`` is not installed: the package is an
apt dependency of corridor_gateway and CI resolves it through rosdep, but this
repository's other gates must stay runnable on a machine without it.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import domain_coordinator
import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "src/corridor_gateway/config/corridor_domain_bridge.yaml"
# Must name a topic the shipped config actually relays: this test publishes on it
# and asserts it crosses. Pointing it at anything off the allowlist would turn a
# working bridge into a silent, permanent failure of the positive half.
TOPIC = "/p_cam/image_raw"

# Matches the QoS the config declares and the observer subscribes with. A
# mismatch here would show up as silence and be easy to misread as a boundary
# that never opened.
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _bridge_executable() -> Path:
    try:
        from ament_index_python.packages import get_package_prefix
    except ImportError:  # pragma: no cover - ROS not sourced at all
        pytest.skip("ament_index_python unavailable; ROS is not sourced")
    try:
        prefix = Path(get_package_prefix("domain_bridge"))
    except Exception:
        pytest.skip("domain_bridge is not installed (apt install ros-jazzy-domain-bridge)")
    executable = prefix / "lib" / "domain_bridge" / "domain_bridge"
    if not executable.is_file():
        pytest.skip(f"domain_bridge package found but no executable at {executable}")
    return executable


def _image() -> Image:
    message = Image()
    message.height, message.width, message.encoding = 360, 640, "rgb8"
    message.step = 640 * 3
    message.data = bytes(640 * 360 * 3)
    return message


def test_the_shipped_gateway_config_delivers_the_camera_across_the_boundary() -> None:
    executable = _bridge_executable()
    assert CONFIG.is_file(), "the demonstration's own gateway configuration must exist"

    with domain_coordinator.domain_id() as robot, domain_coordinator.domain_id() as police:
        robot_context, police_context = Context(), Context()
        robot_context.init(domain_id=robot)
        police_context.init(domain_id=police)
        publisher_node = Node("robot_camera", context=robot_context)
        publisher = publisher_node.create_publisher(Image, TOPIC, SENSOR_QOS)
        observer = Node("police_observer_probe", context=police_context)
        received: list[Image] = []
        observer.create_subscription(Image, TOPIC, lambda m: received.append(m), SENSOR_QOS)
        executor = SingleThreadedExecutor(context=police_context)
        executor.add_node(observer)

        # The positive control, on A's own domain. Without it this test reports a
        # closed crossing in any environment where DDS simply does not work --
        # a container without multicast, a misconfigured RMW -- because "nothing
        # arrived" is equally consistent with a broken bridge and a broken
        # network. test_domain_isolation.py holds every negative to this same
        # rule; the crossing is held to it too.
        control_node = Node("robot_side_control", context=robot_context)
        control: list[Image] = []
        control_node.create_subscription(Image, TOPIC, lambda m: control.append(m), SENSOR_QOS)
        control_executor = SingleThreadedExecutor(context=robot_context)
        control_executor.add_node(control_node)

        def pump(seconds: float) -> None:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                publisher.publish(_image())
                control_executor.spin_once(timeout_sec=0.05)
                executor.spin_once(timeout_sec=0.05)
                time.sleep(0.05)

        bridge = None
        try:
            # Negative first: the publisher is already live, so anything that
            # arrives later cannot be blamed on it starting late.
            pump(3.0)
            if not control:
                pytest.skip(
                    "the publisher's own domain received nothing, so DDS delivery is "
                    "unavailable here and a closed-crossing result would be vacuous"
                )
            before_bridge = len(received)

            bridge = subprocess.Popen(  # noqa: S603
                [str(executable), str(CONFIG), "--from", str(robot), "--to", str(police)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            # Discovery plus the bridge's own wait-for-publisher handshake.
            pump(10.0)
            after_bridge = len(received) - before_bridge

            assert bridge.poll() is None, (
                "the bridge exited early; the shipped configuration was rejected: "
                f"{bridge.communicate()[0].decode(errors='replace')[:800]}"
            )
            assert before_bridge == 0, (
                f"{before_bridge} images crossed before the bridge started; "
                "the domains were not isolated to begin with"
            )
            assert after_bridge > 0, (
                f"no image crossed with the bridge running, though the publisher's own "
                f"domain received {len(control)}; the sanctioned crossing is closed"
            )
        finally:
            if bridge is not None and bridge.poll() is None:
                os.killpg(os.getpgid(bridge.pid), signal.SIGTERM)
                try:
                    bridge.wait(timeout=10)
                except subprocess.TimeoutExpired:  # pragma: no cover - shutdown safety net
                    os.killpg(os.getpgid(bridge.pid), signal.SIGKILL)
            control_executor.shutdown()
            executor.shutdown()
            control_node.destroy_node()
            observer.destroy_node()
            publisher_node.destroy_node()
            robot_context.shutdown()
            police_context.shutdown()
