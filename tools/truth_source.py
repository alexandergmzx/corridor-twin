#!/usr/bin/env python3
"""Publish simulator truth on A's plane, so the isolation claim has a subject.

    ROS_DOMAIN_ID=42 python3 tools/truth_source.py

This is test scaffolding for the isolation certificate, not part of the
demonstration. It publishes `/test/ground_truth/speed` -- the topic the
truth-isolation invariant forbids reaching P -- on whatever domain it is
started in, and nothing else.

It exists because a certificate taken against an empty graph certifies nothing.
The adapter alone publishes only the allowlist on A's plane, so "P sees exactly
the allowlist" would hold with nothing available to leak in the first place.
With this running, the certificate makes a real claim: a non-allowlisted topic
is live on A's plane and does not appear in P's.

The values are zeros on purpose. What is under test is whether the topic
crosses a domain boundary, and a zero TwistStamped crosses exactly as well as a
populated one.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

TOPIC = "/test/ground_truth/speed"


def main() -> None:
    rclpy.init()
    node = Node("corridor_truth_source")
    publisher = node.create_publisher(TwistStamped, TOPIC, 10)

    def tick() -> None:
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)

    node.create_timer(0.1, tick)
    node.get_logger().info(f"publishing {TOPIC} on this domain (isolation scaffolding)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The context is often already torn down by the signal that stopped the
        # spin; destroying into a dead context raises RCLError and buries the
        # real reason the process exited under a traceback.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001 - shutdown races are not failures here
            pass


if __name__ == "__main__":
    main()
