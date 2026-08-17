#!/usr/bin/env python3
"""Block until the TF chain Nav2 needs actually exists.

    python3 tools/wait_for_tf.py --target map --source base_footprint --timeout 120

WHY THIS EXISTS
---------------
Nav2 was being launched as soon as the twin was up, which is too early. Its
costmaps immediately start asking for `map -> base_footprint`, and until SLAM
and the EKF have both published, that lookup fails with

    Invalid frame ID "base_footprint" passed to canTransform ...
    frame does not exist

Enough of those and `bt_navigator`'s lifecycle transition times out, at which
point `lifecycle_manager_nav` reports "Failed to bring up all requested nodes.
Aborting bringup" and every later goal is answered with "Action server is
inactive. Rejecting the goal."

That is a STARTUP RACE, and it is why nominally identical runs alternated
between reaching 0.24 m of B and never leaving the spawn. Waiting for the
transform removes the race at its source rather than retrying the goal after
the stack has already given up.

Exits 0 when the transform resolves, 1 on timeout -- which the caller treats as
infrastructure, because a twin whose TF never comes up is not a fact about
navigation.
"""

from __future__ import annotations

import argparse
import time

import rclpy
import tf2_ros
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", default="map")
    parser.add_argument("--source", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=120.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = Node("wait_for_tf")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    deadline = time.monotonic() + arguments.timeout
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        # can_transform at the LATEST available time, not `now`: sim publishers
        # run slightly behind the wall clock, and asking for `now` fails on a
        # healthy chain purely because the newest transform is 30 ms old.
        if buffer.can_transform(arguments.target, arguments.source, rclpy.time.Time()):
            ok = True
            break

    waited = arguments.timeout - max(0.0, deadline - time.monotonic())
    if ok:
        print(f"  TF {arguments.target} -> {arguments.source} available after {waited:.1f}s")
    else:
        print(f"  TF {arguments.target} -> {arguments.source} NEVER appeared "
              f"in {arguments.timeout:.0f}s", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
