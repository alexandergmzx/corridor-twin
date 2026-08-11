#!/usr/bin/env python3
"""Certify, from inside P's plane, that P sees exactly the declared allowlist.

    python3 tools/isolation_certificate.py --out out/evidence/crossing/certificate.json

This is the v2 requirement gate. ADR 0021 recast "the robot cannot see the
traffic police" as communication-domain isolation, and this is the measurement
that either supports it or does not: a node stood up in P's domain enumerates
its own graph and the result is compared against `RELAYED_TOPICS`.

WHAT MAKES A GREEN CERTIFICATE MEAN ANYTHING
--------------------------------------------
"P saw only the allowlist" is trivially true when nothing is publishing at all,
in a container without multicast, or with the RMW misconfigured. So the
certificate is never green on absence alone:

  * `/clock` must be present AND ADVANCING in P's plane. A frozen clock is the
    exact silent failure ADR 0020 documents -- every estimate in P's plane
    zeroes while the run looks healthy.
  * the robot plane must show STRICTLY MORE than the allowlist. That is the
    positive control: it proves the instrument can see topics at all and that
    there was something on the other side to be isolated from. Without it, a
    dead simulator certifies as perfect isolation.

If the positive control cannot be established the verdict is INCONCLUSIVE, not
green -- the same skip-never-pass rule the occlusion and domain-isolation gates
run under.

Built-in topics are excluded by an explicit, short list rather than a pattern.
`/rosout` and `/parameter_events` exist in every ROS graph because this node
itself creates them, so counting them as leaks would make the certificate
permanently and meaninglessly red.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

sys.path.insert(0, str(Path(__file__).parent.parent / "src/corridor_gateway"))
from corridor_gateway.domains import (  # noqa: E402
    POLICE_DOMAIN_ID,
    RELAYED_TOPICS,
    ROBOT_DOMAIN_ID,
)

#: Created by any ROS node, including this one. Not evidence of a leak.
NODE_LOCAL_TOPICS = frozenset({"/rosout", "/parameter_events"})

CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def observed_topics(node: Node) -> set[str]:
    """Every topic this node's domain shows it, minus what it created itself."""

    return {
        name for name, _types in node.get_topic_names_and_types()
    } - NODE_LOCAL_TOPICS


def classify(observed: set[str], declared: set[str]) -> dict:
    """Compare a graph against the allowlist. Pure, so it is testable without DDS."""

    unexpected = sorted(observed - declared)
    missing = sorted(declared - observed)
    return {
        "observed": sorted(observed),
        "declared": sorted(declared),
        "unexpected": unexpected,
        "missing": missing,
        "matches_exactly": not unexpected and not missing,
    }


def _watch(domain_id: int, name: str, settle_s: float, clock: bool) -> tuple[set[str], list[int]]:
    context = Context()
    context.init(domain_id=domain_id)
    node = Node(name, context=context)
    ticks: list[int] = []
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    if clock:
        node.create_subscription(
            Clock,
            "/clock",
            lambda m: ticks.append(m.clock.sec * 1_000_000_000 + m.clock.nanosec),
            CLOCK_QOS,
        )
    try:
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        return observed_topics(node), ticks
    finally:
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            context.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--police-domain", type=int, default=POLICE_DOMAIN_ID)
    parser.add_argument("--robot-domain", type=int, default=ROBOT_DOMAIN_ID)
    parser.add_argument("--settle", type=float, default=12.0)
    parser.add_argument("--label", default="nominal")
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--expect-red",
        action="store_true",
        help="mutation control: exit 0 only if the certificate comes back RED",
    )
    arguments = parser.parse_args()

    rclpy.init(args=None)
    try:
        police_topics, clock_ticks = _watch(
            arguments.police_domain, "isolation_certificate_police", arguments.settle, clock=True
        )
        robot_topics, _ = _watch(
            arguments.robot_domain, "isolation_certificate_robot", 5.0, clock=False
        )
    finally:
        rclpy.try_shutdown()

    declared = set(RELAYED_TOPICS)
    comparison = classify(police_topics, declared)

    clock_advancing = len(set(clock_ticks)) > 1
    control_sees_more = bool(robot_topics - declared)

    certificate = {
        "label": arguments.label,
        "domains": {"robot": arguments.robot_domain, "police": arguments.police_domain},
        "police_plane": comparison,
        "clock": {
            "messages": len(clock_ticks),
            "distinct_values": len(set(clock_ticks)),
            "advancing": clock_advancing,
        },
        "positive_control": {
            "robot_plane_topics": sorted(robot_topics),
            "robot_plane_shows_more_than_allowlist": control_sees_more,
        },
    }

    if not control_sees_more:
        certificate["verdict"] = "INCONCLUSIVE"
        certificate["reason"] = (
            "the robot plane showed nothing beyond the allowlist, so an isolation "
            "negative would be vacuous -- is the simulator running?"
        )
    elif comparison["matches_exactly"] and clock_advancing:
        certificate["verdict"] = "GREEN"
    else:
        certificate["verdict"] = "RED"
        certificate["reason"] = (
            f"unexpected={comparison['unexpected']} missing={comparison['missing']} "
            f"clock_advancing={clock_advancing}"
        )

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(certificate, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(certificate, indent=2))
    print(f"\nwritten: {destination}")
    print("VERDICT:", certificate["verdict"])

    if arguments.expect_red:
        # Mutation control: a green certificate here means the mutation was not
        # detected, which is the failure being tested for.
        return 0 if certificate["verdict"] == "RED" else 1
    return 0 if certificate["verdict"] == "GREEN" else 1


if __name__ == "__main__":
    sys.exit(main())
