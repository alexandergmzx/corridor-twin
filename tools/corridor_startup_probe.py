#!/usr/bin/env python3
"""Who commanded that? The first seconds of a run, recorded rather than argued.

    python3 tools/corridor_startup_probe.py --seconds 60 --out startup.json

A is observed turning on the spot before it has a goal. Three explanations have
been offered for that across two sessions -- a stale `behavior_server` left on
the domain, Nav2's stock recovery firing into empty costmaps, and the controller
itself -- and none of them has ever been checked against a log, because nothing
in either repository subscribes to `/behavior_tree_log` and `/cmd_vel_raw` was
recorded only inside a session bag nobody opened for this question.

So this records both, live, with the goal-send moment marked. It commands
nothing.

WHAT IT ANSWERS
---------------
* Was anything commanded BEFORE the goal was active? That alone separates "Nav2
  is recovering" from "something else is driving".
* If yes, was it rotation, and how much?
* Which behaviour-tree node was RUNNING when it happened? `Spin`, `BackUp` and
  `Wait` are named nodes in the stock recovery subtree, so the log says which
  one rather than leaving it to be inferred from the shape of the motion.

It deliberately does NOT decide. The verdict belongs in the artifact.
"""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#: Below this a command is noise, not motion. The governor's own floor for
#: rotation is 0.2 rad/s (nav2_robot1_corridor.yaml min_rotational_vel), so
#: anything at or above a tenth of that was meant.
MOVING_MPS = 0.01
MOVING_RAD_S = 0.02


class StartupProbe(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("corridor_startup_probe")
        self.t0 = time.monotonic()
        self.commands: list[dict] = []
        self.transitions: list[dict] = []
        self.goal_at_s: float | None = None

        self.create_subscription(
            Twist, f"{namespace}/cmd_vel_raw", self._on_command, 50
        )
        try:
            from nav2_msgs.msg import BehaviorTreeLog

            self.create_subscription(
                BehaviorTreeLog, f"{namespace}/behavior_tree_log", self._on_bt, 50
            )
            self.bt_available = True
        except ImportError:
            # Recorded rather than fatal: the cmd_vel half is the half that says
            # whether anything moved at all, and it is worth having alone.
            self.bt_available = False

    def _elapsed(self) -> float:
        return round(time.monotonic() - self.t0, 3)

    def _on_command(self, message: Twist) -> None:
        linear, angular = message.linear.x, message.angular.z
        moving = abs(linear) >= MOVING_MPS or abs(angular) >= MOVING_RAD_S
        self.commands.append({
            "t": self._elapsed(),
            "linear_mps": round(linear, 4),
            "angular_rad_s": round(angular, 4),
            "moving": moving,
            "before_goal": self.goal_at_s is None,
        })

    def _on_bt(self, message) -> None:
        for event in message.event_log:
            self.transitions.append({
                "t": self._elapsed(),
                "node": event.node_name,
                "from": event.previous_status,
                "to": event.current_status,
                "before_goal": self.goal_at_s is None,
            })


def summarise(probe: StartupProbe) -> dict:
    before = [row for row in probe.commands if row["before_goal"] and row["moving"]]
    rotation = [row for row in before if abs(row["angular_rad_s"]) >= MOVING_RAD_S]
    # Integrated, not counted: one stray sample is noise and a sustained stream
    # is a pirouette, and only the integral tells them apart.
    turned_rad = 0.0
    for earlier, later in zip(probe.commands, probe.commands[1:], strict=False):
        if not earlier["before_goal"]:
            break
        turned_rad += earlier["angular_rad_s"] * (later["t"] - earlier["t"])

    running_before = sorted({
        row["node"] for row in probe.transitions
        if row["before_goal"] and row["to"] == "RUNNING"
    })
    recovery = sorted({
        row["node"] for row in probe.transitions
        if row["to"] == "RUNNING"
        and row["node"] in {"Spin", "BackUp", "Wait", "RecoveryFallback",
                            "ClearLocalCostmap-Subtree", "ClearGlobalCostmap-Subtree",
                            "RecoveryActions", "NavigateRecovery"}
    })
    return {
        "goal_at_s": probe.goal_at_s,
        "behavior_tree_log_available": probe.bt_available,
        "commands_total": len(probe.commands),
        "commands_before_goal": len([r for r in probe.commands if r["before_goal"]]),
        "moving_commands_before_goal": len(before),
        "rotating_commands_before_goal": len(rotation),
        "commanded_rotation_before_goal_rad": round(turned_rad, 4),
        "commanded_rotation_before_goal_deg": round(turned_rad * 57.29577951, 2),
        "peak_angular_before_goal_rad_s": (
            round(max((abs(r["angular_rad_s"]) for r in before), default=0.0), 4)
        ),
        "bt_nodes_running_before_goal": running_before,
        "recovery_nodes_seen": recovery,
        # The acceptance U2 states, evaluated here so the artifact carries the
        # verdict rather than a reader recomputing it.
        "zero_rotation_before_goal": not rotation,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--goal-marker", type=Path,
                        help="a file whose appearance marks the goal being sent")
    parser.add_argument("--ready-marker", type=Path,
                        help="written once the subscriptions exist, so the caller "
                             "can wait rather than race the goal")
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args()

    rclpy.init()
    probe = StartupProbe(arguments.namespace)

    stopping = False

    def _stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    # A probe that comes up AFTER the goal cannot answer the question it was
    # built for. On its first live run it reported goal_at_s = 0.081, which is
    # not "the goal was sent 81 ms in" -- it is "the marker was already there".
    if arguments.ready_marker:
        arguments.ready_marker.parent.mkdir(parents=True, exist_ok=True)
        arguments.ready_marker.write_text("ready\n", encoding="utf-8")

    end = time.monotonic() + arguments.seconds
    while time.monotonic() < end and not stopping:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if (
            probe.goal_at_s is None
            and arguments.goal_marker
            and arguments.goal_marker.exists()
        ):
            probe.goal_at_s = probe._elapsed()

    report = {
        "seconds_requested": arguments.seconds,
        "observed_s": round(time.monotonic() - (end - arguments.seconds), 2),
        "summary": summarise(probe),
        "commands": probe.commands,
        "bt_transitions": probe.transitions,
    }
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(report["summary"], indent=2))
    print(f"\nwritten: {arguments.out}")

    probe.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
