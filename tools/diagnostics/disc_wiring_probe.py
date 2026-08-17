#!/usr/bin/env python3
"""T1: does the disc declaration actually reach the real governor node?

    ROS_DOMAIN_ID=69 python3 tools/diagnostics/disc_wiring_probe.py \
        --json out/evidence/creep-bench/wiring-probe.json

Between the bench (T0, no ROS at all) and Isaac (T3/T4, ~25 minutes a cycle)
there is a gap the bench cannot cover: topic names, QoS, namespaces, message
field order, and whether the governor node was even built from the source being
edited. Every one of those failures looks identical from the outside -- the
robot does not move -- and each one cost an Isaac cycle to find.

This runs the REAL `cmd_vel_governor` node in-process against a synthetic scan
of the authored corridor and asks the only question that matters: with a disc
declared, does forward motion come out of /cmd_vel? And with nothing declared,
does it correctly stop?

    driver ---> /scan (best effort)          ---> [ real cmd_vel_governor ] ---> /cmd_vel
           ---> /cmd_vel_raw (0.05 m/s)                      ^                       |
           ---> /cmd_vel_governor/docking_disc --------------+                       v
                                                                            probe reads it back

Not a substitute for T3. It proves the wiring, not the physics: there is no
contact here, no slip, and the robot does not move. A green probe and a red
Isaac run means the problem is in the simulator or the stack around it, which
is exactly the split this tier exists to make.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

# NO `.resolve()`, NO `realpath` (D5) -- see `creep_bench.py` for the incident.
#
# Two different path questions here, and conflating them is the trap. IMPORTING
# this repo's own modules can use the physical path, because it is the same
# file either way. RESOLVING THE FLEET cannot: that walk must stay logical or it
# escapes the symlink into ~/Development.
sys.path.insert(0, os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import build_corridor_arena as _layout  # noqa: E402

# `fleet_src_root()` walks `tools/<file> -> tools -> checkout -> src`, so it must
# be handed a path at `tools/` level. This file sits one deeper, in
# `tools/diagnostics/`, and letting it default walked to the checkout and
# reported the fleet missing at `<checkout>/yahboomcar-ros2`. Anchored
# explicitly rather than by counting `..` at the call site.
_HERE = _layout.this_file()                       # logical, from argv[0]
_TOOLS_DIR = _layout.logical_abspath(
    os.path.join(os.path.dirname(_HERE), os.pardir))
FLEET_SRC = _layout.fleet_src_root(
    os.path.join(_TOOLS_DIR, "build_corridor_arena.py"))

REPO = Path(_layout.logical_abspath(os.path.join(_TOOLS_DIR, os.pardir)))
sys.path.insert(0, str(REPO / "tools"))

#: The two scenarios, and the contact geometry they are built around.
B_RADIUS_M = 0.12
#: A's standoff for the probe: inside the 0.35 m stop, where the cone leaks and
#: the disc must not. This is the range at which last night's runs pinned.
PROBE_RANGE_M = 0.34
CREEP_MPS = 0.05
#: Long enough for the governor's 20 Hz tick and a scan to have been seen.
SETTLE_S = 2.0
OBSERVE_S = 2.0


def _fleet_governor():
    pkg = Path(FLEET_SRC) / "yahboomcar-ros2" / "yahboomcar_safety"
    if not pkg.is_dir():
        raise SystemExit(
            f"fleet safety package not found at {pkg}\n"
            "  run from the symlinked path, or set CORRIDOR_FLEET_SRC"
        )
    sys.path.insert(0, str(pkg))
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        from yahboomcar_safety.cmd_vel_governor import CmdVelGovernor
    finally:
        sys.dont_write_bytecode = previous
    return CmdVelGovernor


def _disc_scan(centre_range, radius, beams=360, background=5.0):
    """A full-circle scan of a cylinder dead ahead. Exact ray-circle solution."""

    ranges = [background] * beams
    for i in range(beams):
        angle = -math.pi + i * (2 * math.pi / beams)
        b = math.cos(angle) * centre_range
        c = centre_range * centre_range - radius * radius
        disc = b * b - c
        if disc < 0:
            continue
        hit = b - math.sqrt(disc)
        if 0.0 < hit < ranges[i]:
            ranges[i] = hit
    return ranges


def run(declare: bool) -> dict:
    import rclpy
    from corridor_nav_gate import GOVERNOR_CMD_TOPIC, GOVERNOR_DISC_TOPIC
    from geometry_msgs.msg import Twist, Vector3Stamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    governor_class = _fleet_governor()
    ranges = _disc_scan(PROBE_RANGE_M, B_RADIUS_M)

    class Driver(Node):
        def __init__(self):
            super().__init__("disc_wiring_driver")
            self.scan_pub = self.create_publisher(
                LaserScan, "/scan", qos_profile_sensor_data)
            self.cmd_pub = self.create_publisher(Twist, GOVERNOR_CMD_TOPIC, 10)
            self.disc_pub = self.create_publisher(
                Vector3Stamped, GOVERNOR_DISC_TOPIC, 10)
            self.governed = []
            self.create_subscription(Twist, "/cmd_vel", self._on_governed, 10)
            self.create_timer(0.05, self._tick)
            self.observing = False

        def _on_governed(self, message: Twist):
            if self.observing:
                self.governed.append(float(message.linear.x))

        def _tick(self):
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = "laser_frame"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / len(ranges)
            scan.range_min, scan.range_max = 0.12, 8.0
            scan.ranges = [float(r) for r in ranges]
            self.scan_pub.publish(scan)

            command = Twist()
            command.linear.x = CREEP_MPS
            self.cmd_pub.publish(command)

            if declare:
                disc = Vector3Stamped()
                disc.header.stamp = scan.header.stamp
                disc.vector.x = 0.0
                disc.vector.y = PROBE_RANGE_M
                disc.vector.z = B_RADIUS_M
                self.disc_pub.publish(disc)

    rclpy.init()
    try:
        governor = governor_class()
        driver = Driver()
        executor = SingleThreadedExecutor()
        executor.add_node(governor)
        executor.add_node(driver)

        deadline = time.monotonic() + SETTLE_S
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        driver.observing = True
        deadline = time.monotonic() + OBSERVE_S
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        samples = list(driver.governed)
        governor.destroy_node()
        driver.destroy_node()
    finally:
        rclpy.shutdown()

    moving = [v for v in samples if v > 1e-6]
    return {
        "declared": declare,
        "samples": len(samples),
        "moving": len(moving),
        "duty": (None if not samples else round(len(moving) / len(samples), 3)),
        "max_governed_vx": (None if not samples else round(max(samples), 4)),
        "range_m": PROBE_RANGE_M,
        "target_radius_m": B_RADIUS_M,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", type=Path)
    arguments = parser.parse_args()

    domain = os.environ.get("ROS_DOMAIN_ID", "<unset>")
    print(f"disc wiring probe, ROS_DOMAIN_ID={domain}")
    if domain in {"20", "42", "43", "44", "66", "68", "70"}:
        raise SystemExit(f"refusing to run on reserved domain {domain}")

    declared = run(declare=True)
    print(f"  with disc     : duty {declared['duty']} over {declared['samples']} "
          f"samples, max vx {declared['max_governed_vx']}")
    control = run(declare=False)
    print(f"  without (ctrl): duty {control['duty']} over {control['samples']} "
          f"samples, max vx {control['max_governed_vx']}")

    # The probe passes only if BOTH halves behave: motion with the declaration
    # and none without it. A green half on its own proves nothing -- a governor
    # that passes everything through would satisfy the first test and be broken.
    passed = (
        declared["samples"] > 0 and declared["duty"] == 1.0
        and control["samples"] > 0 and control["moving"] == 0
    )
    print(f"  VERDICT: {'PASS' if passed else 'FAIL'}")

    if arguments.json:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json.dumps(
            {"passed": passed, "declared": declared, "control": control},
            indent=2) + "\n", encoding="utf-8")
        print(f"  written: {arguments.json}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
