#!/usr/bin/env python3
"""Exit when the Nav2 servers' get_state services are discoverable. The event
that replaces the lifecycle manager's 5 s TimerAction.

    python3 tools/nav_ready_waiter.py --nodes controller_server planner_server \
        behavior_server bt_navigator --timeout 30

WHY. robot1_nav_corridor_launch.py holds the history: the lifecycle manager
calls `get_state` on services that may not be discoverable yet, does not
retry, and aborts the entire bring-up -- 7 of 27 runs on 2026-08-13 (26%).
The 5 s TimerAction removed the failure by waiting longer than the race;
this waits for THE CONDITION the race is about -- all four `get_state`
services announced -- and typically clears in ~1-2 s.

EXIT 0 ALWAYS (ready or timeout), with one line saying which. The manager
must start either way: a timeout here means bring-up is already sick, and
the runner's lifecycle deadline plus retry-once own that failure. A waiter
that could veto the manager would be a second, worse deadline.
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--nodes", nargs="+", required=True)
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    import rclpy
    from lifecycle_msgs.srv import GetState

    rclpy.init()
    node = rclpy.create_node("nav_ready_waiter")
    clients = {name: node.create_client(GetState, f"/{name}/get_state")
               for name in args.nodes}
    t0 = time.time()
    pending = set(args.nodes)
    while pending and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        pending = {name for name in pending
                   if not clients[name].service_is_ready()}
    elapsed = time.time() - t0
    if pending:
        print(f"nav_ready_waiter: TIMEOUT after {elapsed:.1f}s, still absent: "
              f"{sorted(pending)} -- starting the manager anyway", flush=True)
    else:
        print(f"nav_ready_waiter: all {len(args.nodes)} get_state services "
              f"discoverable in {elapsed:.1f}s", flush=True)
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
