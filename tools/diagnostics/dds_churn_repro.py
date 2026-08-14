#!/usr/bin/env python3
"""Reproduce the deaf lens without Isaac: participant churn against Fast DDS SHM.

    # the SHM arm (default transport, the one every corridor run uses today)
    python3 tools/diagnostics/dds_churn_repro.py --domain 69 --out out/evidence/lens-deafness/shm

    # the UDP-only arm (fleet OI-13's profile)
    python3 tools/diagnostics/dds_churn_repro.py --domain 69 --out out/evidence/lens-deafness/udp \
        --profile ../robot-fleet/ground_station/fastdds_udp_only.xml

WHAT THIS IS. Hypothesis H1 of the 2026-08-14 bring-up rework: the deaf lens
is Fast DDS shared-memory transport poisoned by participant churn. The novel
churn ADR 0037 identified is simctl's `sim_target`/`probe_topics` helpers --
short-lived rclpy participants that subscribe to /scan and then `os._exit(0)`
every ~10 s, which is exactly the no-cleanup exit Fast DDS's own docs say
leaves zombie segments, lock files and mutexes behind (eProsima/Fast-DDS
#5053: after churn, a subscriber "successfully discovers and matches ... but
receives no data"; UDP unaffected). The fleet has already measured the end
state once: OI-13, "35 accumulated /dev/shm/fastrtps* segments made NEW
participants completely blind".

So: one 12 Hz /scan publisher, one long-lived victim subscriber wearing the
same matched-event instrumentation the lens now carries, and a churn loop of
subscribe-then-`os._exit(0)` children. If the victim goes silent while the
publisher provably advances, the deafness is reproduced -- in minutes, with
no Isaac load spent -- and the matched state at that moment says which KIND
of deaf (matched-but-silent = transport, never-matched = discovery). Run
once per arm; the two verdict files are the A/B.

CLEANUP IS NOT OPTIONAL. This tool manufactures zombie segments on a host
that runs other DDS sessions (the MicroROS work shares this box -- the
outage in _dds_shm.py's docstring is why "stale" must be earned). On exit it
unlinks only segments no live process has mapped, via the fleet's own
`_dds_shm.stale_segments()`, imported through the D5 resolver -- never
copied, never realpath'd.

The verdict JSON is the artifact. Deafness is a FINDING (exit 0); only a
measurement that could not run (publisher stalled, victim never matched) is
infrastructure (exit 2).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir))

SCAN_HZ = 12.0
STATUS_EVERY_S = 1.0
#: The victim is deaf when its count is static this long while the publisher
#: advances. 15 s is ~180 scans at 12 Hz -- far past any queue hiccup, far
#: short of the 60 s freeze the lens uses for its own (different) purpose.
SILENCE_DEAF_S = 15.0
#: And the publisher must have advanced by at least this many messages over
#: the same window, or the silence proves nothing (a dead publisher is
#: infrastructure, not deafness).
MIN_PUB_DELTA = 60

DENY_DOMAINS = {20, 42, 43, 44, 66, 68, 70}


def write_json(path: str, payload: dict) -> None:
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def read_json(path: str) -> dict | None:
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def deaf_at(timeline: list[dict], silence_s: float = SILENCE_DEAF_S,
            min_pub_delta: int = MIN_PUB_DELTA) -> dict | None:
    """First row where the victim is PROVABLY deaf. -> that row, or None.

    Provably: its count unchanged across a >= silence_s window in which the
    publisher advanced by >= min_pub_delta. A window where both stand still
    convicts the publisher, not the victim, and returns nothing.
    """

    for i, start in enumerate(timeline):
        for row in timeline[i + 1:]:
            if row["t"] - start["t"] < silence_s:
                continue
            if (row["sub"] == start["sub"]
                    and row["pub"] - start["pub"] >= min_pub_delta):
                return row
            break
    return None


# --------------------------------------------------------------- ROS roles
#
# Each role is its own process (the orchestrator re-invokes this file), so a
# churn child's `os._exit(0)` -- the entire point of a churn child -- cannot
# take anything else with it.


def _scan_msg(LaserScan):
    msg = LaserScan()
    msg.header.frame_id = "laser_frame"
    msg.angle_min, msg.angle_max = -3.14, 3.14
    msg.angle_increment = 6.28 / 360.0
    msg.range_min, msg.range_max = 0.05, 8.0
    msg.ranges = [2.0] * 360
    return msg


def run_publisher(args) -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("churn_repro_pub")
    pub = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    msg = _scan_msg(LaserScan)
    status = os.path.join(args.out, "pub.json")
    count, t0, last_status = 0, time.time(), 0.0
    while rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        count += 1
        now = time.time()
        if now - last_status >= STATUS_EVERY_S:
            write_json(status, {"pub": count, "t": round(now - t0, 1)})
            last_status = now
        time.sleep(1.0 / SCAN_HZ)
    return 0


def run_victim(args) -> int:
    import rclpy
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("churn_repro_victim")
    state = {"sub": 0, "matched_current": 0, "matched_total": 0}

    def on_scan(_msg):
        state["sub"] += 1

    def on_matched(info):
        state["matched_current"] = info.current_count
        state["matched_total"] = info.total_count
        print(f"victim: matched current={info.current_count} "
              f"total={info.total_count} change={info.current_count_change:+d}",
              flush=True)

    node.create_subscription(
        LaserScan, "/scan", on_scan, qos_profile_sensor_data,
        event_callbacks=SubscriptionEventCallbacks(matched=on_matched))

    status = os.path.join(args.out, "victim.json")
    t0, last_status = time.time(), 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        if now - last_status >= STATUS_EVERY_S:
            write_json(status, {**state, "t": round(now - t0, 1)})
            last_status = now
    return 0


def run_churn_child(args) -> int:
    """simctl's ros_eval shape: join, subscribe /scan, spin briefly, os._exit(0).

    The hard exit is the experiment. No rclpy.shutdown(), no context destroy:
    Fast DDS gets no chance to clean its SHM registration, exactly like
    simctl:287 and :1056.
    """

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node(f"churn_child_{os.getpid()}")
    node.create_subscription(LaserScan, "/scan", lambda m: None,
                             qos_profile_sensor_data)
    end = time.time() + args.child_life
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    os._exit(0)


# ------------------------------------------------------------ orchestration


def _shm_counts():
    from build_corridor_arena import yahboom_tools

    # Anchor the D5 walk at the resolver's own file, not at argv[0]: this
    # script lives one level deeper (tools/diagnostics/), and the default
    # anchor would walk one directory short of the fleet src.
    anchor = os.path.join(HERE, os.pardir, "build_corridor_arena.py")
    sys.path.insert(0, yahboom_tools(anchor))
    import _dds_shm
    return _dds_shm


def orchestrate(args) -> int:
    if args.domain in DENY_DOMAINS:
        print(f"REFUSED: domain {args.domain} is deny-listed for corridor "
              f"sessions (CLAUDE.md).", file=sys.stderr)
        return 2
    os.makedirs(args.out, exist_ok=True)
    dds = _shm_counts()

    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.domain)
    env["PYTHONNOUSERSITE"] = "1"
    for name in ("FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        env.pop(name, None)
    arm = "shm-default"
    if args.profile:
        profile = os.path.abspath(args.profile)
        if not os.path.isfile(profile):
            print(f"profile not found: {profile}", file=sys.stderr)
            return 2
        env["FASTDDS_DEFAULT_PROFILES_FILE"] = profile
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile
        arm = "udp-only"

    def child(role, *extra):
        return subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--role", role,
             "--domain", str(args.domain), "--out", args.out,
             "--child-life", str(args.child_life), *extra],
            env=env,
            stdout=open(os.path.join(args.out, f"{role}.log"), "a"),
            stderr=subprocess.STDOUT)

    shm_before = len(dds.all_segments())
    publisher = child("publisher")
    victim = child("victim")
    churn: list[subprocess.Popen] = []
    churn_spawned = 0
    timeline: list[dict] = []
    verdict_path = os.path.join(args.out, "repro.json")
    t0 = time.time()
    verdict = {
        "arm": arm, "domain": args.domain,
        "profile": env.get("FASTDDS_DEFAULT_PROFILES_FILE"),
        "churn_every_s": args.churn_every, "child_life_s": args.child_life,
        "silence_deaf_s": SILENCE_DEAF_S, "min_pub_delta": MIN_PUB_DELTA,
        "shm_segments_before": shm_before,
    }

    try:
        # The victim must demonstrably work once, or there is no experiment.
        deadline = time.time() + 30
        started = False
        while time.time() < deadline:
            v = read_json(os.path.join(args.out, "victim.json"))
            if v and v["sub"] > 0 and v["matched_current"] >= 1:
                started = True
                break
            time.sleep(0.5)
        if not started:
            verdict.update(outcome="infra_victim_never_started")
            write_json(verdict_path, verdict)
            print(f"{arm}: INFRA -- victim never received while matched; "
                  f"no experiment ran", flush=True)
            return 2

        last_churn, shm_peak = 0.0, shm_before
        while time.time() - t0 < args.duration:
            now = time.time()
            if now - last_churn >= args.churn_every:
                churn.append(child("churn-child"))
                churn_spawned += 1
                churn = [c for c in churn if c.poll() is None]
                last_churn = now
            p = read_json(os.path.join(args.out, "pub.json"))
            v = read_json(os.path.join(args.out, "victim.json"))
            if p and v:
                shm_now = len(dds.all_segments())
                shm_peak = max(shm_peak, shm_now)
                timeline.append({
                    "t": round(now - t0, 1), "pub": p["pub"], "sub": v["sub"],
                    "matched_current": v["matched_current"],
                    "matched_total": v["matched_total"], "shm": shm_now,
                })
                row = deaf_at(timeline)
                if row is not None:
                    verdict.update(
                        outcome="deaf", t_deaf_s=row["t"],
                        matched_at_deaf={"current": row["matched_current"],
                                         "total": row["matched_total"]},
                        deaf_kind=("matched-but-silent"
                                   if row["matched_current"] >= 1
                                   else "never-matched"))
                    break
                if len(timeline) >= 2:
                    a, b = timeline[0], timeline[-1]
                    if (b["t"] - a["t"] >= SILENCE_DEAF_S
                            and b["pub"] == a["pub"]):
                        verdict.update(outcome="infra_publisher_stalled")
                        break
            time.sleep(1.0)
        else:
            verdict.update(outcome="clean")
    finally:
        for proc in (*churn, victim, publisher):
            if proc.poll() is None:
                proc.terminate()
        time.sleep(1.0)
        for proc in (*churn, victim, publisher):
            if proc.poll() is None:
                proc.kill()
        # Sweep ONLY what no live process maps -- the fleet's earned "stale".
        removed = 0
        for segment in dds.stale_segments():
            try:
                os.unlink(segment)
                removed += 1
            except OSError:
                pass
        verdict.update(
            duration_s=round(time.time() - t0, 1),
            churn_spawned=churn_spawned,
            shm_segments_peak=shm_peak if timeline else shm_before,
            shm_stale_removed=removed,
            shm_segments_after=len(dds.all_segments()),
            timeline=timeline,
        )
        write_json(verdict_path, verdict)

    outcome = verdict.get("outcome", "aborted")
    print(f"{arm}: {outcome.upper()}"
          + (f" at t={verdict['t_deaf_s']}s ({verdict['deaf_kind']})"
             if outcome == "deaf" else "")
          + f" -- pub={timeline[-1]['pub'] if timeline else 0}"
            f" sub={timeline[-1]['sub'] if timeline else 0}"
            f" shm peak {verdict['shm_segments_peak']},"
            f" {verdict['shm_stale_removed']} stale removed"
            f" -> {verdict_path}", flush=True)
    return 0 if outcome in ("deaf", "clean") else 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 allow_abbrev=False)
    ap.add_argument("--role", default="orchestrate",
                    choices=["orchestrate", "publisher", "victim", "churn-child"])
    ap.add_argument("--domain", type=int, default=69)
    ap.add_argument("--out", required=True)
    ap.add_argument("--profile", default="",
                    help="FastDDS XML profile for ALL participants (the "
                         "UDP-only arm); default is the default transport")
    ap.add_argument("--duration", type=float, default=900.0)
    ap.add_argument("--churn-every", type=float, default=2.0)
    ap.add_argument("--child-life", type=float, default=2.0)
    args = ap.parse_args()

    if args.role == "publisher":
        return run_publisher(args)
    if args.role == "victim":
        return run_victim(args)
    if args.role == "churn-child":
        return run_churn_child(args)
    return orchestrate(args)


if __name__ == "__main__":
    sys.exit(main())
