#!/usr/bin/env python3
"""Measure what the gateway actually delivers from A's plane into P's plane.

    python3 tools/crossing_measure.py --seconds 60 --out out/evidence/crossing/640x360.json

Run it on system Jazzy with a live adapter on the robot domain and the gateway
running. It joins BOTH domains itself and needs no domain set in its own shell.

WHY IT SUBSCRIBES ON BOTH SIDES
-------------------------------
"Added latency" is a difference, so measuring one side cannot produce it. This
node subscribes to the same topic twice -- once on the robot domain, straight
from the publisher, and once on the police domain, through the gateway -- in one
process, and matches the two streams by header stamp. The delta between receipt
times for the SAME frame is the bridge's contribution, measured against one
wall clock on one host, with no assumption that simulation time tracks it.

Comparing a header stamp against wall time would not have worked: under
`use_sim_time` the stamp is simulation time, and Isaac's real-time factor is
neither 1.0 nor constant, so that subtraction measures the simulator's pacing
and reports it as transport delay.

Both contexts are constructed explicitly rather than using the default one:
rclpy's default context belongs to a single domain, and two domains in one
process need two.

The delivered-frame ratio is measured against the NOMINAL rate the contract
declares, not against what the publisher happened to emit. Dividing what
arrived by what was sent would score a bridge as perfect while the publisher
starved, which is precisely the failure this gate exists to catch -- so the
publisher's own count is reported alongside, and both are in the artifact.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, str(Path(__file__).parent))
from isaac_gpu import gpu_memory_snapshot  # noqa: E402

#: The adapter's contract simulation rate; the camera divider derives from it.
SIMULATION_HZ = 60.0

IMAGE_TOPIC = "/p_cam/image_raw"
CAMERA_INFO_TOPIC = "/p_cam/camera_info"
CLOCK_TOPIC = "/clock"

# Depth 200, not the contract's 5. This subscriber is an instrument, not a
# consumer: a queue that overflows while the measuring thread is servicing the
# other domain would be recorded as transport loss. Both planes get the SAME
# profile, which is what makes the two counts comparable at all -- the
# difference between them is then attributable to what sits between them.
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def stamp_ns(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class PlaneWatcher:
    """One domain's view of the stream."""

    def __init__(self, name: str, domain_id: int) -> None:
        self.name = name
        self.domain_id = domain_id
        self.context = Context()
        self.context.init(domain_id=domain_id)
        self.node = Node(f"crossing_watch_{name}", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.image_receipts: dict[int, float] = {}
        self.image_order: list[int] = []
        self.info_stamps: set[int] = set()
        self.clock_ns: list[int] = []
        self.frame_ids: set[str] = set()
        self.sizes: set[tuple[int, int]] = set()
        self.encodings: set[str] = set()

        self.node.create_subscription(Image, IMAGE_TOPIC, self._on_image, SENSOR_QOS)
        self.node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_info, SENSOR_QOS)
        self.node.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, CLOCK_QOS)

    def _on_image(self, message: Image) -> None:
        key = stamp_ns(message)
        # Wall time, deliberately: this is the only clock both planes share, and
        # the delta between planes is what is being measured.
        self.image_receipts.setdefault(key, time.monotonic())
        self.image_order.append(key)
        self.frame_ids.add(message.header.frame_id)
        self.sizes.add((message.width, message.height))
        self.encodings.add(message.encoding)

    def _on_info(self, message: CameraInfo) -> None:
        self.info_stamps.add(stamp_ns(message))

    def _on_clock(self, message: Clock) -> None:
        self.clock_ns.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec)

    def spin(self, seconds: float) -> None:
        self.executor.spin_once(timeout_sec=seconds)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()

    def non_monotonic_stamps(self) -> int:
        return sum(
            1
            for earlier, later in zip(self.image_order, self.image_order[1:], strict=False)
            if later <= earlier
        )

    def clock_summary(self) -> dict:
        if not self.clock_ns:
            return {"messages": 0, "advancing": False, "span_s": 0.0}
        span = (max(self.clock_ns) - min(self.clock_ns)) / 1e9
        return {
            "messages": len(self.clock_ns),
            "advancing": span > 0.0,
            "span_s": round(span, 3),
            "first_ns": min(self.clock_ns),
            "last_ns": max(self.clock_ns),
        }


def bridge_process(settle_s: float = 2.0) -> dict:
    """The domain_bridge process, if it is running, with a CPU sample."""

    try:
        listing = subprocess.check_output(["pgrep", "-af", "domain_bridge"], text=True)
    except subprocess.CalledProcessError:
        return {"running": False}
    pids = [
        int(line.split()[0])
        for line in listing.strip().splitlines()
        # pgrep matches its own shell when the pattern is on the command line.
        if "/domain_bridge/domain_bridge" in line or line.split()[1].endswith("domain_bridge")
    ]
    if not pids:
        return {"running": False}
    pid = pids[0]

    def cpu_seconds() -> float:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        # utime + stime, fields 14 and 15 of stat (1-based), in clock ticks.
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")

    first, wall_first = cpu_seconds(), time.monotonic()
    time.sleep(settle_s)
    second, wall_second = cpu_seconds(), time.monotonic()
    return {
        "running": True,
        "pid": pid,
        "cpu_percent_sample": round(
            100.0 * (second - first) / max(1e-9, wall_second - wall_first), 2
        ),
        "sample_window_s": settle_s,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--robot-domain", type=int, default=42)
    parser.add_argument("--police-domain", type=int, default=43)
    parser.add_argument("--rate-hz", type=float, default=15.0, help="declared nominal rate")
    parser.add_argument("--label", default="640x360")
    parser.add_argument(
        "--producer-manifest",
        help=(
            "The adapter's --drive-out schedule. Its per-update simulation times "
            "are the publisher's OWN record of what it rendered, reached without "
            "DDS, so the producer gate does not rest on a subscriber that can "
            "drop the very frames it is counting."
        ),
    )
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--delivery-floor", type=float, default=0.95, help="fraction of nominal required"
    )
    parser.add_argument("--latency-ceiling-ms", type=float, default=1000.0 / 15.0)
    arguments = parser.parse_args()

    rclpy.init(args=None)
    robot = PlaneWatcher("robot", arguments.robot_domain)
    police = PlaneWatcher("police", arguments.police_domain)

    gpu_name, vram_before, vram_total = gpu_memory_snapshot()
    # Sampled DURING the capture, not after it. The first version of this tool
    # took one reading at the end and recorded 611 MiB for a session whose
    # publisher had already exited -- a post-mortem number reported as a load
    # figure. The same mistake applied to bridge CPU.
    vram_samples: list[int] = [vram_before]
    bridge_samples: list[float] = []
    bridge = {"running": False}

    deadline = time.monotonic() + arguments.seconds
    next_sample = time.monotonic() + 5.0
    try:
        while time.monotonic() < deadline:
            robot.spin(0.005)
            police.spin(0.005)
            if time.monotonic() >= next_sample:
                next_sample = time.monotonic() + 5.0
                try:
                    _, used, _ = gpu_memory_snapshot()
                    vram_samples.append(used)
                except (subprocess.SubprocessError, OSError, ValueError):
                    pass
                sample = bridge_process(settle_s=0.5)
                if sample.get("running"):
                    bridge = sample
                    bridge_samples.append(sample["cpu_percent_sample"])
    finally:
        robot.close()
        police.close()
        rclpy.try_shutdown()

    nominal = arguments.rate_hz * arguments.seconds
    delivered = len(police.image_receipts)
    published = len(robot.image_receipts)
    # CameraInfo leaves the same render product on the same tick as the image,
    # so it counts FRAMES without being confounded by frame SIZE. Counted on
    # both planes, it is the schedule token that separates "the bridge dropped
    # images" from "large messages drop everywhere on this transport".
    published_info = len(robot.info_stamps)
    delivered_info = len(police.info_stamps)

    robot_span_s = robot.clock_summary().get("span_s") or 0.0
    police_span_s = police.clock_summary().get("span_s") or 0.0

    producer = {"manifest": arguments.producer_manifest}
    # The adapter writes its schedule when the drive ENDS, which is after this
    # capture window closes -- so the file is normally absent here and the
    # producer gate is filled in by tools/producer_gate.py once the adapter has
    # exited. Reading it here is the convenience path for a re-analysis.
    if arguments.producer_manifest and Path(arguments.producer_manifest).is_file():
        schedule = json.loads(Path(arguments.producer_manifest).read_text(encoding="utf-8"))
        samples = schedule.get("samples", [])
        # The graph publishes one frame every (skip + 1) updates; the adapter
        # derives that divider from the same contract this tool is checking.
        divider = max(1, round(SIMULATION_HZ / arguments.rate_hz))
        sim_span = (
            float(samples[-1]["sim_time_s"]) - float(samples[0]["sim_time_s"])
            if len(samples) > 1
            else 0.0
        )
        rendered_frames = len(samples) // divider
        producer.update(
            {
                "updates_completed": len(samples),
                "update_divider": divider,
                "frames_the_adapter_rendered": rendered_frames,
                "adapter_sim_span_s": round(sim_span, 3),
                "adapter_rate_hz_sim_basis": (
                    round(rendered_frames / sim_span, 3) if sim_span else None
                ),
            }
        )

    # Only frames seen on BOTH planes can contribute a difference.
    shared = sorted(set(robot.image_receipts) & set(police.image_receipts))
    added_ms = [
        1000.0 * (police.image_receipts[key] - robot.image_receipts[key]) for key in shared
    ]

    result = {
        "label": arguments.label,
        "seconds": arguments.seconds,
        "domains": {"robot": arguments.robot_domain, "police": arguments.police_domain},
        "nominal_rate_hz": arguments.rate_hz,
        "frames": {
            "nominal_expected": round(nominal, 1),
            "published_on_robot_plane": published,
            "delivered_in_police_plane": delivered,
            "delivered_ratio_of_nominal": round(delivered / nominal, 4) if nominal else 0.0,
            "delivered_ratio_of_published": (
                round(delivered / published, 4) if published else 0.0
            ),
            "matched_on_both_planes": len(shared),
            "camera_info_published_on_robot_plane": published_info,
            "camera_info_delivered": delivered_info,
            "camera_info_crossing_ratio": (
                round(delivered_info / published_info, 4) if published_info else 0.0
            ),
        },
        "stamp_monotonicity": {
            "police_plane_non_monotonic": police.non_monotonic_stamps(),
            "robot_plane_non_monotonic": robot.non_monotonic_stamps(),
        },
        "clock": {
            "police_plane": police.clock_summary(),
            "robot_plane": robot.clock_summary(),
        },
        "added_latency_ms": {
            "samples": len(added_ms),
            "median": round(statistics.median(added_ms), 3) if added_ms else None,
            "p95": (
                round(sorted(added_ms)[int(0.95 * (len(added_ms) - 1))], 3) if added_ms else None
            ),
            "max": round(max(added_ms), 3) if added_ms else None,
            "ceiling_ms": round(arguments.latency_ceiling_ms, 3),
        },
        "stream": {
            "frame_ids": sorted(police.frame_ids),
            "sizes": sorted(f"{w}x{h}" for w, h in police.sizes),
            "encodings": sorted(police.encodings),
        },
        "gpu": {
            "name": gpu_name,
            "vram_used_mib_at_start": vram_before,
            "vram_used_mib_peak_during_capture": max(vram_samples),
            "vram_used_mib_samples": len(vram_samples),
            "vram_total_mib": vram_total,
            "method": (
                "nvidia-smi --query-gpu=memory.used --id=0 (tools/isaac_gpu.py), "
                "sampled every 5 s DURING the capture"
            ),
        },
        "bridge": {
            **bridge,
            "cpu_percent_samples": bridge_samples,
            "cpu_percent_max": max(bridge_samples) if bridge_samples else None,
        },
        # The publisher's own liveness, because a short source makes the
        # delivered-vs-nominal ratio meaningless without it.
        "source_liveness": {
            "robot_plane_clock_span_s": robot_span_s,
            "police_plane_clock_span_s": police_span_s,
            # Named for what it is. This is the rate at which IMAGES ARRIVED at
            # a best-effort subscriber on A's plane -- not the rate the adapter
            # published at. Reading it as the latter is the error the rung-1
            # analysis caught.
            "image_arrival_rate_hz_robot_plane_sim_basis": (
                round(published / robot_span_s, 3) if robot_span_s else None
            ),
            "camera_info_arrival_rate_hz_robot_plane_sim_basis": (
                round(published_info / robot_span_s, 3) if robot_span_s else None
            ),
        },
        "producer": producer,
    }

    # --- gates, decomposed --------------------------------------------------
    # One combined "delivery" number cannot fail informatively: it mixes how
    # fast the adapter rendered, how much the transport lost, and how long the
    # source lived. Each of those is separately actionable, so each gets its own
    # gate and its own evidence.
    adapter_rate = producer.get("adapter_rate_hz_sim_basis")
    producer_ratio = (adapter_rate / arguments.rate_hz) if adapter_rate else None
    info_ratio = result["frames"]["camera_info_crossing_ratio"]
    image_ratio = result["frames"]["delivered_ratio_of_published"]

    # Attribution: CameraInfo and Image cross the same bridge on the same tick
    # and differ only in size. If the small stream crosses intact while the
    # large one does not, the loss is size-dependent transport behaviour on both
    # legs, not a bridge declining to forward.
    attribution = (
        "unattributed (no camera_info baseline)"
        if not published_info
        else (
            "size-dependent transport loss: the small stream crossed at "
            f"{info_ratio:.3f} while the large stream crossed at {image_ratio:.3f}"
            if info_ratio - image_ratio > 0.02
            else "bridge-attributable: both streams crossed at comparable ratios"
        )
    )

    result["gates"] = {
        "producer": {
            "definition": (
                "frames the adapter rendered per simulation second, from its own "
                "schedule, vs the declared rate"
            ),
            "measured_hz": adapter_rate,
            "declared_hz": arguments.rate_hz,
            "ratio": round(producer_ratio, 4) if producer_ratio else None,
            "floor": arguments.delivery_floor,
            "pass": bool(producer_ratio and producer_ratio >= arguments.delivery_floor),
            "evidence": arguments.producer_manifest or "NOT MEASURED (no --producer-manifest)",
        },
        "crossing": {
            "definition": "delivered in P's plane vs published on A's plane, same QoS both sides",
            "image_ratio": image_ratio,
            "camera_info_ratio": info_ratio,
            "floor": arguments.delivery_floor,
            "pass": image_ratio >= arguments.delivery_floor,
            "attribution": attribution,
        },
    }

    checks = {
        # The single "delivery vs nominal" number this replaced could not fail
        # informatively: it mixed the adapter's render rate, the transport's
        # loss, and how long the source lived. Its first run scored 0.37 purely
        # because the authored route finishes in ~24 s against a 60 s window,
        # and its second scored 0.79 because image arrivals were read as the
        # publisher's rate. Both were recorded as transport failures that had
        # not happened. The two gates below are separately actionable, and the
        # ratio against nominal is kept in `frames` as a number, not a verdict.
        "crossing_gate": result["gates"]["crossing"]["pass"],
        "stamps_monotonic_in_police_plane": (
            result["stamp_monotonicity"]["police_plane_non_monotonic"] == 0
        ),
        "clock_advancing_in_police_plane": police.clock_summary()["advancing"],
        "added_latency_under_one_camera_period": (
            added_ms is not None
            and len(added_ms) > 0
            and max(added_ms) < arguments.latency_ceiling_ms
        ),
        "camera_info_accompanies_images": result["frames"]["camera_info_delivered"] > 0,
    }
    # The producer gate is deliberately absent from `checks` when the adapter's
    # schedule does not exist yet, which is the normal case: the adapter writes
    # it at drive end, after this window closes. Counting an unmeasurable gate
    # as failed would report a producer fault on every run and train the reader
    # to ignore the verdict. tools/producer_gate.py adds it and recomputes.
    if result["gates"]["producer"].get("measured_hz") is not None:
        checks["producer_gate"] = result["gates"]["producer"]["pass"]
    else:
        result["gates"]["producer"]["pass"] = None
        result["gates"]["producer"]["status"] = "DEFERRED to tools/producer_gate.py"
    result["checks"] = checks
    result["pass"] = all(checks.values())

    destination = Path(arguments.out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(result, indent=2))
    print(f"\nwritten: {destination}")
    print("RESULT:", "PASS" if result["pass"] else "**FAIL**")
    for name, value in checks.items():
        if not value:
            print(f"  failed: {name}")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
