#!/usr/bin/env python3
"""SLAM lens: a live browser view of map + scan + poses, ALIGNED, with the
four numbers that convict a bad mapping session while it is still running.

    ./tools/slam_lens.py                     # sim default (domain 66), port 8765
    ./tools/slam_lens.py --domain 68 --sim-time   # watching a replay
    then open  http://localhost:8765/

WHAT IT SHOWS (all in the SLAM map frame, one canvas):
  * the occupancy grid as slam_toolbox publishes it
  * the latest scan's endpoints at their TF-resolved pose, colored hit/miss
    against the map — misalignment is directly visible, not inferred
  * the TF pose (map->base: SLAM's opinion), a ground-truth ghost (aligned
    once at start), and a pure-odometry ghost (odom->base under the
    map->odom captured at start, i.e. the pose if SLAM never corrected)
  * live metrics with history: scan-to-map fit, pose divergence vs truth,
    /odom_raw-vs-truth yaw-rate ratio, scan staleness, TF resolve health

WHY IT EXISTS: the 2026-08-09 isaac --fun session produced a smeared map
with FIVE contributing failure layers (render-pacing runaway, the encoder
yaw lie, queue-full TF drops, shoved fun boxes, and an RViz frame artifact)
and not one of them was visible in RViz while it happened. Each of the four
metrics here corresponds to one measured failure; the metric module
(tools/_slam_lens_core.py) documents which.

DESIGN NOTES, deliberately copied from fleet-console (the console's
"SLAM map view + lidar overlay" milestone is where this ports to later):
  * subscriptions are BEST_EFFORT sensor QoS regardless of the publisher's
    offer (fleet OI-20: BEST_EFFORT matches any offer; the reverse starves)
    — except /map, which is latched TRANSIENT_LOCAL/RELIABLE by slam_toolbox
    and needs the matching subscription.
  * SingleThreadedExecutor, and callbacks only store-and-stamp; all metric
    computation happens in the 5 Hz snapshot loop. fleet-console measured
    MultiThreaded at 102.5% CPU vs 8.8% on this exact workload class, and a
    per-callback reduction starving the send loop through the GIL.
  * the browser gets COALESCED SNAPSHOTS at a fixed rate over one WebSocket,
    never per-message forwarding. The map ships only when its seq changes.

READ-ONLY BY CONSTRUCTION: subscribes and looks up TF, publishes nothing,
so it can watch any session — sim, replay, or (from the bench, domain given
explicitly) the real car — without being able to disturb it.

Dependencies: rclpy + tf2_ros from the sourced fleet env, numpy, and the
`websockets` package (present system-wide; FastAPI/aiohttp are not, and a
diagnostic tool should not grow a venv). The page is plain canvas + JS
served by this same process: one port, no build step.
"""
import argparse
import asyncio
import hashlib
import http
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _lens_core import (                                        # noqa: E402
    PoseAligner, StalenessTracker, TruthHistory, YawRatioWindow,
    divergence, occupied_mask_dilated, rle_encode, scan_endpoints,
    scan_map_fit, transform_points)

# The content-lag tile is DROPPED, not ported. It scores the scan against
# `segments_room()`, which is the fleet's stock 4x4 m arena and not this
# corridor, so every offset it produced here was computed against the wrong
# geometry -- and the tell (a large lag_rms) is easy to miss. A metric that
# cannot be right in this scene should not be on the page.
WALL_SEGS = None

# Corridor additions live one directory up.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'corridor_lens.html')
SNAPSHOT_HZ = 5.0
HISTORY_LEN = 1500              # 5 min at SNAPSHOT_HZ
TF_WINDOW = 100                 # snapshots in the TF-health ratio window

# The history row, named ONCE. The sampler built this row from a literal list
# and the dump named the columns in a second literal, and when the content-lag
# tile was removed only the metric was deleted -- both literals kept asking for
# 'lag_s'. The sampler then raised KeyError on its first tick, which killed the
# task that fills `latest['state']`, so the lens served one frozen frame for
# the whole run and the --dump wrote nothing because `history` never grew.
#
# The lens is mandatory equipment (CLAUDE.md, "watch the run, do not autopsy
# it"). It had never worked. One tuple, two consumers, and a test that this
# tuple is a subset of what build_state actually emits.
#
# 't' comes from the snapshot itself; the rest are keys of state['metrics'].
HISTORY_COLUMNS = ('t', 'fit', 'div_pos', 'yaw_ratio', 'stale_run')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _r3(x):
    return None if x is None else round(float(x), 3)


class LensNode:
    """Owns the rclpy node, subscriptions and TF buffer. Thread-safe state."""

    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        import tf2_ros

        self.args = args
        self.lock = threading.Lock()
        self.node = Node('slam_lens', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=bool(args.sim_time))])

        # ---- stored state (written by callbacks under lock) ----
        self.scans = deque(maxlen=4)          # (msg, rx wall time), newest last
        self.tf_pending = deque(maxlen=64)    # (stamp msg, rx) awaiting their verdict
        self.map_msg = None
        self.map_seq = 0
        self.truth = None
        self.odom = None
        self.counts = {'scan': 0, 'map': 0, 'truth': 0, 'odom': 0, 'odom_raw': 0}
        self.t0 = time.time()
        self.stale = StalenessTracker()
        self.yaw_win = YawRatioWindow()
        self.truth_hist = TruthHistory()
        # Corridor additions. The detector is the one the MISSION runs, imported
        # rather than reimplemented, so what the page shows is what A decides on.
        self.detector = None
        self._landmark = None
        self._landmark_map = None
        self.truth_markers = {}
        self.truth_align = PoseAligner()      # truth frame -> map frame
        self.odom_align = PoseAligner()       # odom frame  -> map frame (frozen at t0)
        self.tf_results = deque(maxlen=TF_WINDOW)
        self.tf_fail_streak = 0
        self._mask_cache = (None, None)       # (map_seq, dilated mask)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.node, spin_thread=False)

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        s = qos_profile_sensor_data

        self.node.create_subscription(OccupancyGrid, args.map_topic, self._on_map, latched)
        self.node.create_subscription(LaserScan, args.scan_topic, self._on_scan, s)
        self.node.create_subscription(Odometry, args.truth_topic, self._on_truth, s)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, s)
        self.node.create_subscription(Odometry, '/odom_raw', self._on_odom_raw, s)

        self._rclpy = rclpy

    # ---- callbacks: store and stamp, nothing else -------------------------
    def _on_map(self, msg):
        with self.lock:
            self.map_msg = msg
            self.map_seq += 1
            self.counts['map'] += 1

    def _on_scan(self, msg):
        digest = hashlib.blake2b(
            np.asarray(msg.ranges, dtype=np.float32).tobytes(), digest_size=16).digest()
        now = time.time()
        with self.lock:
            self.scans.append((msg, now))
            self.tf_pending.append((msg.header.stamp, now))
            self.counts['scan'] += 1
            self.stale.feed(digest)

    def _on_truth(self, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.counts['truth'] += 1
            self.yaw_win.feed_truth_yaw(time.time(), self.truth[2])
            self.truth_hist.feed(stamp, self.truth)

    def _on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.counts['odom'] += 1

    def _on_odom_raw(self, msg):
        with self.lock:
            self.counts['odom_raw'] += 1
            self.yaw_win.feed_odom(time.time(), msg.twist.twist.angular.z)

    # ---- TF ----------------------------------------------------------------
    def _landmark_payload(self):
        """What the detector currently believes, including its near misses.

        `candidates_this_frame` matters as much as the confirmation: a page that
        only showed confirmed hits would say nothing while the detector was
        busy fitting circles to a wall.
        """

        v = self._landmark
        if v is None:
            return {'armed': self.detector is not None}
        c = v.get('candidate')
        return {
            'armed': True,
            'confirmed': bool(v.get('confirmed')),
            'candidates': v.get('candidates_this_frame', 0),
            'frames_agreeing': v.get('frames_agreeing', 0),
            'range_m': None if not c else round(c['range_m'], 3),
            'bearing_deg': None if not c else round(math.degrees(c['bearing_rad']), 1),
            'fitted_radius_m': None if not c else c['fitted_radius_m'],
            'residual_m': None if not c else c['residual_m'],
            'points': None if not c else c['points'],
            'map_xy': None if not self._landmark_map else [round(v, 3) for v in self._landmark_map],
        }

    def _lookup(self, target, source, stamp=None):
        """-> (x, y, yaw) or None. Zero timeout: the snapshot loop never blocks."""
        try:
            t = self.tf_buffer.lookup_transform(
                target, source,
                stamp if stamp is not None else self._rclpy.time.Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, yaw_of(q))

    # ---- the snapshot ------------------------------------------------------
    def build_state(self):
        """Compute everything for one snapshot. Called from the asyncio loop."""
        with self.lock:
            scans = list(self.scans)
            map_msg = self.map_msg
            map_seq = self.map_seq
            truth = self.truth
            odom = self.odom
            counts = dict(self.counts)
            stale_run = self.stale.current_run
            stale_max = self.stale.max_run
            stale_frac = self.stale.duplicate_fraction
            yaw_ratio, yaw_n = self.yaw_win.ratio()

        now = time.time()
        base = self.args.base_frame

        pose = self._lookup(self.args.map_frame, base)

        # TF health, scored the way the CONSUMER experiences it: a scan's
        # stamp is only judged unresolvable once it is older than the grace
        # a message filter would give it (transform_timeout 0.2 s + one EKF
        # period). Judging the newest scan immediately over-counted failures
        # 4:1 on a healthy 2D session [measured tonight] because a fresh
        # scan's odom TF simply hasn't arrived yet — that is latency, not
        # unavailability.
        TF_GRACE_S = 0.35
        with self.lock:
            due = []
            while self.tf_pending and now - self.tf_pending[0][1] >= TF_GRACE_S:
                due.append(self.tf_pending.popleft()[0])
        for stamp_msg in due:
            ok = self._lookup(self.args.map_frame, self.args.base_frame,
                              self._rclpy.time.Time.from_msg(stamp_msg)) is not None
            self.tf_results.append(ok)
            self.tf_fail_streak = 0 if ok else self.tf_fail_streak + 1

        # Render/score the newest scan that resolves at its own stamp; fall
        # back to the newest scan at latest TF (flagged) so the display never
        # freezes just because the freshest stamp is still in its TF grace.
        scan_payload = None
        fit = None
        if scans:
            scan, scan_rx, tf_pose, resolved = None, 0.0, None, False
            for msg, rx in reversed(scans):
                p = self._lookup(self.args.map_frame, msg.header.frame_id,
                                 self._rclpy.time.Time.from_msg(msg.header.stamp))
                if p is not None:
                    scan, scan_rx, tf_pose, resolved = msg, rx, p, True
                    break
            if scan is None:
                scan, scan_rx = scans[-1]
                tf_pose = self._lookup(self.args.map_frame, scan.header.frame_id)

            # LANDMARK DETECTION, on the newest scan. This is the tile the
            # corridor needed and the fleet lens has no equivalent of: it shows
            # what the detector believes it is looking at, so a PHANTOM is
            # visible while it is happening rather than in a JSON afterwards.
            # A phantom at 0.910 m once re-aimed a whole mission.
            if self.detector is not None:
                newest = scans[-1][0]
                verdict = self.detector.feed(
                    newest.ranges, newest.angle_min, newest.angle_increment,
                    newest.range_min, newest.range_max)
                with self.lock:
                    self._landmark = verdict
                    if verdict.get('confirmed') and tf_pose is not None:
                        c = verdict['candidate']
                        cos_y, sin_y = math.cos(tf_pose[2]), math.sin(tf_pose[2])
                        self._landmark_map = [
                            tf_pose[0] + c['x'] * cos_y - c['y'] * sin_y,
                            tf_pose[1] + c['x'] * sin_y + c['y'] * cos_y,
                        ]

            if tf_pose is not None:
                pts_l = scan_endpoints(scan.ranges, scan.angle_min,
                                       scan.angle_increment, scan.range_min,
                                       scan.range_max)
                pts_m = transform_points(pts_l, tf_pose)
                hits = np.zeros(pts_m.shape[0], dtype=bool)
                if map_msg is not None:
                    seq, mask = self._mask_cache
                    if seq != map_seq:
                        h, w = map_msg.info.height, map_msg.info.width
                        grid = np.asarray(map_msg.data, dtype=np.int8).reshape(h, w)
                        mask = occupied_mask_dilated(grid)
                        self._mask_cache = (map_seq, mask)
                    fit, hits = scan_map_fit(
                        pts_m, mask, map_msg.info.resolution,
                        map_msg.info.origin.position.x,
                        map_msg.info.origin.position.y)
                scan_payload = {
                    'age': _r3(now - scan_rx),
                    'resolved': resolved,
                    'points': np.round(pts_m, 3).tolist(),
                    'hits': hits.astype(int).tolist(),
                }

        # Ghosts. Both aligners lock their anchor on the first snapshot where
        # the needed pair exists; everything after is relative motion.
        truth_ghost = None
        if pose is not None and truth is not None:
            self.truth_align.feed(pose, truth)
            truth_ghost = self.truth_align.truth_in_map(truth)
        odom_ghost = None
        if pose is not None and odom is not None:
            self.odom_align.feed(pose, odom)
            odom_ghost = self.odom_align.truth_in_map(odom)

        div_pos = div_yaw = None
        if pose is not None and truth_ghost is not None:
            div_pos, div_yaw = divergence(pose, truth_ghost)

        dt = max(1e-6, now - self.t0)
        tf_ok = (sum(self.tf_results) / len(self.tf_results)) if self.tf_results else None

        state = {
            't': _r3(now - self.t0),
            'rates': {k: _r3(v / dt) for k, v in counts.items()},
            'pose': None if pose is None else [_r3(v) for v in pose],
            'truth_ghost': None if truth_ghost is None else [_r3(v) for v in truth_ghost],
            'odom_ghost': None if odom_ghost is None else [_r3(v) for v in odom_ghost],
            'scan': scan_payload,
            'metrics': {
                'fit': _r3(fit),
                'div_pos': _r3(div_pos),
                'div_yaw': _r3(div_yaw),
                'yaw_ratio': _r3(yaw_ratio),
                'yaw_n': yaw_n,
                'stale_run': stale_run,
                'stale_max': stale_max,
                'stale_frac': _r3(stale_frac),
                'tf_ok_frac': _r3(tf_ok),
                'tf_fail_streak': self.tf_fail_streak,
            },
            # Everything the corridor needs that the fleet lens does not carry.
            'landmark': self._landmark_payload(),
            'truth_markers': self.truth_markers,
            'map_seq': map_seq,
        }
        map_payload = None
        if map_msg is not None:
            i = map_msg.info
            map_payload = {
                'seq': map_seq, 'w': i.width, 'h': i.height,
                'res': i.resolution,
                'ox': _r3(i.origin.position.x), 'oy': _r3(i.origin.position.y),
                'rle': rle_encode(map_msg.data),
            }
        return state, map_payload


async def serve(node: LensNode, args):
    import websockets

    history = deque(maxlen=HISTORY_LEN)
    latest = {'state': None, 'map': None}

    stop = asyncio.Event()

    async def sampler():
        # Also the process's shutdown watcher: rclpy's signal handlers absorb
        # SIGINT/SIGTERM and shut the ROS context down WITHOUT exiting, which
        # left the first smoke-test's server running headless until SIGKILL.
        # rclpy.ok() going false is therefore the one reliable stop signal.
        period = 1.0 / SNAPSHOT_HZ
        while True:
            if not node._rclpy.ok():
                stop.set()
                return
            state, map_payload = node.build_state()
            latest['state'] = state
            latest['map'] = map_payload
            m = state['metrics']
            history.append(
                [state['t'] if column == 't' else m[column]
                 for column in HISTORY_COLUMNS]
            )
            await asyncio.sleep(period)

    async def handler(ws):
        # A client hanging up mid-send is the NORMAL end of a connection
        # (page closed, probe finished) — swallow it, or every disconnect
        # writes a 12-line traceback into the session log.
        sent_map_seq = -1
        try:
            await ws.send(json.dumps({'type': 'hello', 'history': list(history),
                                      'config': {'snapshot_hz': SNAPSHOT_HZ}}))
            while True:
                state = latest['state']
                if state is not None:
                    msg = {'type': 'snapshot', 'state': state}
                    if latest['map'] is not None and latest['map']['seq'] != sent_map_seq:
                        msg['map'] = latest['map']
                        sent_map_seq = latest['map']['seq']
                    await ws.send(json.dumps(msg))
                await asyncio.sleep(1.0 / SNAPSHOT_HZ)
        except websockets.ConnectionClosed:
            return

    async def process_request(path, request_headers):
        if path.split('?')[0] in ('/', '/index.html'):
            with open(PAGE, 'rb') as f:
                body = f.read()
            return (http.HTTPStatus.OK,
                    [('Content-Type', 'text/html; charset=utf-8'),
                     ('Cache-Control', 'no-store')], body)
        if path == '/healthz':
            return (http.HTTPStatus.OK, [('Content-Type', 'text/plain')], b'ok\n')
        return None      # anything else: proceed with the WebSocket handshake

    asyncio.create_task(sampler())
    # A lens is often already open on the default port (an operator tab from
    # the previous session -- normal, and never ours to kill). Walk forward a
    # few ports instead of dying on EADDRINUSE.
    server = None
    port = args.port
    for candidate in range(args.port, args.port + 6):
        try:
            server = await websockets.serve(handler, args.host, candidate,
                                            process_request=process_request)
            port = candidate
            break
        except OSError:
            continue
    if server is None:
        print(f'slam_lens: ports {args.port}-{args.port + 5} all busy; exiting',
              flush=True)
        return
    async with server:
        if port != args.port:
            print(f'slam_lens: port {args.port} busy (another lens?), '
                  f'using {port}', flush=True)
        print(f'slam_lens: http://{args.host}:{port}/   '
              f'(domain {os.environ.get("ROS_DOMAIN_ID", "?")}, '
              f'map {args.map_topic}, scan {args.scan_topic})', flush=True)
        await stop.wait()
        print('slam_lens: ROS context shut down, exiting', flush=True)

    # File the metric history with the session it watched (audit 2026-08-10:
    # live metrics used to die with the process). Fail-open: a dump problem
    # must never turn a clean shutdown into a traceback.
    try:
        dump = args.dump or None
        if dump and history:
            with open(dump, 'w') as f:
                json.dump({'columns': list(HISTORY_COLUMNS),
                           'snapshot_hz': SNAPSHOT_HZ,
                           'history': list(history)}, f)
            print(f'slam_lens: metric history -> {dump}', flush=True)
    except Exception as e:
        print(f'slam_lens: history dump failed ({e})', flush=True)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0], allow_abbrev=False)
    ap.add_argument('--domain', type=int, default=66,
                    help='ROS_DOMAIN_ID (66 = the sim convention; the hardware '
                         'domain is never a default in this repo)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--map-topic', default='/map')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--truth-topic', default='/sim/ground_truth')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--sim-time', action='store_true',
                    help='use /clock (replays only, same rule as replay_slam_bag)')
    ap.add_argument('--manifest', default='',
                    help='scene manifest: arms the landmark detector with the '
                         'AUTHORED radius and marks B, the post and the '
                         'delivery standoff on the canvas')
    ap.add_argument('--dump', default='',
                    help='where to write the metric history on exit (default: '
                         'the domain\'s latest session dir, if one exists)')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = LensNode(args)

    # Arm the detector from the manifest, exactly as the mission does, and mark
    # where the scene says B actually is. Seeing a "confirmed" detection at
    # 0.9 m while B's marker sits 5 m away is the whole point: the phantom is
    # obvious on the canvas and invisible in a metric.
    if args.manifest:
        try:
            with open(args.manifest) as f:
                man = json.load(f)
            actors = man.get('actors', {})
            radius = actors.get('b_radius_m')
            if radius:
                from landmark_detector import LandmarkDetector
                node.detector = LandmarkDetector(radius)
                print(f'corridor_lens: detector armed, radius {radius} m', flush=True)
            # ADR 0031: one marker, because there is one object. The pink
            # confirmed-detection crosshair is supposed to land ON the yellow
            # B ring now -- two circles far apart is the phantom, and it is the
            # single most useful thing this canvas shows.
            node.truth_markers = {'b': actors.get('b_xyz_m', [None])[:2]}
        except Exception as e:
            print(f'corridor_lens: no manifest markers ({e})', flush=True)

    # Deliberately SingleThreaded — see the module docstring for the measured
    # reason. The executor runs in a daemon thread; asyncio owns the main one.
    ex = SingleThreadedExecutor()
    ex.add_node(node.node)

    def _spin():
        # ExternalShutdownException at teardown is the normal Ctrl+C path,
        # not an error; the relay's log grew the same traceback until it was
        # understood. Exit the thread quietly.
        from rclpy.executors import ExternalShutdownException
        try:
            ex.spin()
        except ExternalShutdownException:
            pass

    spin = threading.Thread(target=_spin, daemon=True)
    spin.start()

    try:
        asyncio.run(serve(node, args))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
