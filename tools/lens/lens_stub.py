#!/usr/bin/env python3
"""Serve the real lens page against synthetic data. No ROS, no Isaac, no GPU.

    python3 tools/lens/lens_stub.py --port 8766
    # then, in another shell:
    python3 tools/lens/lens_probe.py --url http://127.0.0.1:8766/

WHY THIS EXISTS
---------------
The corridor lens's map rendering was broken from the day it was written
(`4e0f903`) and nobody saw it for four days, because the only way to exercise
the page was a seven-minute Isaac run. An instrument that costs a robot to test
does not get tested.

This serves `corridor_lens.html` -- **the real page, not a copy** -- and speaks
the lens's own wire protocol with a scripted scene: an occupancy grid that
GROWS frame to frame, a scan that lands on its walls, three pose ghosts, the
manifest markers, and a landmark verdict. The whole loop is then seconds, and
it runs on a laptop with nothing else up.

WHAT IT DOES NOT DO
-------------------
It does not test the ROS half. `corridor_lens.py`'s subscriptions, TF lookups
and `build_state()` are out of scope here by construction -- this proves the
browser, which is where the bug was. A regression on the ROS side needs a live
run or a bag, and this file does not pretend otherwise.

The payload shape is asserted against the real producer by
`test/test_lens_stub_matches_the_lens.py`, so the stub cannot quietly drift
into describing a page nobody serves.
"""

from __future__ import annotations

import argparse
import asyncio
import http
import json
import math
import os
from pathlib import Path

ROOT = Path(os.path.abspath(__file__)).parent
PAGE = ROOT / "corridor_lens.html"

#: Matches `corridor_lens.SNAPSHOT_HZ`. Asserted by the contract test.
SNAPSHOT_HZ = 5.0

#: Matches `corridor_lens.HISTORY_COLUMNS`.
HISTORY_COLUMNS = ("t", "fit", "div_pos", "yaw_ratio", "stale_run")

#: A corridor-shaped grid at the committed 0.02 m resolution.
GRID_W, GRID_H, GRID_RES = 300, 260, 0.02
# Chosen so the corridor's north wall lands at row 200 of 260 -- inside the
# grid, not clipped off the top of it.
GRID_OX, GRID_OY = -1.0, -2.0

#: Occupancy values, as slam_toolbox emits them.
UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def rle_encode(cells: list[int]) -> list[int]:
    """Flat [value, count, ...]. Same encoding as `_lens_core.rle_encode`."""

    out: list[int] = []
    for value in cells:
        if out and out[-2] == value:
            out[-1] += 1
        else:
            out.extend((value, 1))
    return out


def grid_at(progress: float) -> list[int]:
    """A corridor that is revealed left to right as `progress` goes 0 -> 1.

    Walls on both sides, free space between, unknown ahead of the frontier --
    the shape a real SLAM map has while it is still being built, so a page that
    renders this correctly renders a real one.
    """

    cells = [UNKNOWN] * (GRID_W * GRID_H)
    frontier = max(1, int(GRID_W * min(max(progress, 0.0), 1.0)))
    north = int((2.0 - GRID_OY) / GRID_RES)
    for column in range(frontier):
        # The corridor tapers, which is the whole point of the scenario.
        width_m = 1.8 - 0.9 * (column / GRID_W)
        south = north - int(width_m / GRID_RES)
        for row in range(max(south, 0), min(north + 1, GRID_H)):
            index = row * GRID_W + column
            cells[index] = OCCUPIED if row in (south, north) else FREE
    return cells


def scene(tick: int) -> tuple[dict, dict]:
    """One snapshot: the state the page draws, and the map when it changed."""

    seconds = tick / SNAPSHOT_HZ
    progress = min(seconds / 30.0, 1.0)

    # A drives east down the corridor while the map grows around it.
    x = -0.5 + 5.0 * progress
    y = 1.0 - 0.4 * progress
    yaw = -0.12

    points, hits = [], []
    for index in range(120):
        bearing = yaw + (index / 120.0 - 0.5) * math.radians(220.0)
        reach = 1.4 + 0.6 * math.sin(index * 0.7)
        points.append([round(x + reach * math.cos(bearing), 3),
                       round(y + reach * math.sin(bearing), 3)])
        hits.append(1 if index % 7 else 0)

    state = {
        "t": round(seconds, 3),
        # A live scene is never frozen; the key exists because the real lens
        # emits it (ADR 0035) and the contract test is bidirectional.
        "frozen": False,
        "rates": {"scan": 10.4, "map": 1.0, "truth": 11.0, "odom": 10.0,
                  "odom_raw": 11.0},
        "pose": [round(x, 3), round(y, 3), yaw],
        "truth_ghost": [round(x + 0.05, 3), round(y - 0.03, 3), yaw + 0.01],
        "odom_ghost": [round(x - 0.09, 3), round(y + 0.06, 3), yaw - 0.02],
        "scan": {"age": 0.04, "resolved": True, "points": points, "hits": hits},
        "metrics": {
            "fit": 0.98, "div_pos": 0.06, "div_yaw": 0.01,
            "yaw_ratio": 1.05, "yaw_n": 40, "stale_run": 0, "stale_max": 0,
            "stale_frac": 0.0, "tf_ok_frac": 1.0, "tf_fail_streak": 0,
        },
        "landmark": {
            "armed": True, "confirmed": progress > 0.6, "candidates": 1,
            "frames_agreeing": 4, "range_m": round(3.0 * (1.0 - progress) + 0.5, 3),
            "bearing_deg": -3.2, "fitted_radius_m": 0.118, "residual_m": 0.008,
            "points": 6,
            "map_xy": [5.038, -2.4] if progress > 0.6 else None,
        },
        # The exact key that carried the ReferenceError. Non-null, as the
        # runner's --manifest makes it on every real run.
        "truth_markers": {"b": [5.038, -2.4]},
        "map_seq": tick // 5 + 1,
    }
    map_payload = {
        "seq": state["map_seq"],
        "w": GRID_W, "h": GRID_H, "res": GRID_RES,
        "ox": GRID_OX, "oy": GRID_OY,
        "rle": rle_encode(grid_at(progress)),
    }
    return state, map_payload


async def main_async(args) -> int:
    import websockets

    latest: dict = {"state": None, "map": None}
    history: list[list] = []

    async def ticker():
        tick = 0
        while True:
            state, map_payload = scene(tick)
            latest["state"] = state
            latest["map"] = map_payload
            metrics = state["metrics"]
            history.append([state["t"] if column == "t" else metrics[column]
                            for column in HISTORY_COLUMNS])
            tick += 1
            await asyncio.sleep(1.0 / SNAPSHOT_HZ)

    async def handler(ws):
        # Same dirty-bit as the real handler: the map ships only when its seq
        # changes, and `sent_map_seq` is per connection so a reload gets one.
        sent_map_seq = -1
        try:
            await ws.send(json.dumps({"type": "hello", "history": history[-1500:],
                                      "config": {"snapshot_hz": SNAPSHOT_HZ}}))
            while True:
                state = latest["state"]
                if state is not None:
                    message = {"type": "snapshot", "state": state}
                    if latest["map"] and latest["map"]["seq"] != sent_map_seq:
                        message["map"] = latest["map"]
                        sent_map_seq = latest["map"]["seq"]
                    await ws.send(json.dumps(message))
                await asyncio.sleep(1.0 / SNAPSHOT_HZ)
        except websockets.ConnectionClosed:
            return

    async def process_request(path, request_headers):
        if path.split("?")[0] in ("/", "/index.html"):
            body = PAGE.read_bytes()
            return (http.HTTPStatus.OK,
                    [("Content-Type", "text/html; charset=utf-8"),
                     ("Cache-Control", "no-store")], body)
        if path == "/healthz":
            return (http.HTTPStatus.OK, [("Content-Type", "text/plain")], b"ok\n")
        return None

    asyncio.get_event_loop().create_task(ticker())
    async with websockets.serve(handler, args.host, args.port,
                                process_request=process_request):
        print(f"lens_stub: http://{args.host}:{args.port}/  "
              f"(synthetic; grid {GRID_W}x{GRID_H} @ {GRID_RES} m)", flush=True)
        await asyncio.Future()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
