#!/usr/bin/env python3
"""Is the along-corridor error universal, or was one bag unrepresentative?

Runs the along/across decomposition over many bags and reports, per bag, where
the error reaches half its final value and where it plateaus. Profile-aware:
each profile has its own spawn heading, so the corridor axis differs.
"""

import json
import math
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

with open("out/corridor.manifest.json") as _f:
    MANIFEST = json.load(_f)


def spawn_yaw(profile):
    h = MANIFEST["profiles"][profile]["delivery_trajectory"]["approach_heading"]
    return math.atan2(h[1], h[0])


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    odom, truth, corr = [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        s = t * 1e-9
        if topic == "/odom":
            m = deserialize_message(data, Odometry).pose.pose
            odom.append((s, m.position.x, m.position.y, yaw_of(m.orientation)))
        elif topic == "/sim/ground_truth":
            m = deserialize_message(data, Odometry).pose.pose
            truth.append((s, m.position.x, m.position.y))
        elif topic == "/tf":
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    v = tr.transform.translation
                    corr.append((s, v.x, v.y, yaw_of(tr.transform.rotation)))
    return odom, truth, corr


def at(track, when):
    return min(track, key=lambda row: abs(row[0]-when))


def main():
    print(f"{'bag':<22}{'profile':<22}{'travel@half':>12}{'peak along':>12}"
          f"{'peak across':>13}{'|along|/|across|':>18}")
    for spec in sys.argv[1:]:
        bag, profile = spec.split("=")
        odom, truth, corr = read(bag)
        if len(odom) < 10 or len(corr) < 10:
            print(f"{bag.split('/')[-1][:21]:<22}{profile:<22}  (insufficient data)")
            continue
        yaw0 = spawn_yaw(profile)
        c_, s_ = math.cos(-yaw0), math.sin(-yaw0)
        start = truth[0]
        series = []
        for row in truth[::11]:
            w = row[0]
            _, ox, oy, oyaw = at(odom, w)
            _, cx, cy, cyaw = at(corr, w)
            cc, ss = math.cos(cyaw), math.sin(cyaw)
            mx, my = cx + ox*cc - oy*ss, cy + ox*ss + oy*cc
            ex, ey = row[1]*c_ - row[2]*s_, row[1]*s_ + row[2]*c_
            travelled = math.dist((row[1], row[2]), (start[1], start[2]))
            series.append((travelled, mx-ex, my-ey))
        # RESTRICT TO THE STRAIGHT APPROACH. Past the corner A leaves the
        # corridor on most runs, and the across-axis error then explodes --
        # including it measures the excursion, not the corridor.
        approach = MANIFEST["profiles"][profile]["delivery_trajectory"]["approach_length_m"]
        inside = [r for r in series if r[0] <= approach] or series
        peak_along = max(inside, key=lambda r: abs(r[1]))
        peak_across = max(inside, key=lambda r: abs(r[2]))
        half = abs(peak_along[1]) / 2.0
        at_half = next((r[0] for r in inside if abs(r[1]) >= half), float("nan"))
        ratio = abs(peak_along[1]) / max(abs(peak_across[2]), 1e-6)
        print(f"{bag.split('/')[-1][:21]:<22}{profile:<22}{at_half:>10.2f} m"
              f"{peak_along[1]:>12.3f}{peak_across[2]:>13.3f}{ratio:>18.1f}")


if __name__ == "__main__":
    main()
