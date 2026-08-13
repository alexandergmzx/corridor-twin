#!/usr/bin/env python3
"""How much of A's forward motion does SLAM actually register?

The along-corridor pose error is negative and appears early. If the matcher is
failing to measure travel along the corridor, then over the first metres SLAM's
own pose should advance LESS than truth does -- and the EKF, which is accurate
here, should advance correctly. That is the difference between "the map is
noisy" and "the matcher cannot see motion along this axis".
"""

import json
import math
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

with open("out/corridor.manifest.json") as handle:
    MANIFEST = json.load(handle)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def at(track, when):
    return min(track, key=lambda row: abs(row[0]-when))


def main():
    print(f"{'bag':<22}{'truth':>8}{'EKF':>8}{'SLAM':>8}"
          f"{'EKF/truth':>11}{'SLAM/truth':>12}")
    for spec in sys.argv[1:]:
        bag, profile = spec.split("=")
        r = SequentialReader()
        r.open(StorageOptions(uri=bag, storage_id="mcap"),
               ConverterOptions("cdr", "cdr"))
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
        if len(corr) < 10:
            continue

        def slam_xy(when, odom=odom, corr=corr):
            _, ox, oy, oyaw = at(odom, when)
            _, cx, cy, cyaw = at(corr, when)
            cc, ss = math.cos(cyaw), math.sin(cyaw)
            return (cx + ox*cc - oy*ss, cy + ox*ss + oy*cc)

        start = truth[0]
        # The window: from the start of motion until truth has covered 2 m.
        target = next((row for row in truth
                       if math.dist((row[1], row[2]), (start[1], start[2])) >= 2.0), None)
        if target is None:
            continue
        moved = next(row for row in truth
                     if math.dist((row[1], row[2]), (start[1], start[2])) >= 0.05)
        t_travel = math.dist((target[1], target[2]), (moved[1], moved[2]))
        o0, o1 = at(odom, moved[0]), at(odom, target[0])
        e_travel = math.dist((o1[1], o1[2]), (o0[1], o0[2]))
        s0, s1 = slam_xy(moved[0]), slam_xy(target[0])
        s_travel = math.dist(s1, s0)
        print(f"{bag.split('/')[-1][:21]:<22}{t_travel:>8.3f}{e_travel:>8.3f}"
              f"{s_travel:>8.3f}{e_travel/t_travel:>11.3f}{s_travel/t_travel:>12.3f}")


if __name__ == "__main__":
    main()
