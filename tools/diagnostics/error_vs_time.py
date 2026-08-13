#!/usr/bin/env python3
"""Does the SLAM pose error creep, or does it jump?"""
import math
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

BAG = sys.argv[1]
SPAWN_YAW = math.atan2(0.12403, 0.99228)
def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("cdr","cdr"))
odom, truth, corr = [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    s = t*1e-9
    if topic == "/odom":
        m = deserialize_message(data, Odometry).pose.pose
        odom.append((s, m.position.x, m.position.y, yaw_of(m.orientation)))
    elif topic == "/sim/ground_truth":
        m = deserialize_message(data, Odometry).pose.pose
        truth.append((s, m.position.x, m.position.y))
    elif topic == "/tf":
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.header.frame_id=="map" and tr.child_frame_id=="odom":
                v = tr.transform.translation
                corr.append((s, v.x, v.y, yaw_of(tr.transform.rotation)))

def at(tk, w):
    return min(tk, key=lambda r_: abs(r_[0]-w))
c_, s_ = math.cos(-SPAWN_YAW), math.sin(-SPAWN_YAW)
t0 = truth[0][0]
print(f"{'t':>6} {'truth x,y':>16} {'slam err':>9} {'|corr|':>8} {'corr step':>10}")
prev_c = None
prev_e = None
for row in truth[::22]:
    w = row[0]
    _, ox, oy, oyaw = at(odom, w)
    _, cx, cy, cyaw = at(corr, w)
    cc, ss = math.cos(cyaw), math.sin(cyaw)
    mx, my = cx + ox*cc - oy*ss, cy + ox*ss + oy*cc
    ex, ey = row[1]*c_ - row[2]*s_, row[1]*s_ + row[2]*c_
    err = math.dist((mx,my),(ex,ey))
    cmag = math.hypot(cx,cy)
    step = "" if prev_c is None else f"{cmag-prev_c:+9.3f}"
    flag = ""
    if prev_e is not None and err-prev_e > 0.25:
        flag = "  <== JUMP"
    print(f"{w-t0:6.1f} ({row[1]:6.2f},{row[2]:6.2f}) {err:9.3f} {cmag:8.3f} {step}{flag}")
    prev_c, prev_e = cmag, err
