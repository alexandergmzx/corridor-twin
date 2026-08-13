#!/usr/bin/env python3
"""Is the SLAM pose error ALONG the corridor axis, or across it?"""
import math
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

SPAWN_YAW = math.atan2(0.12403, 0.99228)
def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
r = SequentialReader()
r.open(StorageOptions(uri=sys.argv[1], storage_id="mcap"),
                               ConverterOptions("cdr","cdr"))
odom, truth, corr = [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    s = t*1e-9
    if topic=="/odom":
        m = deserialize_message(data,Odometry).pose.pose
        odom.append((s,m.position.x,m.position.y,yaw_of(m.orientation)))
    elif topic=="/sim/ground_truth":
        m = deserialize_message(data,Odometry).pose.pose
        truth.append((s,m.position.x,m.position.y))
    elif topic=="/tf":
        for tr in deserialize_message(data,TFMessage).transforms:
            if tr.header.frame_id=="map" and tr.child_frame_id=="odom":
                v = tr.transform.translation
                corr.append((s,v.x,v.y,yaw_of(tr.transform.rotation)))
def at(tk, w):
    return min(tk,key=lambda r_:abs(r_[0]-w))
c_ = math.cos(-SPAWN_YAW)
s_ = math.sin(-SPAWN_YAW)
t0 = truth[0][0]
print(f"{'t':>6} {'A along corridor':>17} {'err ALONG':>10} {'err ACROSS':>11}")
for row in truth[::44]:
    w = row[0]
    _, ox, oy, oyaw = at(odom, w)
    _, cx, cy, cyaw = at(corr, w)
    cc, ss = math.cos(cyaw), math.sin(cyaw)
    mx, my = cx+ox*cc-oy*ss, cy+ox*ss+oy*cc
    ex, ey = row[1]*c_-row[2]*s_, row[1]*s_+row[2]*c_
    # map +x IS the corridor axis (map is anchored on A's spawn heading)
    along, across = mx-ex, my-ey
    if 52 < w-t0 < 112:
        print(f"{w-t0:6.1f} {ex:17.2f} {along:10.3f} {across:11.3f}")
