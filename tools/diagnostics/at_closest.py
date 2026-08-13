#!/usr/bin/env python3
"""At the moment A was physically at the standoff, what did Nav2 believe?"""
import math
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from tf2_msgs.msg import TFMessage

BAG = sys.argv[1]
STANDOFF_WORLD = (4.438, -2.4)
B_WORLD = (5.038, -2.4)
GOAL_MAP, GOAL_YAW = (4.1061, -2.9319), 0.0
SPAWN_YAW = math.atan2(0.12403, 0.99228)   # nominal approach heading
XY_TOL, YAW_TOL = 0.15, 0.6

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
odom, truth, corr = [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    s = t * 1e-9
    if topic == "/odom":
        m = deserialize_message(data, Odometry).pose.pose
        odom.append((s, m.position.x, m.position.y, yaw_of(m.orientation)))
    elif topic == "/sim/ground_truth":
        m = deserialize_message(data, Odometry).pose.pose
        truth.append((s, m.position.x, m.position.y, yaw_of(m.orientation)))
    elif topic == "/tf":
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                v = tr.transform.translation
                corr.append((s, v.x, v.y, yaw_of(tr.transform.rotation)))

def at(tr, w):
    return min(tr, key=lambda row: abs(row[0]-w))
def map_pose(w):
    _, ox, oy, oyaw = at(odom, w)
    _, cx, cy, cyaw = at(corr, w)
    c, s = math.cos(cyaw), math.sin(cyaw)
    return (cx + ox*c - oy*s, cy + ox*s + oy*c, cyaw + oyaw)

# world -> map is a pure rotation by -spawn_yaw (spawn is the origin of both)
def world_to_map(x, y):
    c, s = math.cos(-SPAWN_YAW), math.sin(-SPAWN_YAW)
    return (x*c - y*s, x*s + y*c)

closest = min(truth, key=lambda row: math.dist((row[1], row[2]), STANDOFF_WORLD))
t0 = truth[0][0]
w = closest[0]
mx, my, myaw = map_pose(w)
ex, ey = world_to_map(closest[1], closest[2])
_, cx, cy, cyaw = at(corr, w)

print("=== WHEN A WAS PHYSICALLY AT THE STANDOFF ===")
print(f"  t (bag)                 {w-t0:.2f} s")
print(f"  truth world             ({closest[1]:.3f}, {closest[2]:.3f})"
      f"  yaw {math.degrees(closest[3]):7.1f} deg")
print(f"  distance to standoff    {math.dist((closest[1],closest[2]), STANDOFF_WORLD):.4f} m")
print(f"  distance to B           {math.dist((closest[1],closest[2]), B_WORLD):.4f} m")
print()
print(f"  where SLAM thought it was   ({mx:.3f}, {my:.3f})"
      f"  yaw {math.degrees(myaw):7.1f} deg")
print(f"  where it SHOULD have been   ({ex:.3f}, {ey:.3f})   (truth rotated into map)")
print(f"  SLAM POSE ERROR             {math.dist((mx,my),(ex,ey)):.3f} m")
print(f"  map->odom correction        ({cx:.3f}, {cy:.3f})"
      f" yaw {math.degrees(cyaw):.1f} deg")
print()
xy = math.dist((mx,my), GOAL_MAP)
yerr = abs((myaw - GOAL_YAW + math.pi) % (2*math.pi) - math.pi)
print(f"  goal checker xy   {xy:.3f} m  vs {XY_TOL}     -> {'PASS' if xy<=XY_TOL else 'FAIL'}")
print(f"  goal checker yaw  {math.degrees(yerr):.1f} deg"
      f" vs {math.degrees(YAW_TOL):.1f} -> {'PASS' if yerr<=YAW_TOL else 'FAIL'}")
print()
print("  Counterfactual -- if SLAM had been PERFECT, would it have completed?")
xy2 = math.dist((ex,ey), GOAL_MAP)
yaw_true_map = closest[3] - SPAWN_YAW
y2 = abs((yaw_true_map - GOAL_YAW + math.pi) % (2*math.pi) - math.pi)
print(f"    xy  {xy2:.3f} m -> {'PASS' if xy2<=XY_TOL else 'FAIL'}")
print(f"    yaw {math.degrees(y2):.1f} deg -> {'PASS' if y2<=YAW_TOL else 'FAIL'}"
      "   <-- the goal asks A to face yaw 0")
