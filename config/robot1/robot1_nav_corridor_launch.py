"""Governed Nav2 for robot1 in the corridor. Launch by absolute path.

    ros2 launch config/robot1/robot1_nav_corridor_launch.py

Assumes the robot1 stack is ALREADY up (simctl start --robot robot1 --backend
isaac brings up bringup + safety + laser odometry + SLAM). This file adds only
the Nav2 servers, exactly as robot2_nav_sim_launch.py does for robot2 --
simctl:436-437 records that simctl never starts Nav2 for either robot.

WHY THIS FILE EXISTS AT ALL
---------------------------
There is no robot1 Nav2 launch anywhere in the fleet, governed or otherwise,
and no nav2_robot1.yaml. The two candidates in the vendor tree are both
UNGOVERNED: yahboomcar_nav/launch/navigation_dwb_launch.py has no cmd_vel
remapping whatsoever (nav2_smoke.py:162-164 says so in as many words), and
yahboomcar_multi/launch/navigation_launch.py remaps into a velocity_smoother
whose output lands straight on /cmd_vel. Running either would put Nav2's
commands on the motors with nothing in between.

ADR 0023 makes the governed path law. So this launch mirrors the robot2
precedent (robot2_nav_sim_launch.py:37-38,:43,:53) rather than inventing
anything.

THE ONE THING THAT MAKES IT WORK
--------------------------------
robot1's governor cannot be namespaced or retargeted: it hardcodes its topics
absolutely -- '/scan' in, '/cmd_vel_raw' in, '/cmd_vel' out
(yahboomcar_safety/cmd_vel_governor.py:82,84,86). robot2's governor has
cmd_in_topic / cmd_out_topic parameters; robot1's has none.

At ROOT namespace that is not a problem, it is the mechanism: with no
`namespace=` on these nodes, the remapping ('cmd_vel' -> 'cmd_vel_raw')
resolves to the absolute /cmd_vel_raw, which is precisely the governor's input.
robot1 running unprefixed (architecture.md:46-51) is what makes governed Nav2
reachable without touching yahboomcar_safety.

**Consequence, stated so nobody trips over it later:** any future attempt to
namespace robot1 breaks this. A namespaced Nav2 would publish
/robot1/cmd_vel_raw, the governor would keep listening to /cmd_vel_raw, and
Nav2 would drive the motors ungoverned while every topic name still looked
right. Namespacing robot1 requires porting rasptank's topic parameters into
yahboomcar_safety FIRST.
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

#: Absolute, because this file is launched by path rather than from a package
#: share directory.
#: CORRIDOR_NAV_PARAMS selects the controller arm for the U3 comparison. It is
#: an env override rather than a launch argument so the runner can set it
#: without every caller learning a new flag, and it defaults to the DWB file, so
#: an unset environment behaves exactly as before.
PARAMS = os.environ.get(
    "CORRIDOR_NAV_PARAMS",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav2_robot1_corridor.yaml"),
)

#: Both writers of motion are remapped into the governed pipe. Path following
#: AND recovery behaviours (spin, backup) -- a recovery that bypassed the
#: governor would be the most dangerous motion the robot makes, since it fires
#: exactly when Nav2 is already confused.
GOVERNED = [("cmd_vel", "cmd_vel_raw")]

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description() -> LaunchDescription:
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[PARAMS],
        remappings=GOVERNED,
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[PARAMS],
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[PARAMS],
        remappings=GOVERNED,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[PARAMS],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": LIFECYCLE_NODES,
                # D-19 / OI-14: a bond_timeout of 30.0 declared "no heartbeat
                # for 30000 ms" within ~200 ms of connecting and deactivated
                # the managed node, on BOTH robots' managers.
                "bond_timeout": 0.0,
            }
        ],
    )
    # THE MANAGER MUST NOT RACE THE NODES IT MANAGES.
    #
    # All five used to start in one LaunchDescription, at the same instant, with
    # `autostart: True`. The manager then immediately calls `get_state` on
    # services that may not be discoverable yet, and when that call fails it does
    # not retry -- it aborts the ENTIRE bring-up:
    #
    #     Failed to change state for node: planner_server. Exception:
    #       planner_server/get_state service client: async_send_request failed.
    #     Failed to bring up all requested nodes. Aborting bringup.
    #
    # Measured: 7 of 27 runs on 2026-08-13 died this way, 26%, and it recurred
    # on 2026-08-14. The evidence for it being a discovery race rather than a
    # slow node is that the victim MOVES -- controller_server on some runs,
    # planner_server on others -- and that the reported `last state` is
    # sometimes `unknown`, i.e. the query itself never landed. A node that was
    # merely slow would fail in the same place every time.
    #
    # Five seconds against a bring-up that already takes ~110 s, to remove a
    # failure that costs a whole ~250 s run one time in four. It cannot slow a
    # healthy run by more than those five seconds, and the runner's own
    # lifecycle deadline absorbs it with 100 s to spare.
    #
    # This is a LAUNCH-COMPOSITION fix, not a Nav2 parameter change: nothing
    # about how Nav2 plans or drives is touched.
    LIFECYCLE_MANAGER_DELAY_S = 5.0
    return LaunchDescription([
        controller, planner, behaviors, bt_navigator,
        TimerAction(period=LIFECYCLE_MANAGER_DELAY_S, actions=[lifecycle]),
    ])
