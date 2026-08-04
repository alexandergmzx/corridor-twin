"""Launch the one sanctioned crossing between A's domain and P's domain.

This starts the upstream ``domain_bridge`` node against
``config/corridor_domain_bridge.yaml``. The allowlist lives in that file, not
here, so the sanctioned surface is reviewable without reading launch code.

This launch file deliberately does **not** set ``ROS_DOMAIN_ID``. The bridge is
the one process that is legitimately in both domains at once: it creates its own
participants per domain from the configuration, so confining it to one ambient
domain would either break it or hide which side it was on. Every other node in
this demonstration is pinned to exactly one domain by its own launch file.

``--wait-for-publisher`` is left at its upstream default of true, which is what
the demonstration ordering needs: ``tools/run_demo.sh`` starts the ROS side
before Isaac, so the bridge comes up while A is still loading and waits rather
than failing. The cost is a failure mode worth knowing on the day -- if the
Isaac side dies, P receives nothing, and that looks exactly like isolation
working. To tell the two apart, list the robot domain: no camera topic there
means the publisher is gone, not that the boundary is holding.

    ROS_DOMAIN_ID=42 ros2 topic list
"""

from corridor_gateway.domains import POLICE_DOMAIN_ID, ROBOT_DOMAIN_ID
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("corridor_gateway"), "config", "corridor_domain_bridge.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_domain_id", default_value=str(ROBOT_DOMAIN_ID)),
            DeclareLaunchArgument("police_domain_id", default_value=str(POLICE_DOMAIN_ID)),
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name="corridor_twin_gateway",
                # The YAML path is positional; --from/--to override the domains
                # declared inside it so a rehearsal on a shared network can move
                # both halves without editing the reviewed allowlist.
                arguments=[
                    config,
                    "--from",
                    LaunchConfiguration("robot_domain_id"),
                    "--to",
                    LaunchConfiguration("police_domain_id"),
                ],
                output="screen",
            ),
        ]
    )
