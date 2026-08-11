"""Launch simulator-free camera playback, the police observer, and the display.

This is the demonstration's recorded fallback: it needs no GPU and no Isaac
install, and it shows the same picture the live run does, so a failed live run
on the day costs the viewport rather than the whole demonstration.

It is also the only end-to-end path that exercises the domain split on an
ordinary machine, so it runs the same two-domain topology the live path does:
the synthetic publisher stands in for A on the robot domain, the observer and
display sit on the police domain, and ``corridor_gateway`` is the one crossing
between them. A fallback that quietly ran everything in one domain would
demonstrate a system this repository no longer ships. See ADR 0020.

The domain is set per node with ``additional_env`` rather than once for the whole
launch description, because a launch-wide ``SetEnvironmentVariable`` applies to
whatever is visited after it -- which would make each node's domain a property of
where its line happens to sit in the list.

Wall time is the default here, so there is no ``/clock`` to carry; with
``use_sim_time:=true`` the publisher emits one and the same gateway relays it.
"""

from corridor_gateway.domains import POLICE_DOMAIN_ID, ROBOT_DOMAIN_ID
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    manifest = LaunchConfiguration("manifest")
    speed = LaunchConfiguration("speed_mps")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    robot_domain = LaunchConfiguration("robot_domain_id")
    police_domain = LaunchConfiguration("police_domain_id")
    robot_side = {"ROS_DOMAIN_ID": robot_domain}
    police_side = {"ROS_DOMAIN_ID": police_domain}
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("police_observer"), "rviz", "corridor_twin.rviz"]
    )
    gateway = PathJoinSubstitution(
        [FindPackageShare("corridor_gateway"), "launch", "gateway.launch.py"]
    )
    return LaunchDescription(
        [
            # Ignore ~/.local NumPy wheels: ROS Jazzy's cv_bridge/OpenCV binaries
            # are built against the Ubuntu NumPy ABI supplied by apt.
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            DeclareLaunchArgument("manifest"),
            DeclareLaunchArgument("speed_mps", default_value="1.8"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Run to the end of the covered window rather than stopping at 7.2 m.
            # This is the demonstration's recorded fallback, so it has to be able
            # to reach the corner: gates 8.0 and 10.0 are where the strict rule
            # applies, and a run that stops short can never show the violation
            # the live demonstration shows.
            DeclareLaunchArgument("end_station_m", default_value="10.8"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("robot_domain_id", default_value=str(ROBOT_DOMAIN_ID)),
            DeclareLaunchArgument("police_domain_id", default_value=str(POLICE_DOMAIN_ID)),
            # The one sanctioned crossing. It is not pinned to a domain: it is
            # the only participant that is legitimately in both.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gateway]),
                launch_arguments={
                    "robot_domain_id": robot_domain,
                    "police_domain_id": police_domain,
                }.items(),
            ),
            # --- A's side -----------------------------------------------------
            Node(
                package="police_observer",
                executable="synthetic-camera-publisher",
                name="synthetic_camera_publisher",
                parameters=[
                    {
                        "manifest_path": manifest,
                        "speed_mps": speed,
                        "end_station_m": ParameterValue(
                            LaunchConfiguration("end_station_m"), value_type=float
                        ),
                        "use_sim_time": use_sim_time,
                        "publish_clock": use_sim_time,
                    }
                ],
                additional_env=robot_side,
            ),
            # --- P's side -----------------------------------------------------
            # Truth stays behind on the robot domain: the synthetic publisher
            # emits test/ground_truth/speed there, and nothing on this side can
            # discover it. That isolation used to be policy enforced by a source
            # audit; here it is a property of the transport.
            Node(
                package="police_observer",
                executable="police-observer",
                name="police_observer",
                parameters=[{"manifest_path": manifest, "use_sim_time": use_sim_time}],
                additional_env=police_side,
            ),
            Node(
                package="police_observer",
                executable="enforcement-view",
                name="enforcement_view",
                parameters=[{"manifest_path": manifest, "use_sim_time": use_sim_time}],
                additional_env=police_side,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                additional_env=police_side,
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
