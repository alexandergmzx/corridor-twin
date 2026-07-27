"""Launch simulator-free camera playback, the police observer, and the display.

This is the demonstration's recorded fallback: it needs no GPU and no Isaac
install, and it shows the same picture the live run does, so a failed live run
on the day costs the viewport rather than the whole demonstration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    manifest = LaunchConfiguration("manifest")
    speed = LaunchConfiguration("speed_mps")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("police_observer"), "rviz", "corridor_twin.rviz"]
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
            ),
            Node(
                package="police_observer",
                executable="police-observer",
                name="police_observer",
                parameters=[{"manifest_path": manifest, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="police_observer",
                executable="enforcement-view",
                name="enforcement_view",
                parameters=[{"manifest_path": manifest, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
