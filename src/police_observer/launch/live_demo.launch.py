"""Launch the ROS side of the live demonstration.

This is the half that runs on system Jazzy: the camera-only observer, the
enforcement display, and RViz. The Isaac adapter is deliberately *not* started
here. It runs under Isaac's bundled Jazzy on a different Python ABI and
re-execs itself into an isolated environment, so putting it in this launch
description would drag system ROS paths into a process that rejects them.
``tools/run_demo.sh`` starts both halves in their own environments.

Sim time is the default. The adapter publishes ``/clock`` and stamps camera
messages from the simulation clock, and the observer differentiates those
stamps; running this side on wall time would mix two clocks in one speed
measurement.
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
    profile = LaunchConfiguration("corridor_profile")
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
            DeclareLaunchArgument("corridor_profile", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="police_observer",
                executable="police-observer",
                name="police_observer",
                parameters=[
                    {
                        "manifest_path": manifest,
                        "corridor_profile": profile,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="police_observer",
                executable="enforcement-view",
                name="enforcement_view",
                parameters=[
                    {
                        "manifest_path": manifest,
                        "corridor_profile": profile,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
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
