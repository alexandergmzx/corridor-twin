"""Launch simulator-free camera playback and the police observer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    manifest = LaunchConfiguration("manifest")
    speed = LaunchConfiguration("speed_mps")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription(
        [
            # Ignore ~/.local NumPy wheels: ROS Jazzy's cv_bridge/OpenCV binaries
            # are built against the Ubuntu NumPy ABI supplied by apt.
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            DeclareLaunchArgument("manifest"),
            DeclareLaunchArgument("speed_mps", default_value="1.8"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="police_observer",
                executable="synthetic-camera-publisher",
                name="synthetic_camera_publisher",
                parameters=[
                    {
                        "manifest_path": manifest,
                        "speed_mps": speed,
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
        ]
    )
