"""Launch Glider's GPS-only, north-aligned UAV profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="uav"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("assume_north_aligned", default_value="false"),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="mavros/global_position/global",
                description="NavSatFix input; absolute names support legacy bags",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("glider"), "launch", "glider-robot.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_ns": LaunchConfiguration("robot_ns"),
                    "profile": "uav",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "assume_north_aligned": LaunchConfiguration(
                        "assume_north_aligned"
                    ),
                    "gps_topic": LaunchConfiguration("gps_topic"),
                }.items(),
            ),
        ]
    )
