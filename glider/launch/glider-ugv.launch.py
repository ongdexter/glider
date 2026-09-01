"""Launch Glider's GPS/compass/IMU/RKO-LIO UGV fusion profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="ugv"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "gps_topic", default_value="mavros/global_position/global"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("glider"), "launch", "glider-robot.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_ns": LaunchConfiguration("robot_ns"),
                    "profile": "ugv",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "assume_north_aligned": "false",
                    "gps_topic": LaunchConfiguration("gps_topic"),
                }.items(),
            ),
        ]
    )
