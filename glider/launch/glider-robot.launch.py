"""Launch a namespaced Glider instance with a UAV or UGV sensor profile."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    os.makedirs("/tmp/glider", exist_ok=True)
    robot_ns = LaunchConfiguration("robot_ns")
    profile = LaunchConfiguration("profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gps_topic = LaunchConfiguration("gps_topic")

    ros_params = PathJoinSubstitution(
        [FindPackageShare("glider"), "config", ["ros-params-", profile, ".yaml"]]
    )
    graph_filename = PythonExpression(
        ["'glider-params-uav.yaml' if '", profile, "' == 'uav' else 'glider-params.yaml'"]
    )
    graph_params = PathJoinSubstitution(
        [FindPackageShare("glider"), "config", graph_filename]
    )
    assume_north_aligned = LaunchConfiguration("assume_north_aligned")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", description="Robot namespace"),
            DeclareLaunchArgument(
                "profile",
                description="Sensor profile: uav or ugv",
                choices=["uav", "ugv"],
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "assume_north_aligned",
                default_value="false",
                description="Use GPS-only north-aligned output (legacy bags only)",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="mavros/global_position/global",
                description="NavSatFix input; use an absolute name for legacy bags",
            ),
            Node(
                package="glider",
                executable="glider_node",
                namespace=robot_ns,
                name="glider_node",
                output="screen",
                parameters=[
                    ros_params,
                    {
                        "path": graph_params,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "subscribers.gps_topic": gps_topic,
                        "subscribers.assume_north_aligned": ParameterValue(
                            assume_north_aligned, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
