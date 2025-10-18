"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os
import yaml
import launch

import ament_index_python.packages

from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    param_config = os.path.join(config_directory, 'zed_f9p.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['ublox_gps_node']['ros__parameters']

    # Find package share directory
    glider_share = FindPackageShare('glider')
    glider_share_dir = get_package_share_directory('glider')

    # Path to parameter files
    ros_params_file = PJoin([
        glider_share,
        'config',
        'ros-params.yaml'
    ])
    
    graph_params_file = PJoin([
        glider_share,
        'config',
        'glider-params.yaml'
    ])

    container = ComposableNodeContainer(
        name="high_altitude_ec_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # prefix=['xterm -e gdb -ex run --args'],
            composable_node_descriptions=[
                # Rosbag2 recorder
                ComposableNode(
                    package='rosbag2_composable_recorder',
                    plugin='rosbag2_composable_recorder::ComposableRecorder',
                    name="recorder",
                    parameters=[{'topics': [
                        "/ublox_gps_node/fix",
                        "/glider/odom",
                        "/vectornav/imu",
                        ],
                                 'storage_id': 'mcap',
                                 'record_all': False,
                                 'disable_discovery': False,
                                 'serialization_format': 'cdr',
                                 'start_recording_immediately': False,
                                 "bag_name": LaunchConfig("bag"),
                                 "bag_prefix": LaunchConfig("bag_prefix")}],
                    remappings=[],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),

            # Vectornav Raw
            ComposableNode(
                package='vectornav',
                plugin='vectornav::Vectornav',
                name='vectornav',
                parameters=[PJoin(
                    [FindPackageShare('vectornav'),
                     'config', 'vectornav_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}]),

            # Ublox GPS
            ComposableNode(
                package='ublox_gps',
                plugin='ublox_node::UbloxNode',
                name='ublox_gps_node',
                parameters=[params],
                remappings=[("/aidalm",  "/ublox_raw/aidalm"),
                            ("/timtm2", "/ublox_raw/timtm2"),
                            ("/rtcm", "/ublox_raw/rtcm"),
                            ("/nmea", "/ublox_raw/nmea"),
                            ("/navclock", "/ublox_raw/navclock"),
                            ("/navcov", "/ublox_raw/navcov"),
                            ("/navheading", "/ublox_raw/navheading"),
                            ("/navrelposned", "/ublox_raw/navrelposned"),
                            ("/navstate", "/ublox_raw/navstate"),
                            ("/navsvin", "/ublox_raw/navsvin"),
                            ("/navstatus", "/ublox_raw/navstatus"),
                            ("/aideph", "/ublox_raw/aideph"),
                            ("/diagnostics", "/ublox_raw/diagnostics"),
                            ("/monhw", "/ublox_raw/monhw"),
                            ("/navsin", "/ublox_raw/nmea"),
                            ("/rtcm", "/ublox_raw/rtcm"),
                            ("/rxmrtcm", "/ublox_raw/rxmrtcm")],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='glider',
                plugin='GliderROS::GliderNode',
                name='glider_node',
                parameters=[
                    ros_params_file,
                    {'path': graph_params_file,
                     'use_sim_time': False,
                     'use_odom': False}
                ],
                remappings=[("/gps", "/ublox_gps_node/fix"), ("/imu", "/vectornav/imu")],
            ),

            ComposableNode(
                package='vectornav',
                plugin='vectornav::VnSensorMsgs',
                name='vn_sensor_msgs',
                parameters=[PJoin(
                    [FindPackageShare('vectornav'),
                     'config', 'vn_sensor_msgs_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output="screen",
    )
    return [container]




def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg("bag_prefix", default_value=["/home/dcist/data/symbiote_"], description="prefix of output bag"),
            # FLIR camera
            LaunchArg(
                'camera_parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=['camera_calibrations'],
                description='root directory for camera calibration files',
            ),
            # This is for the composed nodes
            OpaqueFunction(function=launch_setup),
        ]
    )
