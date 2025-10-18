"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os
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


import yaml


# --------- master_example.launch.py


camera_list = {
    'cam0': '23540201',
}

serial = '23540201'
camera_type = 'blackfly_s'
parameter_file = PJoin(
        [FindPackageShare('spinnaker_camera_driver'), 'config', camera_type + '.yaml']
    )

exposure_controller_parameters = {
    'type': 'master',
    'brightness_target': 120,  # from 0..255
    'brightness_tolerance': 20,  # when to update exposure/gain
    # watch that max_exposure_time is short enough
    # to support the trigger frame rate!
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 5000,  # usec
    'max_gain': 29.9,
    'gain_priority': False,
}

camera_params = {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'exposure_auto': 'Continuous',
    'exposure_time': 10000,  # not used under auto exposure
    'trigger_mode': 'Off',
    'frame_rate_auto': 'Off',
    'frame_rate_enable': True,
    'gain_auto': 'Continuous',
    'trigger_source': 'Software',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'trigger_activation': 'RisingEdge',
    'balance_white_auto': 'Continuous',
    # You must enable chunk mode and chunks: frame_id, exposure_time, and gain
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    # The Timestamp is not used at the moment
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    'frame_rate': 5
}


def make_parameters(context):
    """Launch synchronized camera driver node."""
    pd = LaunchConfig('camera_parameter_directory')
    calib_url = 'file://' + LaunchConfig('calibration_directory').perform(context) + '/'

    exp_ctrl_names = [cam + '.exposure_controller' for cam in camera_list.keys()]
    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'exposure_controllers': exp_ctrl_names,
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',  # only for ffmpeg image transport
    }
    # generate identical exposure controller parameters for all cameras
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + '.' + k: v for k, v in exposure_controller_parameters.items()}
        )

    # generate camera parameters
    cam_parameters['parameter_file'] = PJoin([pd, 'blackfly_s.yaml'])
    for cam, serial in camera_list.items():
        cam_params = {cam + '.' + k: v for k, v in cam_parameters.items()}
        cam_params[cam + '.serial_number'] = serial
        cam_params[cam + '.camerainfo_url'] = calib_url + serial + '.yaml'
        cam_params[cam + '.frame_id'] = cam
        driver_parameters.update(cam_params)  # insert into main parameter list
        # link the camera to its exposure controller. Each camera has its own controller
        driver_parameters.update({cam + '.exposure_controller_name': cam + '.exposure_controller'})
    return driver_parameters


# ----------------------------------

def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    # For the camera
    #cam_name = LaunchConfig("camera_name")
    #cam_str = cam_name.perform(context)

    # For the GPS
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    param_config = os.path.join(config_directory, 'zed_f9p.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['ublox_gps_node']['ros__parameters']

    # For EC
    # bias_config = os.path.join(
    #     ament_index_python.packages.get_package_share_directory('high_altitude_ec'),
    #     'config/silky_ev_all_zero.bias')
    bias_config = "/home/dcist/fclad/ROS/high_altitude_env/src/high_altitude_ec/config/silkyHD_all_zero.bias"
    # Declare launch arguments

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
        'vectornav-vn100t.yaml'
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
                        "/odom",
                        "/vectornav/imu",
                        "/vectornav/magnetic",
                        "/cam_driver/image_raw"
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

            # FLIR camera
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name="cam_driver",
                parameters=[camera_params, {'parameter_file': parameter_file, 'serial_number': serial}],
                remappings=[
                    ('~/control', '/exposure_control/control'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
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
            #Node(
            #    package="sf000_driver",
            #    executable="reader.py",
            #    name="reader",
            #    remappings=[("/range", "/sf000/range")],
            #)
        ]
    )
