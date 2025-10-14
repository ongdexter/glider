"""
    Jason Hughes
    June 2025

    launch glider as an
    independent node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # create logging directory
    os.makedirs("/home/jason/.ros/log/glider", exist_ok=True)

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_odom_arg = DeclareLaunchArgument(
        'use_odom',
        default_value='false',
        description='Use odometry'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_odom = LaunchConfiguration('use_odom')
    
    # Find package share directory
    glider_share = FindPackageShare('glider')
    
    # Path to parameter files
    ros_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'ros-params.yaml'
    ])
    
    graph_params_file = PathJoinSubstitution([
        glider_share,
        'config',
        'glider-params.yaml'
    ])
     
    # Create the glider node
    glider_node = Node(
        package='glider',
        executable='glider_node',
        name='glider_node',
        output='screen',
        parameters=[
            ros_params_file,
            {'path': graph_params_file,
             'use_sim_time': use_sim_time,
             'use_odom': use_odom}
        ],
        remappings=[
            ('/gps', '/ublox/fix'),
            ('/imu', '/VN100T/imu'),
            ('/odom', '/Odometry'),
        ]
    )
    
    return LaunchDescription([use_sim_time_arg, use_odom_arg, glider_node])
