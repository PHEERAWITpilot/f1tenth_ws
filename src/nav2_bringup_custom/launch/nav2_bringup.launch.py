#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get other package share dirs
    joy_vesc_pkg = get_package_share_directory('joy_vesc')
    odom_bringup_pkg = get_package_share_directory('odom_bringup')
    lidar_scan_pkg = get_package_share_directory('lidar_scan')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')

    # Map and nav parameter args for navigation system
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/f2/f1tenth_ws/src/map_create/map/map_20251002_002740_edited.yaml',
        description='Full path to map yaml file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='/home/f2/f1tenth_ws/src/robot_config/config/nav2param.yaml',
        description='Full path to nav2 param file'
    )

    # cmd_vel_to_vesc node
    cmd_vel_to_vesc_node = Node(
        package='cmd_vel_to_vesc',
        executable='cmd_vel_to_vesc_node',
        name='cmd_vel_to_vesc',
        output='screen',
        parameters=[{
            'min_motor_speed': 2500.0,
            'max_speed_rpm': 6000.0,
            'servo_min': 0.0,
            'servo_center': 0.5,
            'servo_max': 1.0
        }]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        # 1. Joystick and VESC
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(joy_vesc_pkg, 'launch', 'joy_vesc.launch.py')
            )
        ),
        # 2. Odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odom_bringup_pkg, 'launch', 'odom_bringup.launch.py')
            )
        ),
        # 3. LiDAR bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_scan_pkg, 'launch', 'lidar_scan.launch.py')
            )
        ),
        # 4. cmd_vel_to_vesc translator node
        cmd_vel_to_vesc_node,
        # 5. Nav2 system launch (currently commented out)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(robot_bringup_pkg, 'launch', 'robot_system.launch.py')
        #     ),
        #     launch_arguments={'map': map_file, 'params_file': params_file}.items()
        # )
    ])

