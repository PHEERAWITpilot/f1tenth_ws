#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share dirs
    joy_vesc_pkg = get_package_share_directory('joy_vesc')
    odom_bringup_pkg = get_package_share_directory('odom_bringup')
    lidar_scan_pkg = get_package_share_directory('lidar_scan')
    robot_config_pkg = get_package_share_directory('robot_config')

    # Map and nav parameter args
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/f2/f1tenth_ws/src/map_create/maps/bigcapsule.yaml',
        description='Full path to map yaml file'
    )
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='/home/f2/f1tenth_ws/src/robot_config/config/nav2params.yaml',
        description='Full path to nav2 param file'
    )

    # Scan filter - limits LIDAR to ±120° forward
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[os.path.join(robot_config_pkg, 'config', 'scan_filter.yaml')],
        remappings=[
            ('/scan', '/scan_raw'),
            ('/scan_filtered', '/scan')
        ]
    )

    # twist_mux - command multiplexer
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[{
            'topics': {
                'joy': {
                    'topic': 'cmd_vel_joy',
                    'timeout': 0.5,
                    'priority': 100
                },
                'nav': {
                    'topic': 'cmd_vel_nav',
                    'timeout': 1.0,
                    'priority': 10
                }
            }
        }],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')
        ]
    )

    # cmd_vel_to_vesc translator
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
        
        # 4. Scan filter (±120° forward)
        scan_filter_node,
        
        # 5. twist_mux
        twist_mux_node,
        
        # 6. cmd_vel_to_vesc
        cmd_vel_to_vesc_node,
    ])

