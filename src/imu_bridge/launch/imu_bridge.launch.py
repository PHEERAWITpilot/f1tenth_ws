#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('imu_bridge'),
            'config',
            'imu_bridge_params.yaml'
        ]),
        description='Path to config file'
    )
    
    # Arduino Bridge Node
    arduino_bridge_node = Node(
        package='imu_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        arduino_bridge_node
    ])

