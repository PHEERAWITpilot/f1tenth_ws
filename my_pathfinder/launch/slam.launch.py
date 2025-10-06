#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    my_pathfinder_pkg = get_package_share_directory('my_pathfinder')
    
    # Path to config file
    slam_config_file = os.path.join(my_pathfinder_pkg, 'config', 'slam_config.yaml')
    
    return LaunchDescription([
        # SLAM Toolbox Node with custom config
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_file],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
    ])

