#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    joy_vesc_pkg = get_package_share_directory('joy_vesc')
    odom_bringup_pkg = get_package_share_directory('odom_bringup')
    lidar_scan_pkg = get_package_share_directory('lidar_scan')
    my_pathfinder_pkg = get_package_share_directory('my_pathfinder')
    
    return LaunchDescription([
        # 1. Launch joystick and VESC control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(joy_vesc_pkg, 'launch', 'joy_vesc.launch.py')
            ),
        ),
        
        # 2. Launch odometry system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odom_bringup_pkg, 'launch', 'odom_bringup.launch.py')
            ),
        ),
        
        # 3. Launch LiDAR and static transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_scan_pkg, 'launch', 'lidar_scan.launch.py')
            ),
        ),
        
        # 4. Launch SLAM mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_pathfinder_pkg, 'launch', 'slam.launch.py')
            ),
        ),
    ])


