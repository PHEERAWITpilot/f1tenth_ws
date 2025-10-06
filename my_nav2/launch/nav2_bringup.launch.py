#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    my_nav2_pkg = get_package_share_directory('my_nav2')

    params_file = os.path.join(my_nav2_pkg, 'config', 'nav2_params.yaml')
    bringup_launch = os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])

