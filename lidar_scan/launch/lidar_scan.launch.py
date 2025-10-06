#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    urg_node2_pkg = get_package_share_directory('urg_node2')
    my_tf_broadcaster_pkg = get_package_share_directory('my_tf_broadcaster')
    
    return LaunchDescription([
        # Include urg_node2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urg_node2_pkg, 'launch', 'urg_node2.launch.py')
            ),
        ),
        
        # Include tf broadcaster launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_tf_broadcaster_pkg, 'launch', 'my_tf_boardcaster.launch.py')
            ),
        ),
    ])

