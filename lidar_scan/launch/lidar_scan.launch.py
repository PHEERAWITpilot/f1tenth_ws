#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directories
    urg_node2_pkg = get_package_share_directory('urg_node2')
    my_tf_broadcaster_pkg = get_package_share_directory('my_tf_broadcaster')
    
    # ✅ Default URG config path (navigation 140° FOV)
    default_params_file = os.path.join(
        urg_node2_pkg, 'config', 'params_ether.yaml'
    )
    
    # ✅ Declare launch argument for params_file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to URG parameter YAML file'
    )
    
    return LaunchDescription([
        # ✅ Declare the params_file argument
        params_file_arg,
        
        # ✅ Include urg_node2 launch file and pass params_file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urg_node2_pkg, 'launch', 'urg_node2.launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file')
            }.items(),
        ),
        
        # Include tf broadcaster launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_tf_broadcaster_pkg, 'launch', 'my_tf_boardcaster.launch.py')
            ),
        ),
    ])

