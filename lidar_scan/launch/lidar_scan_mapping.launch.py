#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Get package directories
    urg_node2_pkg = get_package_share_directory('urg_node2')
    my_tf_broadcaster_pkg = get_package_share_directory('my_tf_broadcaster')
    
    # ✅ Load MAPPING parameters (270° FOV)
    mapping_config_path = os.path.join(
        urg_node2_pkg, 'config', 'params_ether_mapping.yaml'
    )
    
    with open(mapping_config_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']
    
    # ✅ Create URG node directly with mapping parameters
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        remappings=[('scan', 'scan')],
        parameters=[config_params],
        namespace='',
        output='screen',
    )
    
    # Auto-start: configure transition
    urg_node2_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )
    
    # Auto-start: activate transition
    urg_node2_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )
    
    # TF broadcaster
    tf_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_tf_broadcaster_pkg, 'launch', 'my_tf_boardcaster.launch.py')
        ),
    )
    
    return LaunchDescription([
        lifecycle_node,
        urg_node2_configure,
        urg_node2_activate,
        tf_broadcaster_launch,
    ])
