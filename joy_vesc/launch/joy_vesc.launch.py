#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    joystick_device_arg = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    vesc_port_arg = DeclareLaunchArgument(
        'vesc_port',
        default_value='/dev/ttyACM0',
        description='VESC serial port'
    )
    
    # Get package directories
    joy_vesc_dir = get_package_share_directory('joy_vesc')
    vesc_driver_dir = get_package_share_directory('vesc_driver')
    vesc_speed_calc_dir = get_package_share_directory('vesc_speed_calculator')
    
    # Joy node configuration file
    joy_config = os.path.join(joy_vesc_dir, 'config', 'joy_vesc_params.yaml')
    
    # 1. Joy Node - REMAPPED TO /cmd_vel_joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy'),  # ← CRITICAL: Joystick publishes to /cmd_vel_joy
        ],
        output='screen'
    )
    
    # 2. VESC Driver Launch
    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vesc_driver'),
                'launch',
                'vesc_driver_node.launch.py'
            ])
        ]),
        launch_arguments={
            'port': LaunchConfiguration('vesc_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # 3. VESC Speed Calculator Launch
    vesc_speed_calc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vesc_speed_calculator'),
                'launch',
                'vesc_speed_calculator.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        joystick_device_arg,
        vesc_port_arg,
        
        # Nodes and launches
        joy_node,
        vesc_driver_launch,
        vesc_speed_calc_launch
    ])

