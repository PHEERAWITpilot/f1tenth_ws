from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('vesc_speed_calculator')
    
    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'vesc_speed_params.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # VESC Speed Calculator Node
    vesc_speed_node = Node(
        package='vesc_speed_calculator',
        executable='vesc_speed_calculator',
        name='vesc_speed_calculator',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        vesc_speed_node
    ])

