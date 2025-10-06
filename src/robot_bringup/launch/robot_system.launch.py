from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_map_path = '/home/f2/f1tenth_ws/src/map_create/maps/map_20251003_030429_edited1.yaml'
    default_params_path = '/home/f2/f1tenth_ws/src/robot_config/config/nav2params.yaml'

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the nav2 parameters file'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_localization'), 'launch'),
            '/localization.launch.py'
        ]),
        launch_arguments={'map': map_file, 'params_file': params_file}.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_navigation'), 'launch'),
            '/navigation.launch.py'
        ]),
        launch_arguments={'map': map_file, 'params_file': params_file}.items()
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'global_costmap/global_costmap',
                'local_costmap/local_costmap'
            ]
        }]
    )

    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Initial pose publisher node with delay
    initial_pose_node = Node(
        package='initial_pose_publisher',
        executable='publish_initial_pose',
        name='initial_pose_publisher',
        output='screen'
    )

    # Delay initial pose publication to ensure localization is ready
    initial_pose_delayed = TimerAction(
        period=5.0,  # Wait 5 seconds after launch
        actions=[initial_pose_node]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        localization_launch,
        navigation_launch,
        behavior_server_node,
        lifecycle_manager_loc,
        lifecycle_manager_nav,
        initial_pose_delayed,  # Add delayed initial pose publisher
    ])

