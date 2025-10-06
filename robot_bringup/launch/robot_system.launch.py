from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_map_path = '/home/f2/f1tenth_ws/src/map_create/maps/circle2.yaml'
    default_params_path = '/home/f2/f1tenth_ws/src/robot_config/config/nav2params.yaml'
    default_bt_xml_path = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml')

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
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml_path,
        description='Full path to the behavior tree xml file'
    )

    # Localization nodes
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_localization'), 'launch'),
            '/localization.launch.py'
        ]),
        launch_arguments={'map': map_file, 'params_file': params_file}.items()
    )

    # Navigation server nodes (controller, planner - they create costmaps internally)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_navigation'), 'launch'),
            '/navigation.launch.py'
        ]),
        launch_arguments={'params_file': params_file}.items()
    )

    # Behavior and BT nodes
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        respawn=False
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file,
            {'default_nav_to_pose_bt_xml': bt_xml_file}
        ],
        respawn=False
    )

    # Initial pose publisher
    initial_pose_node = Node(
        package='initial_pose_publisher',
        executable='publish_initial_pose',
        name='initial_pose_publisher',
        output='screen'
    )

    # Lifecycle Manager for Localization
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 30.0,  # Increased
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Lifecycle Manager for Navigation
    # CRITICAL: Must wait for costmaps to finish "Creating" phase
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 30.0,  # Increased
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    # Timed launch sequence
    lifecycle_manager_loc_timed = TimerAction(
        period=5.0,
        actions=[lifecycle_manager_loc]
    )

    initial_pose_delayed = TimerAction(
        period=8.0,
        actions=[initial_pose_node]
    )

    # CRITICAL: Wait 25 seconds for costmaps to fully initialize
    lifecycle_manager_nav_timed = TimerAction(
        period=25.0,  # Increased significantly
        actions=[lifecycle_manager_nav]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_bt_xml_cmd,
        
        # Launch nodes
        localization_launch,
        navigation_launch,
        behavior_server_node,
        bt_navigator_node,
        
        # Timed lifecycle managers
        lifecycle_manager_loc_timed,
        initial_pose_delayed,
        lifecycle_manager_nav_timed,
    ])

