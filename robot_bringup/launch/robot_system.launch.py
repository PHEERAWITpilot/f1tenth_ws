from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_map_path = '/home/f2/f1tenth_ws/src/map_create/maps/bigcapsule.yaml'
    default_params_path = '/home/f2/f1tenth_ws/src/robot_config/config/nav2params.yaml'
    default_bt_xml_path = '/home/f2/f1tenth_ws/src/robot_config/config/ackermann_navigate_to_pose.xml'

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

    # Navigation server nodes
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
        parameters=[params_file],
        respawn=False
    )

    # Initial pose publisher - uses retry logic internally
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
            'bond_timeout': 30.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 30.0,
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

    # OPTIMIZED TIMING SEQUENCE
    # 1. Start localization lifecycle manager (map + AMCL)
    lifecycle_manager_loc_timed = TimerAction(
        period=5.0,  # Localization starts at 5s
        actions=[lifecycle_manager_loc]
    )

    # 2. Publish initial pose - now with more delay for AMCL to be ready
    #    AND using retry publisher that publishes 5 times
    initial_pose_delayed = TimerAction(
        period=12.0,  # Increased to 12s - gives AMCL time to fully initialize
        actions=[initial_pose_node]
    )

    # 3. Start navigation lifecycle manager (controller, planner, behaviors)
    lifecycle_manager_nav_timed = TimerAction(
        period=25.0,  # Navigation starts at 25s - after AMCL is localized
        actions=[lifecycle_manager_nav]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_bt_xml_cmd,
        
        # Launch nodes immediately
        localization_launch,
        navigation_launch,
        behavior_server_node,
        bt_navigator_node,
        
        # Timed lifecycle managers (sequential startup)
        lifecycle_manager_loc_timed,      # 5s: Start localization
        initial_pose_delayed,             # 12s: Set initial pose (with retries)
        lifecycle_manager_nav_timed,      # 25s: Start navigation
    ])

