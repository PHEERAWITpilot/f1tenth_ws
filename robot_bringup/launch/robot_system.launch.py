from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ✅ Correct file paths for your workspace
    default_map_path = '/home/f2/f1tenth_ws/src/map_create/maps/zigzags.yaml'
    default_params_path = '/home/f2/f1tenth_ws/src/robot_config/config/nav2params.yaml'
    default_bt_xml_path = '/home/f2/f1tenth_ws/src/robot_config/config/ackermann_navigate_through_poses.xml'

    # Launch configurations with correct variable
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map YAML file'
    )

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the nav2 parameters file'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml',
        default_value=default_bt_xml_path,
        description='Full path to the behavior tree XML file'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_localization'), 'launch'),
            '/localization.launch.py'
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_navigation'), 'launch'),
            '/navigation.launch.py'
        ]),
        launch_arguments={
            'params_file': params_file
        }.items()
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        respawn=False
    )

    # ✅ BT Navigator: pass params_file AND bt_xml_file (LaunchConfiguration)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
           
            'default_bt_xml_filename': '/home/f2/f1tenth_ws/src/robot_config/config/ackermann_navigate_through_poses.xml'
    
        }],
        respawn=False
    )

    initial_pose_delayed = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='initial_pose_publisher',
                executable='publish_initial_pose',
                name='initial_pose_publisher',
                output='screen'
            )
        ]
    )

    lifecycle_manager_nav_delayed = TimerAction(
        period=15.0,
        actions=[
            Node(
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
        ]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_bt_xml_cmd,
        localization_launch,
        navigation_launch,
        behavior_server_node,
        bt_navigator_node,       # <-- this line now passes your XML correctly!
        initial_pose_delayed,
        lifecycle_manager_nav_delayed,
    ])

