from launch.actions import TimerAction
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': False}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ✅ <-- THE FIX (delay lifecycle_manager_localization start)
    lifecycle_manager_localization_delayed = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                    'bond_timeout': 15.0,
                    'attempt_respawn_reconnection': True,
                    'bond_respawn_max_duration': 10.0
                }]
            )
        ]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        static_tf_node,
        lifecycle_manager_localization_delayed  # ✅ Delay applied here
    ])

