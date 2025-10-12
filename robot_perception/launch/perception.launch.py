from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    global_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='global_costmap',
        name='global_costmap',
        output='screen',
        parameters=[params_file]
    )

    local_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        global_costmap_node,
        local_costmap_node
    ])

