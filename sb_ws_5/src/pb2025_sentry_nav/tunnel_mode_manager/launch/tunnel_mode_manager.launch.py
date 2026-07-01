from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('tunnel_mode_manager'),
        'config',
        'tunnel_mode_manager.yaml',
    ])

    return LaunchDescription([
        Node(
            package='tunnel_mode_manager',
            executable='tunnel_mode_manager_node',
            name='tunnel_mode_manager',
            output='screen',
            parameters=[params_file],
        ),
    ])
