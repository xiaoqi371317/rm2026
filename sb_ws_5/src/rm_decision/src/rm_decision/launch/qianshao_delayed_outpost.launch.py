from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    action_server_name = LaunchConfiguration('action_server_name')
    referee_topic = LaunchConfiguration('referee_topic')
    route_topic = LaunchConfiguration('route_topic')
    posture_topic = LaunchConfiguration('posture_topic')
    debug_topic = LaunchConfiguration('debug_topic')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('rm_decision'),
        'config',
        'qianshao_delayed_outpost.yaml',
    ])

    decision_node = Node(
        package='rm_decision',
        executable='rmuc_decision_qianshao_delayed_outpost',
        name='smart_qianshao_decision_node',
        emulate_tty=True,
        output='screen',
        parameters=[
            params_file,
            {
                'action_server_name': action_server_name,
                'referee_topic': referee_topic,
                'route_topic': route_topic,
                'posture_topic': posture_topic,
                'debug_topic': debug_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('action_server_name', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('referee_topic', default_value='/offboardlink/reference_data_mini'),
        DeclareLaunchArgument('route_topic', default_value='/tunnel_route_id'),
        DeclareLaunchArgument('posture_topic', default_value='/sentry/status/posture'),
        DeclareLaunchArgument('debug_topic', default_value='/sentry/decision_state'),
        decision_node,
    ])
