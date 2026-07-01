from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    action_server_name = LaunchConfiguration('action_server_name')
    referee_topic = LaunchConfiguration('referee_topic')
    route_topic = LaunchConfiguration('route_topic')
    reset_route_on_goal_result = LaunchConfiguration('reset_route_on_goal_result')

    demo_cmd = Node(
        package='rm_decision',
        executable='rmuc_decision_qianshao',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'action_server_name': action_server_name,
            'referee_topic': referee_topic,
            'route_topic': route_topic,
            'reset_route_on_goal_result': ParameterValue(
                reset_route_on_goal_result,
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('action_server_name', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('referee_topic', default_value='/offboardlink/reference_data_mini'),
        DeclareLaunchArgument('route_topic', default_value='/tunnel_route_id'),
        DeclareLaunchArgument('reset_route_on_goal_result', default_value='true'),
        demo_cmd,
    ])
