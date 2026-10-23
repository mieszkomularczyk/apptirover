from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    follower_params_file = PathJoinSubstitution(
        [FindPackageShare('follower'), 'config', 'follower.yaml']
    )

    target_tracker_params_file = PathJoinSubstitution(
        [FindPackageShare('follower'), 'config', 'target_tracker.yaml']
    )

    follower_node = Node(
        package='follower',
        executable='follower_node',
        name='follower',
        output='screen',
        emulate_tty=True,
        parameters=[follower_params_file],
    )

    target_tracker_node = Node(
        package='follower',
        executable='target_tracker_node',
        name='target_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[target_tracker_params_file],
    )

    return LaunchDescription([
        follower_node,
        target_tracker_node,
    ])
