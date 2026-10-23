import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Configs
    bringup_share = get_package_share_directory('bringup')
    foxglove_params = os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[foxglove_params],
    )

    real_image_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bringup'),
                'launch',
                'real_image.launch.py'
            )
        ),
    )

    go2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_robot_sdk'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'foxglove': 'false',
            'joystick': 'false',
            'teleop': 'false',
            'slam': 'false',
            'rviz2': 'false',
            'nav2': 'false',
        }.items()
    )

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('follower'),
                'launch',
                'follower.launch.py'
            )
        ),
    )

    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tracker'),
                'launch',
                'tracker.launch.py'
            )
        ),
    )

    return [
        foxglove_bridge_node,
        tracker_launch,
        real_image_launch,
        go2_launch,
        follower_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])