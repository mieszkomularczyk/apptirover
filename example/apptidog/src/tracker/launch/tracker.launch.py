from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tracker_params = PathJoinSubstitution(
        [FindPackageShare("tracker"), "config", "tracker.yaml"]
    )

    tracker_node = Node(
        package="tracker",
        executable="tracker_node",
        name="tracker",
        output="screen",
        emulate_tty=True,
        parameters=[tracker_params],
    )

    return LaunchDescription([
        tracker_node,
    ])
