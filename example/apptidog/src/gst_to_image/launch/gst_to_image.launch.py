import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gst_to_image'),
        'config',
        'parameters.yaml'
    )

    rgb_receiver_node = Node(
        package='gst_to_image',
        executable='gst_to_image',
        name='receiver',
        namespace='rgb_vision',
        parameters=[config],
        output='screen',
        emulate_tty=True
    )

    thermal_receiver_node = Node(
        package='gst_to_image',
        executable='gst_to_image',
        name='receiver',
        namespace='thermal_vision',
        parameters=[config],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        rgb_receiver_node,
        thermal_receiver_node
    ])
