import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('image_to_gst'),
        'config',
        'parameters.yaml'
    )

    fpv_gst_node = Node(
        package='image_to_gst',
        executable='image_to_gst',
        name='fpv_gst',
        parameters=[config],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        fpv_gst_node
    ])
