from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare("yolo"), "config", "yolo.yaml"])

    IPC = [{'use_intra_process_comms': True}]

    yolo_container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolo',
                plugin='YoloNode',
                name='yolo_node',
                parameters=[params_file],
                extra_arguments=IPC,
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        yolo_container,
    ])
