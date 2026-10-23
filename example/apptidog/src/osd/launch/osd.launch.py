from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([FindPackageShare("osd"), "config", "osd.yaml"])

    # ---------- Option A: Composable container (IPC zero-copy) ----------
    osd_container = ComposableNodeContainer(
        name='osd_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='osd',
                plugin='osd::OsdNode',
                name='fpv_osd',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        osd_container,
    ])
