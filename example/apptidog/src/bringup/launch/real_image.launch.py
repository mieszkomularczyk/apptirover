from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------- Config files ----------
    osd_params = PathJoinSubstitution([FindPackageShare('osd'), 'config', 'osd.yaml'])
    image_to_gst_params = PathJoinSubstitution(
        [FindPackageShare('image_to_gst'), 'config', 'parameters.yaml'])
    yolo_rgb_params = PathJoinSubstitution([FindPackageShare('yolo'), 'config', 'yolo.yaml'])

    gst_to_image_rgb_params = PathJoinSubstitution(
        [FindPackageShare('gst_to_image'), 'config', 'usb_camera.yaml'])

    IPC = [{'use_intra_process_comms': True}]

    # ================================================================
    #  RGB pipeline:  gst_to_image  →  osd  →  image_to_gst
    # ================================================================
    rgb_vision_container = ComposableNodeContainer(
        name='rgb_vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # --- Camera ---
            ComposableNode(
                package='gst_to_image',
                plugin='GstToImage',
                name='usb_camera',
                parameters=[gst_to_image_rgb_params],
                extra_arguments=IPC,
            ),
            # --- YOLO ---
            ComposableNode(
                package='yolo',
                plugin='YoloNode',
                name='yolo',
                parameters=[yolo_rgb_params],
                extra_arguments=IPC,
            ),
            # --- OSD ---
            ComposableNode(
                package='osd',
                plugin='osd::OsdNode',
                name='osd',
                parameters=[osd_params],
                extra_arguments=IPC,
            ),
            # --- GStreamer UDP streamer ---
            ComposableNode(
                package='image_to_gst',
                plugin='ImageToGst',
                name='stream',
                parameters=[image_to_gst_params],
                extra_arguments=IPC,
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        rgb_vision_container
    ])
