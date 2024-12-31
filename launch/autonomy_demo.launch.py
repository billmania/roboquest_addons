"""Roboquest Autonomy Demo launch file."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Define an RPLiDAR and april_tag launch description."""
    april_params = os.path.join(
        get_package_share_directory('roboquest_addons'),
        'config',
        'tags_36h11.yaml'
    )

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            namespace='',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 460800,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            output='both'
        ),
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='apriltag_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    namespace='',
                    remappings=[
                        ('/image_rect', '/rq_camera_node0/image_raw'),
                        ('/camera_info', '/rq_camera_node0/camera_info')
                    ],
                    parameters=[
                        april_params
                    ]
                )
            ],
            output='both'
        )
    ])
