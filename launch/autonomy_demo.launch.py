"""Roboquest Autonomy Demo launch file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
        Command,
        LaunchConfiguration,
        PathJoinSubstitution
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Define an RPLiDAR and april_tag launch description."""
    package_name = 'roboquest_addons'
    pkg_share = FindPackageShare(package_name)

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Launch RP LiDAR node'
    )

    april_params_file = PathJoinSubstitution(
        [pkg_share,
         'config',
         'tags_36h11.yaml']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    PathJoinSubstitution(
                        [pkg_share,
                         'urdf',
                         'rq_tracked.urdf']
                    )
                ])
            ),
            'publish_frequency': 30.0
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        use_rviz_arg,
        use_lidar_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
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
            output='both',
            condition=IfCondition(LaunchConfiguration('use_lidar'))
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
                        april_params_file
                    ]
                )
            ],
            output='both'
        ),
        Node(
            package=package_name,
            executable='navigator.py',
            name='navigator',
            namespace='',
            output='both'
        ),
        robot_state_publisher,
        joint_state_publisher
    ])
