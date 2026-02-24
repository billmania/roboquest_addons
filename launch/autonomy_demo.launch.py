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
        DeclareLaunchArgument(
            'turn_speed', default_value='0.5',
            description='Turn rate in radians per second'),
        DeclareLaunchArgument(
            'move_speed', default_value='0.1',
            description='Linear rate in meters per second'),
        DeclareLaunchArgument(
            'move_period', default_value='0.1',
            description='Seconds per logic loop'),
        DeclareLaunchArgument(
            'max_search_time', default_value='12.0',
            description='Seconds to spend searching for a tag'),
        DeclareLaunchArgument(
            'max_tag_distance_m', default_value='5.0',
            description='Max meters to a detectable tag'),
        DeclareLaunchArgument(
            'max_tag_bearing_rad', default_value='0.7854',
            description='Max radians off dead ahead for a detectable tag'),
        DeclareLaunchArgument(
            'max_tag_lost_s', default_value='0.5',
            description='Max seconds a tag can be lost after detection'),
        DeclareLaunchArgument(
            'obstacle_close_enough', default_value='0.3',
            description='Minimum distance in meters to approach a tag'),
        DeclareLaunchArgument(
            'ignore_ranges', default_value='1',
            description='Ignore this many nearest ranges'),
        DeclareLaunchArgument(
            'mean_ranges', default_value='3',
            description='Average this many nearest ranges'),
        DeclareLaunchArgument(
            'tag_trans_trim', default_value='0.0',
            description='Add this value to tag range'),
        DeclareLaunchArgument(
            'tag_trans_factor', default_value='1.0',
            description='Multiply tag range by this value'),
        DeclareLaunchArgument(
            'tag_close_enough', default_value='0.6',
            description='Distance to tag in meters'),
        DeclareLaunchArgument(
            'avoidance_dist_m', default_value='0.6',
            description='Back this many meters from obstacles'),
        DeclareLaunchArgument(
            'avoidance_cycles', default_value='10',
            description='Move ahead this many logic loops'),
        DeclareLaunchArgument(
            'ahead_angle_deg', default_value='10',
            description='Degrees of FOV considered ahead'),
        DeclareLaunchArgument(
            'side_angle_deg', default_value='20',
            description='Degress beyond ahead'),
        Node(
            package=package_name,
            executable='navigator.py',
            name='navigator',
            namespace='',
            parameters=[{
                'turn_speed': LaunchConfiguration('turn_speed'),
                'move_speed': LaunchConfiguration('move_speed'),
                'move_period': LaunchConfiguration('move_period'),
                'max_search_time': LaunchConfiguration('max_search_time'),
                'max_tag_distance_m':
                    LaunchConfiguration('max_tag_distance_m'),
                'max_tag_bearing_rad':
                    LaunchConfiguration('max_tag_bearing_rad'),
                'max_tag_lost_s': LaunchConfiguration('max_tag_lost_s'),
                'obstacle_close_enough':
                    LaunchConfiguration('obstacle_close_enough'),
                'ignore_ranges': LaunchConfiguration('ignore_ranges'),
                'mean_ranges': LaunchConfiguration('mean_ranges'),
                'tag_trans_trim': LaunchConfiguration('tag_trans_trim'),
                'tag_trans_factor': LaunchConfiguration('tag_trans_factor'),
                'tag_close_enough': LaunchConfiguration('tag_close_enough'),
                'avoidance_dist_m': LaunchConfiguration('avoidance_dist_m'),
                'avoidance_cycles': LaunchConfiguration('avoidance_cycles'),
                'ahead_angle_deg': LaunchConfiguration('ahead_angle_deg'),
                'side_angle_deg': LaunchConfiguration('side_angle_deg'),
            }],
            output='both'
        ),
        robot_state_publisher,
        joint_state_publisher
    ])
