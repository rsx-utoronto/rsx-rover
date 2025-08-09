from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    tf_map_scanmatch_transform_frame_name = LaunchConfiguration('tf_map_scanmatch_transform_frame_name')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    pub_map_odom_transform = LaunchConfiguration('pub_map_odom_transform')
    scan_subscriber_queue_size = LaunchConfiguration('scan_subscriber_queue_size')
    scan_topic = LaunchConfiguration('scan_topic')
    map_size = LaunchConfiguration('map_size')

    rover_share_dir = get_package_share_directory('rover')
    rviz_config_path = os.path.join(rover_share_dir, 'rover', 'autonomy', 'config', 'hector_slam_occupancy_grid.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('tf_map_scanmatch_transform_frame_name', default_value='scanmatcher_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('pub_map_odom_transform', default_value='false'),
        DeclareLaunchArgument('scan_subscriber_queue_size', default_value='5'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('map_size', default_value='2048'),

        # Hector Mapping Node
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'use_tf_scan_transformation': True,
                'use_tf_pose_start_estimate': False,
                'pub_map_odom_transform': pub_map_odom_transform,
                'map_resolution': 0.050,
                'map_size': map_size,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'map_multi_res_levels': 2,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'map_update_distance_thresh': 0.4,
                'map_update_angle_thresh': 0.06,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
                'advertise_map_service': True,
                'scan_subscriber_queue_size': scan_subscriber_queue_size,
                'scan_topic': scan_topic,
                'tf_map_scanmatch_transform_frame_name': tf_map_scanmatch_transform_frame_name
            }]
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_hector_slam',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])