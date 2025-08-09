from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    rover_share_dir = get_package_share_directory('rover')

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        # slam_gmapping node
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[
                os.path.join(rover_share_dir, 'rover', 'simulation', 'config', 'gmapping_params.yaml'),
                {
                    'map_frame': 'map',
                    'base_frame': base_frame,
                    'odom_frame': odom_frame,
                    'use_sim_time': False
                }
            ],
            remappings=[
                ('scan', scan_topic)
            ]
        ),

        # rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_gmapping',
            arguments=['-d', os.path.join(rover_share_dir, 'rover', 'autonomy', 'config', 'gmapping.rviz')],
            output='screen'
        )
    ])