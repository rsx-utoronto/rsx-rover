from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_filename = LaunchConfiguration('bag_filename')
    rover_share_dir = get_package_share_directory('rover')
    rviz_config_path = os.path.join(rover_share_dir, 'rover', 'autonomy', 'config', 'octomap.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_filename',
            description='Path to the ROS 2 bag file to play'
        ),

        # Octomap Server Node
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'frame_id': 'map',
                'sensor_model/max_range': 5.0,
                'height_map': False,
                'occupancy_min_z': 0.6,
                'occupancy_max_z': 1.5,
                'use_sim_time': True
            }],
            remappings=[
                ('cloud_in', '/zed_node/point_cloud/cloud_registered')
            ]
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='octomap_rviz',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # Rosbag play node
        Node(
            package='ros2bag',
            executable='play',
            name='playbag',
            arguments=['--clock', bag_filename],
            output='screen'
        )
    ])