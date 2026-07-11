from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('rover'),
        'rover',
        'autonomy',
        'config',
        'octomap.rviz'
    )

    return LaunchDescription([
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='octomap_rviz',
            arguments=['-d', rviz_config_path],
            output='screen'
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
        )
    ])