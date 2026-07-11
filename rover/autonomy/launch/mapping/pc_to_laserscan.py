from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('/zed_node/point_cloud/cloud_registered', 'cloud_in'),
                ('/scan', 'scan')
            ]
        )
    ])