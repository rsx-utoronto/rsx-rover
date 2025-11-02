from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Static transform broadcaster (base_link -> zed2_base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_broadcaster',
            arguments=[
                '0.33', '0.0', '0.0',  # x y z
                '0', '0', '0',         # roll pitch yaw
                'base_link', 'zed2_base_link', '100'
            ]
        ),

        # Include ZED launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py'
                )
            ),
            launch_arguments={
                'camera_model': 'zed2i',
                'sim_mode': 'false'
            }.items()
        ),

        # PointCloud to LaserScan node
        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     name='collapse_pointcloud',
        #     output='screen',
        #     parameters=[{
        #         'max_height': 1.0,
        #         'min_height': -0.5,
        #         'range_min': 0.75,
        #         'range_max': 5.0
        #     }],
        #     remappings=[('cloud_in', '/zed_node/point_cloud/cloud_registered')]
        # )
    ])