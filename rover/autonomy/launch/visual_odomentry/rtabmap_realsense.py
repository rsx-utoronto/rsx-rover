from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    rtabmap_launch_dir = os.path.join(
        FindPackageShare('rtabmap_launch').find('rtabmap_launch'),
        'launch'
    )

    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('imu/data_raw', '/camera/imu'),
                ('imu/data', '/rtabmap/imu')
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')
            ),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'wait_imu_to_init': 'true',
                'imu_topic': '/rtabmap/imu',
                'rtabmap_viz': 'true',
                'rviz': 'false'
            }.items()
        )
    ])