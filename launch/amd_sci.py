import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rover',
            executable='manual_switch.py', #note this was manual_control
            name='rover_manual_control',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rover',
            executable='imu_to_enu.py',
            name='imu_to_enu',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madqwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[
                {
                    'publish_tf': 'false'
                },
                {
                    'use_mag': 'true'
                },
                {
                    'world_frame': 'enu'
                },
                {
                    'orientation_stddev': '0.2'
                }
            ],
        
        remappings=[
            ('imu/mag', '/zed_node/imu/mag'),
            ('imu/data_raw', '/imu/enu'),
            ('imu/data', '/imu/orient')
        ]

        ),
        launch_ros.actions.Node(
            package='rover',
            executable='check_zed.py',
            name='check_zed',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rover',
            executable='geniecamerapublisher.py',
            name='geniecam_publisher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rover',
            executable='microscopecam.py',
            name='microscopecam',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rover',
            executable='webcam.py',
            name='webcam_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rover',
            executable='science_arduino_transform.py',
            name='science_arduino_transform',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rover'), 'launch/rs_multiple_devices.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'calian_gnss_ros2'), 'launch/moving_baseline_rtk.launch.py')
            )
        )
    ])
    
    return ld


if __name__ == '__main__':
    generate_launch_description()