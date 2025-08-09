import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Manual control
        Node(
            package='rover',
            executable='manual_switch.py',  # Renamed from 'manual_control'
            name='rover_manual_control',
            output='screen'
        ),

        # Realsense cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rover'),
                    'launch',
                    'rs_multiple_devices.py'
                )
            )
        ),

        # IMU to ENU converter
        Node(
            package='rover',
            executable='imu_to_enu.py',
            name='imu_to_enu',
            output='screen'
        ),

        # Madgwick IMU filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',  # Correct executable for ROS 2
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'publish_tf': False,
                'use_mag': True,
                'world_frame': 'enu',
                'orientation_stddev': 0.2
            }],
            remappings=[
                ('imu/mag', '/zed_node/imu/mag'),
                ('imu/data_raw', '/imu/enu'),
                ('imu/data', '/imu/orient')
            ]
        ),

        # # GNSS moving baseline RTK
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('calian_gnss_ros2'),
        #             'launch',
        #             'moving_baseline_rtk.py'
        #         )
        #     )
        # ),

        # ZED camera check node
        Node(
            package='rover',
            executable='check_zed.py',
            name='check_zed',
            output='screen'
        ),

        # Genie camera publisher
        Node(
            package='rover',
            executable='geniecamerapublisher.py',
            name='geniecam_publisher',
            output='screen'
        ),

        # Microscope camera
        Node(
            package='rover',
            executable='microscopecam.py',
            name='microscopecam',
            output='screen'
        ),

        # Web camera
        Node(
            package='rover',
            executable='webcam.py',
            name='webcam_node',
            output='screen'
        ),

        # Science Arduino transform
        Node(
            package='rover',
            executable='science_arduino_transform.py',
            name='science_arduino_transform',
            output='screen'
        ),

        # Optional LED nodes (commented)
        # Node(
        #     package='rover',
        #     executable='led_light.py',
        #     name='led_listener',
        #     output='screen'
        # ),
        # Node(
        #     package='rover',
        #     executable='pub_manual_indicator.py',
        #     name='pub_manual_indicator',
        #     output='screen'
        # )
    ])