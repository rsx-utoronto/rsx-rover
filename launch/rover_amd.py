from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rover_launch_dir = os.path.join(get_package_share_directory('rover'), 'launch')
    gnss_launch_dir = os.path.join(get_package_share_directory('calian_gnss_ros2'), 'launch')

    return LaunchDescription([

        # Manual Control
        Node(
            package='rover',
            executable='manual_control',
            name='rover_manual_control',
            output='screen'
        ),

        # RealSense Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rover_launch_dir, 'rs_multiple_devices.py')
            )
        ),

        # IMU to ENU Conversion
        Node(
            package='rover',
            executable='imu_to_enu.py',
            name='imu_to_enu',
            output='screen'
        ),

        # IMU Filter Madgwick
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_node',
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

        # GNSS RTK
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gnss_launch_dir, 'moving_baseline_rtk.py')
            )
        ),

        # LED Light Node
        Node(
            package='rover',
            executable='led_light.py',
            name='led_listener',
            output='screen'
        ),

        # Manual Indicator Publisher
        Node(
            package='rover',
            executable='pub_manual_indicator.py',
            name='pub_manual_indicator',
            output='screen'
        ),

        # Webcam nodes (for the arm)
        Node(
            package='rover',
            executable='webcam.py',
            name='webcam_node',
            output='screen'
        ),
        Node(
            package='rover',
            executable='webcam2.py',
            name='webcam2_node',
            output='screen'
        )

        # Uncomment these if you want them back in:
        # Node(
        #     package='rover',
        #     executable='check_zed.py',
        #     name='check_zed',
        #     output='screen'
        # ),
        # Node(
        #     package='rover',
        #     executable='final_state_machine.py',
        #     name='rover_state_machine',
        #     output='screen'
        # ),
    ])