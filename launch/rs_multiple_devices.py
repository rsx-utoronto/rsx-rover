from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Declare arguments
    serial_no_camera1 = LaunchConfiguration('serial_no_camera1')
    serial_no_camera2 = LaunchConfiguration('serial_no_camera2')
    serial_no_camera3 = LaunchConfiguration('serial_no_camera3')

    camera1 = LaunchConfiguration('camera1')
    camera2 = LaunchConfiguration('camera2')
    camera3 = LaunchConfiguration('camera3')

    tf_prefix_camera1 = LaunchConfiguration('tf_prefix_camera1')
    tf_prefix_camera2 = LaunchConfiguration('tf_prefix_camera2')
    tf_prefix_camera3 = LaunchConfiguration('tf_prefix_camera3')

    initial_reset = LaunchConfiguration('initial_reset')
    reconnect_timeout = LaunchConfiguration('reconnect_timeout')

    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch'
    )

    return LaunchDescription([

        # Declare all the launch arguments
        DeclareLaunchArgument('serial_no_camera1', default_value='128422271941'),
        DeclareLaunchArgument('serial_no_camera2', default_value='833612070129'),
        DeclareLaunchArgument('serial_no_camera3', default_value=''),

        DeclareLaunchArgument('camera1', default_value='camera1'),
        DeclareLaunchArgument('camera2', default_value='camera2'),
        DeclareLaunchArgument('camera3', default_value='camera3'),

        DeclareLaunchArgument('tf_prefix_camera1', default_value=LaunchConfiguration('camera1')),
        DeclareLaunchArgument('tf_prefix_camera2', default_value=LaunchConfiguration('camera2')),
        DeclareLaunchArgument('tf_prefix_camera3', default_value=LaunchConfiguration('camera3')),

        DeclareLaunchArgument('initial_reset', default_value='false'),
        DeclareLaunchArgument('reconnect_timeout', default_value='6.0'),

        # Camera 1 group
        PushRosNamespace(camera1),
            GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_launch_dir, 'rs_launch.py')
                ),
                launch_arguments={
                    'serial_no': serial_no_camera1,
                    'tf_prefix': tf_prefix_camera1,
                    'initial_reset': initial_reset,
                    'reconnect_timeout': reconnect_timeout,
                    'depth_width': '640',
                    'depth_height': '480',
                    'depth_fps': '15',
                    'color_width': '640',
                    'color_height': '480',
                    'color_fps': '15',
                    'infra_width': '640',
                    'infra_height': '480',
                    'infra_fps': '15',
                    'infra_rgb': 'false',
                    'enable_sync': 'true',
                    'align_depth': 'true',
                    'enable_accel': 'true',
                    'enable_gyro': 'true',
                    'enable_pointcloud': 'true',
                    'unite_imu_method': 'linear_interpolation'
                }.items()
            )
        ]),

        # Camera 2 group
        PushRosNamespace(camera2),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_launch_dir, 'rs_launch.py')
                ),
                launch_arguments={
                    'serial_no': serial_no_camera2,
                    'tf_prefix': tf_prefix_camera2,
                    'initial_reset': initial_reset,
                    'reconnect_timeout': reconnect_timeout,
                    'depth_width': '640',
                    'depth_height': '480',
                    'depth_fps': '15',
                    'color_width': '640',
                    'color_height': '480',
                    'color_fps': '15',
                    'infra_width': '640',
                    'infra_height': '480',
                    'infra_fps': '15',
                    'infra_rgb': 'false',
                    'enable_sync': 'true',
                    'align_depth': 'true',
                    'enable_accel': 'true',
                    'enable_gyro': 'true',
                    'enable_pointcloud': 'true',
                    'unite_imu_method': 'linear_interpolation'
                }.items()
            )
        ]),

        # If you ever decide to love camera3, you can add this:
        # GroupAction([
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(realsense_launch_dir, 'rs_camera.launch.py')
        #         ),
        #         launch_arguments={
        #             'serial_no': serial_no_camera3,
        #             'tf_prefix': tf_prefix_camera3,
        #             'initial_reset': initial_reset,
        #             'reconnect_timeout': reconnect_timeout,
        #             ...
        #         }.items()
        #     )
        # ], namespace=camera3)
    ])