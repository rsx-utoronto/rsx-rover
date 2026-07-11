from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_launch_dir, 'rs_launch.py')
            ),
            launch_arguments={
                'enable_infra': 'true',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
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
        ),

        # Optional RViz node — still commented, like your dreams
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz_camera',
        #     arguments=['-d', os.path.join(
        #         get_package_share_directory('rover'),
        #         'rover',
        #         'autonomy',
        #         'config',
        #         'camera_config.rviz'
        #     )],
        #     output='screen'
        # )
    ])