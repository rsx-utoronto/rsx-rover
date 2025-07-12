import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'realsense2_camera'), 'launch/rs_camera.launch.py')
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
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()