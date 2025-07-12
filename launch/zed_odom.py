import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='zed_broadcaster'
        ),
        launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='collapse_pointcloud',
            output='screen',
            parameters=[
                {
                    'max_height': '1'
                },
                {
                    'min_height': '-0.5'
                },
                {
                    'range_min': '0.75'
                },
                {
                    'range_max': '5.0'
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'zed_wrapper'), 'launch/zed_no_tf.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()