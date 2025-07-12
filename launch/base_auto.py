import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rover',
            executable='gui_pyqt.py',
            name='rover_gui',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rover'), 'launch/joy.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()