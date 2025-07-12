import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rover',
            executable='final_state_machine.py',
            name='rover_state_machine',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()