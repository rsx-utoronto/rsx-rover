import os
import sys

import launch
import launch_ros.actions


# NOT COMPLETE: Need to correct the arguments I think 

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rosbag',
            executable='record',
            name='rosbag_record'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()