from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# from launch.actions import ExecuteProcess  # Uncomment if you want to run shell commands

def generate_launch_description():
    return LaunchDescription([
        # Optional: run `setip; setmaster` if needed
        # ExecuteProcess(
        #     cmd=['bash', '-c', 'setip; setmaster'],
        #     shell=True
        # ),

        # Launch the GUI node
        Node(
            package='rover',
            executable='arm_task_gui.py',
            name='rover_gui',
            output='screen'
        ),

        # Include the joy launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rover'),
                    'launch',
                    'joy.py'
                ])
            )
        )
    ])