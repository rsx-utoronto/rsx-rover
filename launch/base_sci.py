from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # --- Optional: run shell command like `setip; setmaster` ---
        # Uncomment and adapt if you want to run shell commands
        # ExecuteProcess(
        #     cmd=['bash', '-c', 'setip; setmaster'],
        #     shell=True
        # ),

        # --- Launch GUI node ---
        Node(
            package='rover',
            executable='science_gui.py',
            name='rover_gui',
            output='screen'
        ),

        # --- Include joy.launch.py ---
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