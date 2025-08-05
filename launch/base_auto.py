from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# from launch.actions import ExecuteProcess  # Uncomment if using setip/setmaster

def generate_launch_description():
    return LaunchDescription([
        # --- Optional: Execute setip; setmaster (if needed) ---
        # ExecuteProcess(
        #     cmd=['bash', '-c', 'setip; setmaster'],
        #     shell=True
        # ),

        # --- Launch the GUI Node ---
        Node(
            package='rover',
            executable='gui_pyqt.py',
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