from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# from launch.actions import ExecuteProcess  # Uncomment if using setip/setmaster

def generate_launch_description():
    return LaunchDescription([
        # --- Launch the GUI Node ---
        Node(
            package='rover',
            executable='science_gui.py',
            name='science_gui',
            output='screen'
        )
    ])