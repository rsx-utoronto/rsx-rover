from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)



def generate_launch_description():
    # Inline URDF for LiDAR
    urdf = """
    <?xml version="1.0"?>
    <robot name="rover">
        <link name="base_link"/>
        <link name="os1_sensor"/>
        <joint name="base_to_lidar" type="fixed">
            <parent link="base_link"/>
            <child link="os1_sensor"/>
            <origin xyz="0 0 0.5" rpy="0 0 0"/> <!-- 50cm above base -->
        </joint>
    </robot>
"""
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("rover"), "rover","autonomy","config","groundsep_ouster_viz.rviz"]
            ),
            # Path(get_package_share_directory('rover'))/'rover'/ 'autonomy'/ 'config' / 'groundsep_ouster_viz.rviz'
        ],
        # condition=IfCondition(visualize),
    )

    return LaunchDescription([
        Node(
            package='rover',
            executable='visualize_groundsep.py',  # Renamed from 'manual_control'
            name='visualize_groundsep',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ),
        rviz_node
    ])