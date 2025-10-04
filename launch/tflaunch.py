from launch import LaunchDescription
from launch_ros.actions import Node

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

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ),
    ])