from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetParameter


def generate_launch_description():
    return LaunchDescription([
        # Set use_sim_time parameter globally
        SetParameter(name='use_sim_time', value=True),

        # Static transform from base_link to gps
        Node(
            package='tf',
            executable='static_transform_publisher',
            name='gps_broadcaster_base_to_gps',
            arguments=['-0.2', '-0.355', '0.18', '0', '0', '0', '1', 'base_link', 'gps', '100']
        ),

        # imu_to_enu node
        Node(
            package='rover',
            executable='imu_to_enu.py',
            name='imu_to_enu',
            output='screen'
        ),

        # gps_to_pose node
        Node(
            package='rover',
            executable='gps_to_pose.py',
            name='gps_to_pose',
            output='screen'
        ),

        # imu_filter_madgwick node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[
                {'publish_tf': False},
                {'use_mag': True},
                {'world_frame': 'enu'},
                {'orientation_stddev': 0.2}
            ],
            remappings=[
                ('imu/mag', '/zed_node/imu/mag'),
                ('imu/data_raw', '/imu/enu'),
                ('imu/data', '/imu/orient')
            ]
        ),

        # heading_filter node
        Node(
            package='rover',
            executable='heading_filter.py',
            name='heading_filter',
            output='screen',
            parameters=[
                {'magnetic_declination_radians': -0.177764053},
                {'armlength': 1.2}
            ]
        ),

        # ekf_localization_node
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_local_node',
            output='screen',
            parameters=[
                {'frequency': 30.0},
                {'sensor_timeout': 0.2},
                {'two_d_mode': False},
                {'print_diagnostics': True},
                {'publish_tf': True},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'odom'},
                {'imu0': '/imu/orient'},
                {'imu0_differential': False},
                {'imu0_relative': False},
                {'imu0_queue_size': 10},
                {'imu0_remove_gravitational_acceleration': True},
                {'imu0_config': [
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    True, True, True
                ]},
                {'odom0': '/rtabmap/odom'},
                {'odom0_differential': False},
                {'odom0_relative': False},
                {'odom0_queue_size': 10},
                {'odom0_config': [
                    True, True, True,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                    False, False, False
                ]},
            ]
        )
    ])