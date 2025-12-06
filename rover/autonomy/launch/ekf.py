from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher with remap
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gps_broadcaster_ekf",
            output="screen",
            arguments=["-0.2", "0", "0.6", "0", "0", "0", "1", "base_link", "gps", "100"],
            remappings=[("tf", "tf_ekf")]
        ),

        # Static transform publisher without remap
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gps_broadcaster",
            output="screen",
            arguments=["-0.2", "0", "0.6", "0", "0", "0", "1", "base_link", "gps", "100"]
        ),

        # imu_to_enu node
        Node(
            package="rover",
            executable="imu_to_enu.py",
            name="imu_to_enu",
            output="screen"
        ),

        # imu_filter_madgwick node
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[
                {"publish_tf": False},
                {"use_mag": False},
                {"world_frame": "enu"}
            ],
            remappings=[
                ("imu/mag", "/zed_node/imu/mag"),
                ("imu/data_raw", "/imu/enu"),
                ("imu/data", "/imu/orient")
            ]
        ),

        # robot_localization ekf_localization_node
        Node(
            package="robot_localization",
            executable="ekf_localization_node",
            name="ekf_odom_node",
            output="screen",
            parameters=[
                {"frequency": 30.0},
                {"sensor_timeout": 0.2},
                {"two_d_mode": False},
                {"print_diagnostics": True},
                {"publish_tf": True},
                #{"map_frame": "map"},  # commented out param in original
                {"odom_frame": "odom"},
                {"base_link_frame": "base_link"},
                {"world_frame": "odom"},
                {"imu0": "/imu/orient"},
                {"imu0_differential": False},
                {"imu0_relative": False},
                {"imu0_queue_size": 10},
                {"imu0_remove_gravitational_acceleration": True},
                {"imu0_config": [False, False, False, False, False, False, False, False, False,
                                 True, True, True, True, True, True]},
                {"odom0": "/zed_node/odom"},
                {"odom0_differential": False},
                {"odom0_relative": False},
                {"odom0_queue_size": 10},
                {"odom0_config": [True, True, True, False, False, False, False, False, False,
                                  False, False, False, False, False, False]},
                {"twist0": "ublox/fix_velocity"},
                {"twist0_differential": False},
                {"twist0_relative": False},
                {"twist0_queue_size": 10},
                {"twist0_config": [False, False, False, False, False, False, True, True, True,
                                  False, False, False, False, False, False]},
            ],
            remappings=[
                ("odometry/filtered", "/odom/ekf")
            ]
        ),
    ])