from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover',
            executable='state_machine.py',
            name='rover_state_machine',
            output='screen'
        ), 
        
        
         Node(
            package='rover',
            executable='sm_straight_line.py',
            name='straight_line',
            output='screen'
        ), 
        
        Node(
            package='rover',
            executable='sm_grid_search.py',
            name='grid_search',
            output='screen'
        ),
        
        Node(
            package='rover',
            executable='aruco_detection_node.py',
            name='aruco_detector',
            output='screen'
        )
        
        # Node(
        #     package='rover',
        #     executable='aruco_homing.py',
        #     name='aruco_homing',
        #     output='screen'
        # )
        
        
        #add asar obstacle avoidance node here later
        
        
    ])