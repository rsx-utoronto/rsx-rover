#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class Visualizer:
    def __init__(self, nh, frame_id):
        self.frame_id = frame_id
        self.marker_array_pub = nh.advertise('/dwa_trajectories', MarkerArray, queue_size=1)
        self.line_marker_pub = nh.advertise('/dwa_trajectory', Marker, queue_size=1)
        self.footprint_pub = nh.advertise('/robot_footprint', Marker, queue_size=1)
        self.grid_pub = nh.advertise('/robot_grid', Marker, queue_size=1)

        # Subscribe to the A* waypoints topic
        self.astar_sub = nh.subscribe('/astar_waypoints', Float32MultiArray, self.astar_callback)
        self.waypoints = []  # Store waypoints received from the A* algorithm

    def astar_callback(self, msg):
        """
        Callback function to process waypoints published by the A* algorithm.
        """
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        rospy.loginfo(f"Received waypoints from A*: {self.waypoints}")
        self.publish_waypoints(self.waypoints)

    def publish_trajectories(self, trajectories):
        marker_array = MarkerArray()
        marker_id = 0

        for traj in trajectories:
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "dwa_trajectories"
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = 0.02
            line_marker.color.r = 1.0
            line_marker.color.g = 0.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            line_marker.pose.orientation.x = 0.0
            line_marker.pose.orientation.y = 0.0
            line_marker.pose.orientation.z = 0.0
            line_marker.pose.orientation.w = 1.0

            for pose in traj:
                p = Point()
                p.x = pose.x
                p.y = pose.y
                p.z = 0.0
                line_marker.points.append(p)

            marker_array.markers.append(line_marker)
            marker_id += 1

        self.marker_array_pub.publish(marker_array)

    def publish_waypoints(self, waypoints):
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = self.frame_id
        waypoint_marker.header.stamp = rospy.Time.now()
        waypoint_marker.ns = "dwa_waypoints"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.POINTS
        waypoint_marker.action = Marker.ADD

        waypoint_marker.scale.x = 0.1
        waypoint_marker.scale.y = 0.1
        waypoint_marker.color.r = 0.0
        waypoint_marker.color.g = 1.0
        waypoint_marker.color.b = 1.0
        waypoint_marker.color.a = 1.0

        for waypoint in waypoints:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.0
            waypoint_marker.points.append(p)

        self.line_marker_pub.publish(waypoint_marker)

    def navigate_waypoints(self, robot_position, threshold=0.2):
        for waypoint in self.waypoints:
            while self.distance_to_goal(robot_position, waypoint) > threshold:
                rospy.loginfo(f"Navigating to waypoint {waypoint}")
                self.publish_trajectory([robot_position, waypoint])

                # Simulate reaching the waypoint
                robot_position[0] += 0.1 * (waypoint[0] - robot_position[0])
                robot_position[1] += 0.1 * (waypoint[1] - robot_position[1])
                rospy.sleep(0.1)

            rospy.loginfo(f"Reached waypoint {waypoint}")

    @staticmethod
    def distance_to_goal(current_position, goal_position):
        dx = current_position[0] - goal_position[0]
        dy = current_position[1] - goal_position[1]
        return (dx ** 2 + dy ** 2) ** 0.5

# Example usage
if __name__ == "__main__":
    rospy.init_node("trajectory_visualizer")
    nh = rospy

    visualizer = Visualizer(nh, "map")
    rospy.loginfo("Waiting for A* waypoints...")
    
    # Simulate robot navigation (update robot_position as per localization)
    robot_position = [0.0, 0.0]  # Starting position
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if visualizer.waypoints:
            visualizer.navigate_waypoints(robot_position)
        rate.sleep()
