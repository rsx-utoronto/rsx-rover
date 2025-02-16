#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Path  # Assuming DWA publishes trajectories as Path messages

class Visualizer:
    def __init__(self, nh, frame_id):
        self.pub = rospy.Publisher("waypoints", String, queue_size=10)
        self.frame_id = frame_id
        self.marker_array_pub = rospy.Publisher('/dwa_trajectories', MarkerArray, queue_size=1)
        self.line_marker_pub = rospy.Publisher('/dwa_trajectory', Marker, queue_size=1)
        self.footprint_pub = rospy.Publisher('/robot_footprint', Marker, queue_size=1)
        self.grid_pub = rospy.Publisher('/robot_grid', Marker, queue_size=1)

        # Subscribers
        self.astar_sub = rospy.Subscriber('/astar_waypoints', Float32MultiArray, self.astar_callback)
        self.trajectories_sub = rospy.Subscriber('/dwa_trajectories', Path, self.trajectories_callback)  # Adjust topic and msg type as needed
        self.waypoints = []

    def trajectories_callback(self, msg):
        """Callback for DWA trajectories (assuming Path message)."""
        trajectories = []
        # Extract trajectories from msg. Each trajectory is a Path (list of poses)
        # Example conversion: Convert Path to a list of points
        traj = []
        for pose in msg.poses:
            traj.append(pose.pose.position)  # Assuming each pose is a Point
        trajectories.append(traj)
        self.publish_trajectories(trajectories)

    def astar_callback(self, msg):
        self.waypoints = [(msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)]
        rospy.loginfo(f"Received A* waypoints: {self.waypoints}")
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
            line_marker.scale.x = 0.02  # Line width
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.lifetime = rospy.Duration(0.1)  # Adjust lifetime as needed

            for point in traj:
                p = Point()
                p.x = point.x
                p.y = point.y
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
        waypoint_marker.color.r = 1.0
        waypoint_marker.color.g = 0.0
        waypoint_marker.color.b = 0.0
        waypoint_marker.color.a = 1.0
        waypoint_marker.lifetime = rospy.Duration(0)

        for wp in waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = 0.0
            waypoint_marker.points.append(p)

        self.line_marker_pub.publish(waypoint_marker)

if __name__ == "__main__":
    rospy.init_node("trajectory_visualizer")
    visualizer = Visualizer(rospy, "map")
    rospy.spin()  # Use spin to handle callbacks