#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Path

class Visualizer:
    def __init__(self, nh, frame_id):
        self.pub = rospy.Publisher("waypoints", String, queue_size=10)
        self.frame_id = frame_id
        self.marker_array_pub = rospy.Publisher('/dwa_trajectories', MarkerArray, queue_size=1)
        self.astar_marker_pub = rospy.Publisher('/astar_waypoints_markers', MarkerArray, queue_size=1)  # New publisher for A* markers
        self.footprint_pub = rospy.Publisher('/robot_footprint', Marker, queue_size=1)
        self.grid_pub = rospy.Publisher('/robot_grid', Marker, queue_size=1)

        # Subscribers
        self.astar_sub = rospy.Subscriber('/astar_waypoints', Float32MultiArray, self.astar_callback)
        self.trajectories_sub = rospy.Subscriber('/dwa_trajectories', MarkerArray, self.trajectories_callback)
        self.waypoints = []

    def trajectories_callback(self, msg):
        trajectories = []
        traj = []
        for pose in msg.poses:
            traj.append(pose.pose.position)
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
            line_marker.scale.x = 0.02
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.lifetime = rospy.Duration(0.1)

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
        marker_array = MarkerArray()

        # Points marker for waypoints (larger size)
        points_marker = Marker()
        points_marker.header.frame_id = self.frame_id
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = "astar_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.2  # Increased size
        points_marker.scale.y = 0.2
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.lifetime = rospy.Duration(0)

        # Line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "astar_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # Line width
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        line_marker.lifetime = rospy.Duration(0)

        # Populate both markers with waypoints
        for wp in waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = 0.0
            points_marker.points.append(p)
            line_marker.points.append(p)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)

        self.astar_marker_pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node("trajectory_visualizer")
    visualizer = Visualizer(rospy, "map")
    rospy.spin()