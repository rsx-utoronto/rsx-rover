#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String
<<<<<<< HEAD
from nav_msgs.msg import Path  # Assuming DWA publishes trajectories as Path messages
=======
from nav_msgs.msg import Path
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd

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
<<<<<<< HEAD
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
=======
        self.trajectories_sub = rospy.Subscriber('/dwa_trajectories', Path, self.trajectories_callback)
        self.waypoints = []

    def trajectories_callback(self, msg):
        trajectories = []
        traj = []
        for pose in msg.poses:
            traj.append(pose.pose.position)
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd
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
<<<<<<< HEAD
            line_marker.scale.x = 0.02  # Line width
=======
            line_marker.scale.x = 0.02
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
<<<<<<< HEAD
            line_marker.lifetime = rospy.Duration(0.1)  # Adjust lifetime as needed
=======
            line_marker.lifetime = rospy.Duration(0.1)
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd

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
<<<<<<< HEAD
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

=======
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
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd
        for wp in waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = 0.0
<<<<<<< HEAD
            waypoint_marker.points.append(p)

        self.line_marker_pub.publish(waypoint_marker)
=======
            points_marker.points.append(p)
            line_marker.points.append(p)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)

        self.astar_marker_pub.publish(marker_array)
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd

if __name__ == "__main__":
    rospy.init_node("trajectory_visualizer")
    visualizer = Visualizer(rospy, "map")
<<<<<<< HEAD
    rospy.spin()  # Use spin to handle callbacks
=======
    rospy.spin()
>>>>>>> 934e5a2b8085249d2f837966ecd0fc5458b5cafd
