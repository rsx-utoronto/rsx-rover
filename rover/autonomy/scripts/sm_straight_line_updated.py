#!/usr/bin/python3

import csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool, String
from rover.msg import MissionState
import math
import time
import threading
import yaml
import os
import numpy as np
import threading

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")
#file_path = "/home/rsx/rover_ws/src/rsx-rover/rover/autonomy/scripts/sm_config.yaml"

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

# File for the straight line traversal class for the state machine. The class is initialized with linear, angular velocities, and target passed
# through the state machine. When navigate function is called, it calls the move_to_target function which continuously calculates the target distance
# and heading to determine the angular and linear velocities. When the target distance is within the threshold it breaks out of the loop to return.
    
class StraightLineApproach(Node):
    def __init__(self):
        super().__init__('straight_line_approach_node')
        self.found = False
        self.abort_check = False
        self.x = 0#-100000
        self.y = 0#-100000
        self.heading = 0
        self.active = False
        self.target = None
        self.sub = self.create_subscription(String, 'chatter', self.callback, 10)
        # Use QoS depth 10 and add callback logging
        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/pose', self.pose_callback, 10)
        self.target_subscriber = self.create_subscription(
            Float64MultiArray, '/target', self.target_callback, 10)
        self.drive_publisher = self.create_publisher(Twist, '/drive', 10)
        self.abort_sub = self.create_subscription(
            Bool, "/auto_abort_check", self.abort_callback, 10)
        self.aruco_sub = self.create_subscription(
            Bool, "/aruco_found", self.detection_callback, 10)
        
        self.pub = self.create_publisher(MissionState, 'mission_state', 10)
        self.sla_pub = self.create_publisher(String, 'sla_pub', 10)
        self.lin_vel= sm_config.get("straight_line_approach_lin_vel")
        self.ang_vel = sm_config.get("straight_line_approach_ang_vel")
        self.target = None
        self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        self._nav_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._nav_event = threading.Event()
        self.nav_thread = None
        self.SAFETY_MARGIN = 0.00005  # adjust as needed
        self.GUI_avoid_check = False
    
    def callback(self, msg):
        #print("I heard: ", msg.data)
        pass

    def feedback_callback(self, msg):
        print("in SL feedback callback, msg.state:", msg.state)
        self.get_logger().info(f"in SL feedback callback, msg.state: {msg.state}")

        if msg.state == "START_SL":
            self.current_state = getattr(msg, "current_state", "Location Selection")
            print("in SL msg.current_goal", msg.current_goal)
            target_x = msg.current_goal.pose.position.x
            target_y = msg.current_goal.pose.position.y
            with self._nav_lock: #new 
                self.target = [(target_x, target_y)] #had this
            self._nav_event.set() #new
            if self.nav_thread is None or not self.nav_thread.is_alive():
                self.nav_thread = threading.Thread(target=self._nav_loop, daemon=True)
                self.nav_thread.start()

        if msg.state == "GUI_AVOID":
            self.GUI_avoid_check = True
            self.current_state = getattr(msg, "current_state", "Location Selection")
            with self._nav_lock: #new 
                self.target = self.GUI_avoid_targets()
            self._nav_event.set() #new
            if self.nav_thread is None or not self.nav_thread.is_alive():
                self.nav_thread = threading.Thread(target=self._nav_loop, daemon=True)
                self.nav_thread.start()
        else:
            self.active = False
    
    def pose_callback(self, msg):
        print("in SL pose callback")
        with self._pose_lock:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                                msg.pose.orientation.y, msg.pose.orientation.z)[2]
        # self.get_logger().info(f"x: {self.x}, y {self.y}, heading {self.heading}")
    
    def abort_callback(self,msg):
        self.abort_check = msg.data
        
    def odom_callback(self, msg):
        print("in SL odom callback")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.get_logger().info(f"x: {self.x}, y {self.y}, heading {self.heading}")

    def target_callback(self, msg):
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]

    def to_euler_angles(self, w, x, y, z):
        angles = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        angles[0] = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        angles[2] = math.atan2(siny_cosp, cosy_cosp)
        return angles
    
    def detection_callback(self, data):
        self.found = data

    def move_to_target(self, target_x, target_y, state): #navigate needs to take in a state value as well (FINISHIT)
        
        kp = 0.5
        threshold = 0.5
        angle_threshold = 0.2
        
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj,
                   "OBJ3":obj}
        
        first_time=True
        

        while (rclpy.ok()) and (self.abort_check is False):
            msg = Twist()
            if target_x is None or target_y is None or self.x is None or self.y is None:
                continue
            
            if abs(self.x-target_x) < 25 or abs(self.y-target_y) < 25:
                self.start_looking=True
                
            obj = self.mallet_found or self.waterbottle_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj,
                   "OBJ3":obj}
            print("state is mapping[state]", mapping[state], state)
            if mapping[state] is False:
                #nomral operation
                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
                # print(f"Current Position: ({self.x}, {self.y})")
                #print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                angle_diff = target_heading - self.heading
                
                # print ( f"angle_diff: {angle_diff}")

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                    
                # print (f"diff in heading: {angle_diff}", f"target_distance: {target_distance}")

                if target_distance < threshold:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.drive_publisher.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    break

                if abs(angle_diff) <= angle_threshold:
                    msg.linear.x = self.lin_vel
                    msg.angular.z = 0.0
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3
                self.drive_publisher.publish(msg)
            
            else: #if mapping[state] is True --> if the object is found
                print("mapping state is true!")
                print("IN HOMING")
                message="In Homing"
                self.message_pub.publish(message)
                # call homing
                # should publish that it is found
                # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
                # pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
                pub=self.create_publisher(Twist, 'drive', 10) # change topic name
                if state == "AR1" or state == "AR2" :
                    # this sees which camera it is using and then uses the parameters accordingly.
                    if sm_config.get("realsense_detection"):
                        aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
                    else: 
                        aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO

                    self.create_subscription(Float64MultiArray, 'aruco_node/bbox', callback=aimer.rosUpdate) # change topic name
                    #print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                elif state == "OBJ1" or state == "OBJ2":
                    aimer = aruco_homing.AimerROS(640, 360, 1450, 100, 200, sm_config.get("Obj_homing_lin_vel"), sm_config.get("Obj_homing_ang_vel")) # FOR WATER BOTTLE
                    self.create_subscription(Float64MultiArray, 'object/bbox', callback=aimer.rosUpdate)
                    #print (sm_config.get("Obj_homing_lin_vel"),sm_config.get("Obj_homing_ang_vel"))
                 #this code needs to be adjusted
                
                # Wait a bit for initial detection
                for i in range(50):
                    time.sleep(0.1)
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False
                
                while (rclpy.ok()) and (self.abort_check is False):
                    twist = Twist()
                    
                    # Check if we have valid values from the aimer
                    if aimer.linear_v is not None and aimer.angular_v is not None:
                        # We have a detection, update the timer
                        last_detection_time = time.time()
                        detection_active = True
                        
                        # Check if we've reached the target
                        if aimer.linear_v == 0 and aimer.angular_v == 0:
                            print ("at weird", aimer.linear_v, aimer.angular_v)
                            if first_time:
                                first_time=False
                                initial_time=time.time()
                            
                            # faking it til you makin it
                            while abs(initial_time-time.time()) < 0.7:
                                msg.linear.x=self.lin_vel
                                pub.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                rclpy.timer.Rate(1).sleep()
                            twist.linear.x = 0.0
                            msgg=True
                            self.done_early.publish(msgg)
                            twist.angular.z = 0.0
                            pub.publish(twist)
                            return
                            
                        # Normal homing behavior
                        if aimer.angular_v == 1:
                            twist.angular.z = float(aimer.max_angular_v)
                            print ("first if",aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.angular_v == -1:
                            twist.angular.z = float(-aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.linear_v == 1:
                            print ("second check",aimer.max_linear_v)
                            twist.linear.x = float(aimer.max_linear_v)
                            twist.angular.z = 0.0
                    else:
                        # No detection, check if we're within memory duration
                        if detection_active and time.time() - last_detection_time < detection_memory_duration:
                            print("Using last movement commands from memory")
                            # Continue with last valid movement
                            # (twist values are already set from previous iteration)
                        else:
                            # Memory expired, go back to grid search
                            print("Detection lost and memory expired, returning to grid search")
                            detection_active = False
                            break
                    
                    pub.publish(twist)
                    time.sleep(1/50)
              
                break
            
            self.drive_publisher.publish(msg)
            time.sleep(1/50)

    # def navigate(self, state="Location Selection"): # navigate needs to take in a state value as well
    #     print("self.targets", self.target)
    #     for target_x, target_y in self.target:
    #         print(f"Moving towards target: ({target_x}, {target_y})")
    #         self.move_to_target(target_x, target_y)
    #         if self.abort_check:
    #             self.abort_check = False
    #             break
    #         time.sleep(1)
    #     sla_msg = MissionState()
    #     # compute success against first target we were given
    #     tx, ty = targets[0]
    #     if (np.abs(self.x - tx) < 0.5) and (np.abs(self.y - ty) < 0.5):
    #          sla_msg.state = "SLA_DONE"
    #     else:
    #          sla_msg.state = "SLA_FAILED"
 
    #     self.active = False
    #     self.pub.publish(sla_msg)
        
    def _nav_loop(self):
        while rclpy.ok():
            self._nav_event.wait()
            if not rclpy.ok():
                break
            # copy target under lock to avoid races
            with self._nav_lock:
                targets = list(self.target) if self.target else []
            for tx, ty in targets:
                self.move_to_target(tx, ty)
                if self.abort_check:
                    break
            # publish SLA result correlated with current_state
            resp = MissionState()
            resp.current_state = getattr(self, "current_state", "")
            if self.GUI_avoid_check:
                resp.state = "GUI_AVOID_DONE" if not self.abort_check else "GUI_AVOID_FAILED"
                self.GUI_avoid_check = False
            else:
                resp.state = "SLA_DONE" if not self.abort_check else "SLA_FAILED"
            self.pub.publish(resp)
            self._nav_event.clear()

    #----------------------------------------------- GUI SL AVOIDANCE CODE--------------------------------------------------------- 

    def _dijkstra_path(self, adj, start_idx, goal_idx):
        """Simple Dijkstra shortest-path on adjacency list graph."""
        import heapq

        N = len(adj)
        dist_arr = [float("inf")] * N
        prev = [None] * N
        dist_arr[start_idx] = 0.0
        heap = [(0.0, start_idx)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist_arr[u]:
                continue
            if u == goal_idx:
                break
            for v, w in adj[u]:
                nd = d + w
                if nd < dist_arr[v]:
                    dist_arr[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if dist_arr[goal_idx] == float("inf"):
            return None

        path_idx = []
        cur = goal_idx
        while cur is not None:
            path_idx.append(cur)
            cur = prev[cur]
        path_idx.reverse()
        return path_idx

    def _segment_blocked_by_edges(self, p1, p2, edges, tol=1e-9):
        """
        True if segment p1-p2 crosses any obstacle edge in `edges`,
        ignoring exact shared endpoints.
        """
        def almost_equal(a, b, eps=tol):
            return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps

        for (c, d) in edges:
            # allow touching at shared endpoints
            if (almost_equal(p1, c) or almost_equal(p1, d) or
                    almost_equal(p2, c) or almost_equal(p2, d)):
                continue
            if self.segments_intersect(p1, p2, c, d):
                return True
        return False

    def dist(self, p1, p2):
        """Euclidean distance between 2D points (x, y)."""
        if not p1 or not p2:
            return float("inf")
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    
    def _build_all_edges_from_hulls(self, hulls):
        """
        From list of hulls, build a flat list of segment edges.
        Uses obstacle_edges() defined above.
        """
        edges = []
        for ob in hulls:
            edges.extend(self.obstacle_edges(ob["verts"]))
        return edges
    
    def plan_path_between_points(self, start, goal, hulls):
        """
        Visibility-graph style path between start and goal that avoids
        obstacle polygons given in `hulls`.

        start, goal: (x, y)
        hulls: list of {"label": ..., "oid": ..., "verts": [(x, y), ...]}

        Returns:
        list[(x, y)] including start and goal.
        """
        # Node 0 = start, node 1 = goal; others are obstacle vertices
        nodes = [start, goal]
        # meta[i] = None or (label, oid, idx_in_hull, nverts)
        meta = [None, None]

        for ob in hulls:
            verts = ob["verts"]
            n = len(verts)
            for idx, pt in enumerate(verts):
                nodes.append(pt)
                meta.append((ob["label"], ob["oid"], idx, n))

        # Pre-compute all obstacle edges for intersection checks
        all_edges = self._build_all_edges_from_hulls(hulls)

        N = len(nodes)
        adj = [[] for _ in range(N)]

        for i in range(N):
            for j in range(i + 1, N):
                p1 = nodes[i]
                p2 = nodes[j]

                if self.dist(p1, p2) == 0.0:
                    continue

                mi = meta[i]
                mj = meta[j]

                # If both points are vertices of the same obstacle,
                # only connect neighbours on that hull (avoid cutting corners).
                if mi is not None and mj is not None and mi[0] == mj[0] and mi[1] == mj[1]:
                    n = mi[3]
                    idx_i = mi[2]
                    idx_j = mj[2]
                    if not ((idx_i - idx_j) % n == 1 or (idx_j - idx_i) % n == 1):
                        # Not neighbours on the polygon perimeter
                        continue

                # If the segment is blocked by any obstacle edge, skip it
                if self._segment_blocked_by_edges(p1, p2, all_edges):
                    continue

                w = self.dist(p1, p2)
                adj[i].append((j, w))
                adj[j].append((i, w))

        path_idx = self._dijkstra_path(adj, 0, 1)

        if not path_idx:
            # Fallback: if direct line is free, just use it
            if not self._segment_blocked_by_edges(start, goal, all_edges):
                return [start, goal]
            # Last-resort fallback: still return direct (better than nothing)
            return [start, goal]

        return [nodes[k] for k in path_idx]
    
    def inflate_hull(self, hull, margin):
        """
        Inflate a convex hull outward by `margin`.

        hull: list[(x, y)]
        margin: float, same units as x/y.
        """
        if not hull or margin <= 0.0:
            return hull

        cx = sum(x for (x, _y) in hull) / len(hull)
        cy = sum(y for (_x, y) in hull) / len(hull)

        inflated = []
        for (x, y) in hull:
            vx = x - cx
            vy = y - cy
            norm = math.hypot(vx, vy)

            if norm < 1e-12:
                # Vertex at centroid; push in +x direction
                inflated.append((x + margin, y))
            else:
                scale = (norm + margin) / norm
                inflated.append((cx + vx * scale, cy + vy * scale))

        return inflated


    def build_obstacle_hulls(self, all_obstacles):
        """
        Turn the obstacle dict from load_obstacles_from_csv into convex hulls.

        all_obstacles: dict[(label, obstacle_id)] -> list[(x, y)]

        Returns:
        list of {"label": label, "oid": oid, "verts": [(x, y), ...]}
        """
        hulls = []

        for (label, oid), verts in all_obstacles.items():
            # unique + sorted
            pts = sorted(set(verts))
            if len(pts) < 2:
                continue

            if len(pts) == 2:
                hull = pts[:]  # just a segment
            else:
                # Monotone chain convex hull (same idea as in map_viewer_2)
                def cross(o, a, b):
                    return ((a[0] - o[0]) * (b[1] - o[1])
                            - (a[1] - o[1]) * (b[0] - o[0]))

                lower = []
                for p in pts:
                    while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                        lower.pop()
                    lower.append(p)

                upper = []
                for p in reversed(pts):
                    while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                        upper.pop()
                    upper.append(p)

                hull = lower[:-1] + upper[:-1]

            # --- NEW: inflate this hull by the safety margin ---
            hull = self.inflate_hull(hull, self.SAFETY_MARGIN)

            hulls.append({"label": label, "oid": oid, "verts": hull})

        return hulls
    
    def _on_segment(self, p, q, r):
        """
        Given colinear p, q, r, check if point q lies on segment pr.
        """
        return (
            min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
            and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
        )
    
    def _orientation(self, p, q, r):
        """
        Helper for segment intersection:
        returns 0 -> colinear, 1 -> clockwise, 2 -> counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-9:
            return 0
        return 1 if val > 0 else 2
    
    def segments_intersect(self, p1, p2, q1, q2):
        """
        Standard 2D line segment intersection test for p1-p2 vs q1-q2.
        Points are (x, y) tuples. Here x=lat, y=lng (same as your CSVs).
        """
        o1 = self._orientation(p1, p2, q1)
        o2 = self._orientation(p1, p2, q2)
        o3 = self._orientation(q1, q2, p1)
        o4 = self._orientation(q1, q2, p2)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special colinear cases
        if o1 == 0 and self._on_segment(p1, q1, p2):
            return True
        if o2 == 0 and self._on_segment(p1, q2, p2):
            return True
        if o3 == 0 and self._on_segment(q1, p1, q2):
            return True
        if o4 == 0 and self._on_segment(q1, p2, q2):
            return True

        return False
    
    def obstacle_edges(self, vertices):
        """
        Given a list of vertices [(lat, lng), ...] for an obstacle,
        return a list of edges as pairs of points.

        If there are only 2 points, it's treated as an open segment.
        If there are >=3 points, we close the polygon (last->first).
        """
        edges = []
        n = len(vertices)
        if n < 2:
            return edges

        # Edge between successive vertices
        for i in range(n - 1):
            edges.append((vertices[i], vertices[i + 1]))

        # Close polygon for 3 or more vertices
        if n >= 3:
            edges.append((vertices[-1], vertices[0]))

        return edges
    
    def find_blocking_obstacles(self, start_pt, end_pt, all_obstacles):
        """
        Given:
        start_pt: (lat, lng)
        end_pt:   (lat, lng)
        all_obstacles: dict[(label, obstacle_id)] -> list[(lat, lng)]

        Returns:
        list[(label, obstacle_id)] that intersect the straight line
        from start_pt to end_pt.
        """
        blocking = []

        for key, vertices in all_obstacles.items():
            if len(vertices) < 2:
                continue

            for a, b in self.obstacle_edges(vertices):
                if self.segments_intersect(start_pt, end_pt, a, b):
                    blocking.append(key)
                    break  # no need to check more edges for this obstacle

        return blocking
    
    def load_obstacles_from_csv(self, file_path: str, label: str):
        """
        Load obstacles from a CSV exported by map_viewer_2.
        CSV format (per row):
        id,timestamp,lat,lng,label,obstacle_id

        Returns:
        dict[(label, obstacle_id)] -> list[(lat, lng)]
        """
        obstacles = {}
        if not file_path:
            return obstacles

        try:
            with open(file_path, newline="") as f:
                reader = csv.DictReader(f)
                tag = Path(file_path).stem  # <- add this
                for row in reader:
                    try:
                        obstacle_id = row["obstacle_id"]
                        lat = float(row["lat"])
                        lng = float(row["lng"])
                    except (KeyError, ValueError):
                        continue

                    key = (label, f"{tag}:{obstacle_id}")  # <- change this
                    obstacles.setdefault(key, []).append((lat, lng))
        except FileNotFoundError:
            print(f"[WARN] Obstacle CSV not found: {file_path}")

        return obstacles


    def GUI_avoid_targets(self):
        # 1) Load start and end x/y from CSV in this folder
        start_end_csv = os.path.join(os.path.dirname(__file__), "sm_straight_line_start_end.csv")

        with open(start_end_csv, newline="") as f:
            reader = csv.DictReader(f)
            row = next(reader)

        start_x = float(row["start_x"])
        start_y = float(row["start_y"])
        target_x = float(row["target_x"])
        target_y = float(row["target_y"])

        print(f"Start position from CSV:  ({start_x}, {start_y})")
        print(f"Target position from CSV: ({target_x}, {target_y})")

        start_pt = (start_x, start_y)
        end_pt = (target_x, target_y)

        # 1b) Write a separate CSV in the GUI folder for the map to use when
        #     building the path. This is *not* places_to_go.csv.
        BASE_GUI_DIR = os.path.expanduser("~/rover_ws/src/rsx-rover/scripts/GUI")
        GUI_START_END_CSV = os.path.join(BASE_GUI_DIR, "sm_start_end_for_gui.csv")

        try:
            with open(GUI_START_END_CSV, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=["Point Name", "Latitude", "Longitude"]
                )
                writer.writeheader()
                writer.writerow(
                    {"Point Name": "Start", "Latitude": start_x, "Longitude": start_y}
                )
                writer.writerow(
                    {"Point Name": "End", "Latitude": target_x, "Longitude": target_y}
                )
            print(f"Wrote GUI start/end CSV to: {GUI_START_END_CSV}")
        except OSError as e:
            print(f"[WARN] Failed to write GUI start/end CSV: {e}")

        # 2) Find the GUI folder (robust: relative to repo)
        BASE_GUI_DIR = Path(__file__).resolve().parents[3] / "scripts" / "GUI"
        if not BASE_GUI_DIR.exists():
            # fallback to old behavior if repo layout changes
            BASE_GUI_DIR = Path(os.path.expanduser("~/rover_ws/src/rsx-rover/scripts/GUI"))

        # 3) Load ALL obstacle CSVs that match the naming patterns
        perm_csvs = sorted(BASE_GUI_DIR.glob("*permanent_obstacles*.csv"))
        temp_csvs = sorted(BASE_GUI_DIR.glob("*temporary_obstacles*.csv"))

        if not perm_csvs:
            print(f"[WARN] No permanent obstacle CSVs found in: {BASE_GUI_DIR}")
        if not temp_csvs:
            print(f"[WARN] No temporary obstacle CSVs found in: {BASE_GUI_DIR}")

        perm_obstacles = {}
        for fp in perm_csvs:
            perm_obstacles.update(self.load_obstacles_from_csv(str(fp), "permanent"))

        temp_obstacles = {}
        for fp in temp_csvs:
            temp_obstacles.update(self.load_obstacles_from_csv(str(fp), "temporary"))

            all_obstacles = {}
            all_obstacles.update(perm_obstacles)
            all_obstacles.update(temp_obstacles)

            # Print which obstacles block the direct start→end line
            blocking = self.find_blocking_obstacles(start_pt, end_pt, all_obstacles)
            if not blocking:
                print("Direct path from start to end is NOT blocked by any obstacles in the CSVs.")
            else:
                print("Direct path from start to end is BLOCKED by these obstacles:")
                for label, obstacle_id in blocking:
                    print(f"  - {label} obstacle with obstacle_id={obstacle_id}")

            # 4) Build obstacle hulls and compute a waypoint path around them
            hulls = self.build_obstacle_hulls(all_obstacles)
            path_points = self.plan_path_between_points(start_pt, end_pt, hulls)

            print("\nPlanned path waypoints (start -> end):")
            for i, (px, py) in enumerate(path_points):
                print(f"  {i}: ({px}, {py})")

            # 5) Use all intermediate waypoints (excluding the first start point)
            #    as the list of targets for the rover. navigate() already loops
            #    through self.targets in order.
            targets = [(x, y) for (x, y) in path_points[1:]]
            return targets

        

def main():
    rclpy.init(args=None)
    sla = StraightLineApproach()
    try: 
        print("spinning sla node")
        rclpy.spin(sla)
    finally: 
        sla.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()