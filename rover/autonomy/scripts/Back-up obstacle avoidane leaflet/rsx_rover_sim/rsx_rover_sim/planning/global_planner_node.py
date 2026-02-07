#!/usr/bin/env python3
"""
Global planner node (ROS2) using the same core path planning approach as sm_straight_line_GUI_avoid.py:
- Build convex hulls for each obstacle
- Inflate hulls by a safety margin (clearance_m)
- Build a visibility graph:
    start, goal, and obstacle vertices are nodes
    connect nodes if the segment does NOT intersect any inflated obstacle edges
    if both nodes belong to same obstacle hull, only connect if they are neighbors on that hull perimeter
- Run Dijkstra to get a shortest path

Topics:
  Subscribes:
    /mission/start              std_msgs/String JSON: {"name": "...", "lat": ..., "lon": ...}
    /mission/goal               std_msgs/String JSON: {"name": "...", "lat": ..., "lon": ...}
    /world/permanent_obstacles  std_msgs/String JSON: {"obstacles":[{"id":"..","points":[{"lat":..,"lon":..},...]}, ...]}

  Publishes:
    /planner/global_path        std_msgs/String JSON:
      {"path":[{"lat":..,"lon":..},...], "start":{...}, "goal":{...}, "valid": true/false}
"""

import json
import math
import heapq
from typing import Dict, Any, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ------------------- Lat/Lon <-> Local XY (meters) -------------------

def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Local equirectangular projection (meters). Good for small areas like your map bounds."""
    R = 6371000.0
    x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * R
    y = math.radians(lat - lat0) * R
    return x, y


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    R = 6371000.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


# ------------------- Geometry helpers (mostly mirrored from sm_straight_line) -------------------

def dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def cross(o: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Monotone chain convex hull.
    Returns hull in CCW order (no duplicated endpoint).
    """
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts[:]

    lower: List[Tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: List[Tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def inflate_hull(hull: List[Tuple[float, float]], margin_m: float) -> List[Tuple[float, float]]:
    """
    Inflate a convex hull outward by margin_m (meters), by pushing vertices away from centroid.
    This mirrors inflate_hull() in sm_straight_line_GUI_avoid.py but in XY meters. :contentReference[oaicite:1]{index=1}
    """
    if not hull or margin_m <= 0.0:
        return hull

    cx = sum(x for (x, _y) in hull) / len(hull)
    cy = sum(y for (_x, y) in hull) / len(hull)

    inflated: List[Tuple[float, float]] = []
    for (x, y) in hull:
        vx = x - cx
        vy = y - cy
        n = math.hypot(vx, vy)

        if n < 1e-12:
            inflated.append((x + margin_m, y))
        else:
            scale = (n + margin_m) / n
            inflated.append((cx + vx * scale, cy + vy * scale))

    return inflated


def _orientation(p, q, r) -> int:
    """
    0 colinear, 1 clockwise, 2 counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) < 1e-12:
        return 0
    return 1 if val > 0 else 2


def _on_segment(p, q, r) -> bool:
    return (
        min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12
        and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12
    )


def segments_intersect(p1, p2, q1, q2) -> bool:
    """
    Segment intersection test.
    """
    o1 = _orientation(p1, p2, q1)
    o2 = _orientation(p1, p2, q2)
    o3 = _orientation(q1, q2, p1)
    o4 = _orientation(q1, q2, p2)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and _on_segment(p1, q1, p2):
        return True
    if o2 == 0 and _on_segment(p1, q2, p2):
        return True
    if o3 == 0 and _on_segment(q1, p1, q2):
        return True
    if o4 == 0 and _on_segment(q1, p2, q2):
        return True

    return False


def obstacle_edges(vertices: List[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    If >=3, treat as closed polygon (last->first).
    If exactly 2, treat as open segment.
    """
    edges: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    n = len(vertices)
    if n < 2:
        return edges

    for i in range(n - 1):
        edges.append((vertices[i], vertices[i + 1]))

    if n >= 3:
        edges.append((vertices[-1], vertices[0]))

    return edges


def _build_all_edges_from_hulls(hulls: List[Dict[str, Any]]):
    edges = []
    for ob in hulls:
        edges.extend(obstacle_edges(ob["verts"]))
    return edges


def _segment_blocked_by_edges(p1, p2, edges, tol=1e-10) -> bool:
    """
    True if p1->p2 intersects any obstacle edge, ignoring exact shared endpoints.
    Mirrors _segment_blocked_by_edges() in sm_straight_line_GUI_avoid.py. :contentReference[oaicite:2]{index=2}
    """
    def almost_equal(a, b, eps=tol):
        return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps

    for (c, d) in edges:
        if (almost_equal(p1, c) or almost_equal(p1, d) or
                almost_equal(p2, c) or almost_equal(p2, d)):
            continue
        if segments_intersect(p1, p2, c, d):
            return True
    return False


# Maximum obstacle height (in cm) that the rover can traverse over
MAX_TRAVERSABLE_HEIGHT_CM = 30.0

# Cost multiplier for traversing over low obstacles (per cm of height)
TRAVERSAL_COST_PER_CM = 0.5  # meters of cost per cm of obstacle height


def _heuristic(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Euclidean distance heuristic for A*."""
    return dist(p1, p2)


def _astar_path(adj, nodes, start_idx, goal_idx):
    """
    A* pathfinding algorithm.
    adj[i] contains tuples of (neighbor_idx, edge_cost)
    nodes[i] contains (x, y) coordinates for heuristic calculation
    """
    N = len(adj)
    g_score = [float("inf")] * N  # Cost from start to node
    f_score = [float("inf")] * N  # g_score + heuristic
    prev = [None] * N
    
    g_score[start_idx] = 0.0
    f_score[start_idx] = _heuristic(nodes[start_idx], nodes[goal_idx])
    
    # Priority queue: (f_score, g_score, node_idx)
    # Include g_score as tiebreaker for nodes with equal f_score
    heap = [(f_score[start_idx], 0.0, start_idx)]
    closed_set = set()

    while heap:
        f, g, u = heapq.heappop(heap)
        
        if u in closed_set:
            continue
        
        if u == goal_idx:
            break
            
        closed_set.add(u)
        
        for v, edge_cost in adj[u]:
            if v in closed_set:
                continue
                
            tentative_g = g_score[u] + edge_cost
            
            if tentative_g < g_score[v]:
                prev[v] = u
                g_score[v] = tentative_g
                f_score[v] = tentative_g + _heuristic(nodes[v], nodes[goal_idx])
                heapq.heappush(heap, (f_score[v], tentative_g, v))

    if g_score[goal_idx] == float("inf"):
        return None

    path_idx = []
    cur = goal_idx
    while cur is not None:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return path_idx


def _get_crossing_obstacle_cost(p1: Tuple[float, float], p2: Tuple[float, float], 
                                 traversable_hulls: List[Dict[str, Any]]) -> float:
    """
    Calculate additional cost for crossing over traversable (low) obstacles.
    Returns the cost based on the height of obstacles the segment passes through.
    """
    extra_cost = 0.0
    
    for ob in traversable_hulls:
        hull_edges = obstacle_edges(ob["verts"])
        # Check if segment crosses this obstacle
        crosses = False
        for edge in hull_edges:
            if segments_intersect(p1, p2, edge[0], edge[1]):
                crosses = True
                break
        
        if crosses:
            # Add cost based on obstacle height
            height_cm = ob.get("height_cm", 0.0)
            extra_cost += height_cm * TRAVERSAL_COST_PER_CM
    
    return extra_cost


def plan_path_between_points(start: Tuple[float, float],
                             goal: Tuple[float, float],
                             hulls: List[Dict[str, Any]],
                             traversable_hulls: List[Dict[str, Any]] = None) -> Optional[List[Tuple[float, float]]]:
    """
    A* visibility-graph planner with height-based traversal costs.
    
    start, goal are XY meters.
    hulls: inflated convex obstacles that BLOCK passage (height > MAX_TRAVERSABLE_HEIGHT_CM)
    traversable_hulls: obstacles that can be crossed with additional cost (height <= MAX_TRAVERSABLE_HEIGHT_CM)
    
    Returns a list of XY points (including start+goal) or None if no path.
    """
    if traversable_hulls is None:
        traversable_hulls = []
    
    nodes = [start, goal]
    meta = [None, None]  # None or (oid, idx_in_hull, nverts)

    # Add vertices from blocking obstacles
    for ob in hulls:
        verts = ob["verts"]
        n = len(verts)
        for idx, pt in enumerate(verts):
            nodes.append(pt)
            meta.append((ob["oid"], idx, n))
    
    # Also add vertices from traversable obstacles (for potential waypoints)
    for ob in traversable_hulls:
        verts = ob["verts"]
        n = len(verts)
        for idx, pt in enumerate(verts):
            nodes.append(pt)
            meta.append((ob["oid"] + "_trav", idx, n))

    all_edges = _build_all_edges_from_hulls(hulls)  # Only blocking obstacles block edges

    N = len(nodes)
    adj = [[] for _ in range(N)]

    for i in range(N):
        for j in range(i + 1, N):
            p1 = nodes[i]
            p2 = nodes[j]

            if dist(p1, p2) <= 1e-9:
                continue

            mi = meta[i]
            mj = meta[j]

            # If both are vertices of same obstacle, only connect neighbors on that hull perimeter
            if mi is not None and mj is not None and mi[0] == mj[0]:
                n = mi[2]
                idx_i = mi[1]
                idx_j = mj[1]
                # neighbors if +/-1 mod n
                if not ((idx_i - idx_j) % n == 1 or (idx_j - idx_i) % n == 1):
                    continue

            # Check if blocked by impassable obstacles
            if _segment_blocked_by_edges(p1, p2, all_edges):
                continue

            # Base cost is distance
            base_cost = dist(p1, p2)
            
            # Add cost for crossing traversable obstacles
            traversal_cost = _get_crossing_obstacle_cost(p1, p2, traversable_hulls)
            
            total_cost = base_cost + traversal_cost
            adj[i].append((j, total_cost))
            adj[j].append((i, total_cost))

    path_idx = _astar_path(adj, nodes, 0, 1)
    if not path_idx:
        # If even start->goal intersects inflated edges, there is truly no valid route in this graph
        if _segment_blocked_by_edges(start, goal, all_edges):
            return None
        return [start, goal]

    return [nodes[k] for k in path_idx]


# ------------------- ROS2 Node -------------------

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner_node")

        self.start: Optional[Tuple[float, float]] = None  # lat, lon
        self.goal: Optional[Tuple[float, float]] = None   # lat, lon
        self.obstacles: List[Dict[str, Any]] = []

        self.create_subscription(String, "/mission/start", self.on_start, 10)
        self.create_subscription(String, "/mission/goal", self.on_goal, 10)
        self.create_subscription(String, "/world/permanent_obstacles", self.on_obstacles, 10)

        self.pub_path = self.create_publisher(String, "/planner/global_path", 10)

        self.declare_parameter("clearance_m", 5.0)
        self.declare_parameter("rejoin_range_factor", 2.0)
        self.clearance_m = float(self.get_parameter("clearance_m").value)

        self.get_logger().info(
            f"Global planner ready. clearance_m={self.clearance_m:.2f} "
            "(inflated hulls + visibility graph)."
        )

    def on_start(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.start = (float(data["lat"]), float(data["lon"]))
            self.get_logger().info(f"Start set: {data.get('name','')} ({self.start[0]:.6f}, {self.start[1]:.6f})")
            # Optionally replan if we already have a goal
            if self.goal is not None:
                self.try_plan()
        except Exception as e:
            self.get_logger().warn(f"Bad /mission/start: {e}")

    def on_goal(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.goal = (float(data["lat"]), float(data["lon"]))
            self.get_logger().info(f"Goal set: {data.get('name','')} ({self.goal[0]:.6f}, {self.goal[1]:.6f})")
            self.try_plan()
        except Exception as e:
            self.get_logger().warn(f"Bad /mission/goal: {e}")

    def on_obstacles(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.obstacles = data.get("obstacles", [])
            self.get_logger().info(f"Obstacles updated: {len(self.obstacles)}")
            # Replan if start+goal exist
            if self.start is not None and self.goal is not None:
                self.try_plan()
        except Exception as e:
            self.get_logger().warn(f"Bad /world/permanent_obstacles: {e}")

    def try_plan(self):
        if self.start is None or self.goal is None:
            return

        # Anchor projection around the middle of the mission
        lat0 = (self.start[0] + self.goal[0]) * 0.5
        lon0 = (self.start[1] + self.goal[1]) * 0.5

        start_xy = latlon_to_xy(self.start[0], self.start[1], lat0, lon0)
        goal_xy = latlon_to_xy(self.goal[0], self.goal[1], lat0, lon0)

        # Build inflated convex hulls in XY meters (clearance applied here)
        # Separate into blocking (tall) and traversable (low) obstacles
        blocking_hulls: List[Dict[str, Any]] = []
        traversable_hulls: List[Dict[str, Any]] = []
        
        for obs in self.obstacles:
            oid = str(obs.get("id", "0"))
            pts = obs.get("points", [])
            if len(pts) < 3:
                continue
            
            # Get obstacle height (default to blocking height if not specified)
            height_cm = float(obs.get("height_cm", 100.0))  # Default 100cm = blocking

            xy_pts: List[Tuple[float, float]] = []
            for p in pts:
                try:
                    lat = float(p["lat"])
                    lon = float(p["lon"])
                except Exception:
                    continue
                xy_pts.append(latlon_to_xy(lat, lon, lat0, lon0))

            if len(xy_pts) < 3:
                continue

            hull = convex_hull(xy_pts)
            if len(hull) < 3:
                continue

            hull_inflated = inflate_hull(hull, self.clearance_m)
            
            hull_data = {"oid": oid, "verts": hull_inflated, "height_cm": height_cm}
            
            # Obstacles <= 30cm can be traversed with cost penalty
            if height_cm <= MAX_TRAVERSABLE_HEIGHT_CM:
                traversable_hulls.append(hull_data)
                self.get_logger().debug(f"Obstacle {oid}: traversable (height={height_cm}cm)")
            else:
                blocking_hulls.append(hull_data)
                self.get_logger().debug(f"Obstacle {oid}: blocking (height={height_cm}cm)")
        
        self.get_logger().info(
            f"Planning with {len(blocking_hulls)} blocking obstacles, "
            f"{len(traversable_hulls)} traversable obstacles"
        )

        path_xy = plan_path_between_points(start_xy, goal_xy, blocking_hulls, traversable_hulls)

        if not path_xy:
            self.get_logger().warn(
                "No valid path found that respects inflated obstacles. Publishing empty path."
            )
            payload = {
                "path": [],
                "start": {"lat": self.start[0], "lon": self.start[1]},
                "goal": {"lat": self.goal[0], "lon": self.goal[1]},
                "valid": False
            }
            self.pub_path.publish(String(data=json.dumps(payload)))
            return

        # Convert XY back to lat/lon
        path_latlon = []
        for x, y in path_xy:
            lat, lon = xy_to_latlon(x, y, lat0, lon0)
            path_latlon.append({"lat": lat, "lon": lon})

        # Force exact endpoints
        if path_latlon:
            path_latlon[0] = {"lat": self.start[0], "lon": self.start[1]}
            path_latlon[-1] = {"lat": self.goal[0], "lon": self.goal[1]}

        payload = {
            "path": path_latlon,
            "start": {"lat": self.start[0], "lon": self.start[1]},
            "goal": {"lat": self.goal[0], "lon": self.goal[1]},
            "valid": True
        }
        self.pub_path.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"Published path with {len(path_latlon)} points")



def main():
    rclpy.init()
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
