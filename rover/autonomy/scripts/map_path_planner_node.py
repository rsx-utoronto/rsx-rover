#!/usr/bin/env python3
"""MissionState-based global path planner node.

Rover-side planner node:
- subscribes to mission_state for START_SL_AVOIDANCE requests,
- computes a path using PathPlanner,
- publishes GLOBAL_PATH_READY with global_planner_waypoints.
"""

from __future__ import annotations

import csv
import math
import heapq
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple, Union
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from rover.msg import MissionState
from geometry_msgs.msg import PoseStamped

import gps_conversion_functions as functions

LatLng = Tuple[float, float]
ObstacleStore = Dict[str, DefaultDict[str, List[Dict[str, float]]]]


class GlobalPathPlanner:
    """Geometry planner that builds a visibility graph around obstacle hulls."""

    def __init__(self, obstacle_clearance: float, node: Node) -> None:
        self.obstacle_clearance = max(0.0, float(obstacle_clearance))
        self.node = node

    @staticmethod
    def _dist(p1: LatLng, p2: LatLng) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    @staticmethod
    def _cross(o: LatLng, a: LatLng, b: LatLng) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def _inflate_hull(self, hull: List[LatLng]) -> List[LatLng]:
        if not hull or self.obstacle_clearance <= 0.0:
            return hull

        cx = sum(lat for lat, _ in hull) / len(hull)
        cy = sum(lng for _, lng in hull) / len(hull)

        inflated: List[LatLng] = []
        for lat, lng in hull:
            vx = lat - cx
            vy = lng - cy
            norm = math.hypot(vx, vy)
            if norm < 1e-12:
                inflated.append((lat + self.obstacle_clearance, lng))
            else:
                scale = (norm + self.obstacle_clearance) / norm
                inflated.append((cx + vx * scale, cy + vy * scale))
        return inflated

    def _convex_hull(self, pts: List[LatLng]) -> List[LatLng]:
        unique = sorted(set(pts))
        if len(unique) <= 2:
            return unique

        lower: List[LatLng] = []
        for p in unique:
            while len(lower) >= 2 and self._cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        upper: List[LatLng] = []
        for p in reversed(unique):
            while len(upper) >= 2 and self._cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        return lower[:-1] + upper[:-1]

    def collect_hulls(self, obstacles: ObstacleStore) -> List[Dict[str, object]]:
        hulls: List[Dict[str, object]] = []
        for category in ("permanent", "temporary"):
            for oid, points in obstacles.get(category, {}).items():
                raw = [(float(p["lat"]), float(p["lng"])) for p in points if "lat" in p and "lng" in p]
                if len(raw) < 2:
                    continue

                hull = self._convex_hull(raw)
                hull = self._inflate_hull(hull)
                if len(hull) >= 2:
                    hulls.append({"cat": category, "oid": oid, "verts": hull})
        return hulls

    @staticmethod
    def _segments_intersect(p1: LatLng, p2: LatLng, q1: LatLng, q2: LatLng) -> bool:
        def orient(a: LatLng, b: LatLng, c: LatLng) -> float:
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

        def on_seg(a: LatLng, b: LatLng, c: LatLng) -> bool:
            return min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])

        o1 = orient(p1, p2, q1)
        o2 = orient(p1, p2, q2)
        o3 = orient(q1, q2, p1)
        o4 = orient(q1, q2, p2)

        if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
            return True

        if o1 == 0 and on_seg(p1, p2, q1):
            return True
        if o2 == 0 and on_seg(p1, p2, q2):
            return True
        if o3 == 0 and on_seg(q1, q2, p1):
            return True
        if o4 == 0 and on_seg(q1, q2, p2):
            return True
        return False

    def _segment_blocked(self, p1: LatLng, p2: LatLng, obstacle_edges: List[Tuple[LatLng, LatLng]], tol: float = 1e-9) -> bool:
        def almost_equal(a: LatLng, b: LatLng) -> bool:
            return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

        for c, d in obstacle_edges:
            if almost_equal(p1, c) or almost_equal(p1, d) or almost_equal(p2, c) or almost_equal(p2, d):
                continue
            if self._segments_intersect(p1, p2, c, d):
                return True
        return False

    def _dijkstra_path(self, adj: List[List[Tuple[int, float]]], start_idx: int, goal_idx: int) -> Optional[List[int]]:
        n_nodes = len(adj)
        distances = [math.inf] * n_nodes
        previous: List[Optional[int]] = [None] * n_nodes
        distances[start_idx] = 0.0
        queue: List[Tuple[float, int]] = [(0.0, start_idx)]

        while queue:
            d, u = heapq.heappop(queue)
            if d > distances[u]:
                continue
            if u == goal_idx:
                break

            for v, w in adj[u]:
                nd = d + w
                if nd < distances[v]:
                    distances[v] = nd
                    previous[v] = u
                    heapq.heappush(queue, (nd, v))

        if distances[goal_idx] == math.inf:
            return None

        path: List[int] = []
        cur: Optional[int] = goal_idx
        while cur is not None:
            path.append(cur)
            cur = previous[cur]
        path.reverse()
        return path

    def plan_between_points(self, start: LatLng, goal: LatLng, hulls: List[Dict[str, object]]) -> List[LatLng]:
        self.node.get_logger().info(f" in checkpoint 1 for PATHPLANNER")
        nodes: List[LatLng] = [start, goal]
        meta: List[Optional[Tuple[str, str, int, int]]] = [None, None]

        obstacle_edges: List[Tuple[LatLng, LatLng]] = []
        for ob in hulls:
            verts = list(ob["verts"])
            n = len(verts)

            for idx, vertex in enumerate(verts):
                nodes.append(vertex)
                meta.append((str(ob["cat"]), str(ob["oid"]), idx, n))

            if n == 2:
                obstacle_edges.append((verts[0], verts[1]))
            elif n > 2:
                for i in range(n):
                    obstacle_edges.append((verts[i], verts[(i + 1) % n]))

        n_nodes = len(nodes)
        adj: List[List[Tuple[int, float]]] = [[] for _ in range(n_nodes)]

        for i in range(n_nodes):
            for j in range(i + 1, n_nodes):
                p1 = nodes[i]
                p2 = nodes[j]
                if self._dist(p1, p2) == 0.0:
                    continue

                mi = meta[i]
                mj = meta[j]
                if mi is not None and mj is not None and mi[0] == mj[0] and mi[1] == mj[1]:
                    n = mi[3]
                    idx_i = mi[2]
                    idx_j = mj[2]
                    neighbours = ((idx_i - idx_j) % n == 1) or ((idx_j - idx_i) % n == 1)
                    if not neighbours:
                        continue

                if self._segment_blocked(p1, p2, obstacle_edges):
                    continue

                w = self._dist(p1, p2)
                adj[i].append((j, w))
                adj[j].append((i, w))

        path_idx = self._dijkstra_path(adj, 0, 1)
        if not path_idx:
            return [start, goal]
        return [nodes[k] for k in path_idx]


class MissionStatePathPlanner(Node):
    OBSTACLE_CLEARANCE = 1.5

    def __init__(self) -> None:
        super().__init__("mission_state_path_planner")
        self.get_logger().info(f" in first PATHPLANNER")

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._mission_pub = self.create_publisher(MissionState, "mission_state", qos)
        self._mission_sub = self.create_subscription(
            MissionState,
            "mission_state",
            self._mission_state_callback,
            qos,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/pose",
            self.pose_callback,
            qos,
        )
        self.grid_search_avoidance_active = False

        self._planner = GlobalPathPlanner(self.OBSTACLE_CLEARANCE, self)
        self._obstacle_dir = Path("/home/rsx/rover_ws/src/rsx-rover/scripts/GUI")
        self._search_dirs = [self._obstacle_dir]
        self._obstacles: ObstacleStore = {
            "temporary": defaultdict(list),
            "permanent": defaultdict(list),
        }
        self.pose = PoseStamped()
        self._origin_latlng: Optional[LatLng] = None
        self._origin_warned = False
        self._origin_csv_candidates = [
            Path("~/rover_ws/src/rsx-rover").expanduser() / "long_lat_goal.csv"
        ]

        self.get_logger().info("MissionState path planner is running.")

    def pose_callback(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _mission_state_callback(self, msg: MissionState) -> None:
        if msg.state != "START_SL_AVOIDANCE" and msg.state != "GS_WAYPOINT_AVOIDANCE": 
            return
        self.get_logger().info(f"AYEEEE: {msg.state}")
        if msg.state == "GS_WAYPOINT_AVOIDANCE":
            self.grid_search_avoidance_active = True
            self.get_logger().info(f"HEYYYYYYY00000 {msg.state} in mission state callback, starting path planning {self.grid_search_avoidance_active}")
        start = (
            float(self.pose.pose.position.x),
            float(self.pose.pose.position.y),
        )
        goal = (
            float(msg.current_goal.pose.position.x),
            float(msg.current_goal.pose.position.y),
        )

        self._reload_obstacles_from_csv()
        self._maybe_update_origin()
        if not self._convert_obstacles_to_local():
            # self._publish_failed(msg, "Origin not valid; skipping path planning.")
            self.get_logger().warn("Origin not valid; skipping path planning.")
            return
        hulls = self._planner.collect_hulls(self._obstacles)

        try:
            path = self._planner.plan_between_points(start, goal, hulls)
        except Exception as exc:
            self.get_logger().error(f"Planner exception: {exc}")
            path = [start, goal]

        if not path:
            path = [start, goal]

        deduped_path = self._dedupe_points(path)
        self._publish_ready(msg, deduped_path)

    def _publish_ready(self, request_msg: MissionState, points: List[LatLng]) -> None:
        out = MissionState()
        self.get_logger().info(f"HEYYYYOOOOOOOO grid search avoidance is {self.grid_search_avoidance_active} {len(points)} point(s). Points are {points}")
        if self.grid_search_avoidance_active:
            out.state = "GLOBAL_PATH_GS_READY"
        else:
            out.state = "GLOBAL_PATH_READY"
        out.current_state = request_msg.current_state
        out.current_goal = request_msg.current_goal
        out.current_pose = request_msg.current_pose
        out.global_planner_waypoints = self._flatten_points(points)
        self._mission_pub.publish(out)
        self.grid_search_avoidance_active = False

    # def _publish_failed(self, request_msg: MissionState, reason: str) -> None:
    #     out = MissionState()
    #     out.state = "GLOBAL_PATH_FAILED"
    #     out.current_state = request_msg.current_state
    #     out.current_goal = request_msg.current_goal
    #     out.current_pose = request_msg.current_pose
    #     out.global_planner_waypoints = []
    #     self._mission_pub.publish(out)
    #     self.get_logger().warn(f"Global path planning skipped: {reason}")

    @staticmethod
    def _flatten_points(points: List[LatLng]) -> List[float]:
        flattened: List[float] = []
        for lat, lng in points:
            flattened.extend([float(lat), float(lng)])
        return flattened

    @staticmethod
    def _dedupe_points(points: List[LatLng]) -> List[LatLng]:
        seen = set()
        out: List[LatLng] = []
        for lat, lng in points:
            key = (round(lat, 8), round(lng, 8))
            if key in seen:
                continue
            seen.add(key)
            out.append((lat, lng))
        return out

    def _maybe_update_origin(self) -> None:
        if self._origin_latlng and self._origin_latlng != (0.0, 0.0):
            return

        origin_csv = None
        for candidate in self._origin_csv_candidates:
            if candidate.exists():
                origin_csv = candidate
                break

        if origin_csv is None:
            if not self._origin_warned:
                self.get_logger().warn("Origin CSV not found; skipping obstacle conversion.")
                self._origin_warned = True
            return

        try:
            with open(origin_csv, newline="") as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row:
                        continue
                    name = row[0].strip().lower()
                    if name != "start":
                        continue
                    if len(row) < 3:
                        continue
                    lat = float(row[1].strip())
                    lng = float(row[2].strip())
                    self._origin_latlng = (lat, lng)
                    self.get_logger().info(f"Origin set to {self._origin_latlng} from {origin_csv}")
                    self._origin_warned = False
                    return
        except Exception as exc:
            if not self._origin_warned:
                self.get_logger().warn(f"Failed to read origin CSV {origin_csv}: {exc}")
                self._origin_warned = True
            return

        if not self._origin_warned:
            self.get_logger().warn("Start row not found in origin CSV; skipping obstacle conversion.")
            self._origin_warned = True

    def _gps_to_local(self, lat: float, lng: float) -> Optional[LatLng]:
        if self._origin_latlng is None:
            return None
        origin_lat, origin_lng = self._origin_latlng
        distance = functions.getDistanceBetweenGPSCoordinates((origin_lat, origin_lng), (lat, lng))
        theta = functions.getHeadingBetweenGPSCoordinates(origin_lat, origin_lng, lat, lng)
        x = distance * math.sin(theta)
        y = distance * math.cos(theta)
        return (x, y)

    def _convert_obstacles_to_local(self) -> bool:
        if self._origin_latlng is None or self._origin_latlng == (0.0, 0.0):
            if not self._origin_warned:
                self.get_logger().warn("Origin unavailable; obstacles remain in GPS frame.")
                self._origin_warned = True
            return False

        for category, obstacles in self._obstacles.items():
            for oid, points in obstacles.items():
                converted: List[Dict[str, float]] = []
                for point in points:
                    try:
                        lat = float(point.get("lat"))
                        lng = float(point.get("lng"))
                    except (TypeError, ValueError):
                        continue
                    local = self._gps_to_local(lat, lng)
                    if local is None:
                        continue
                    converted.append({"lat": local[0], "lng": local[1]})
                   
                obstacles[oid] = converted
        return True

    def _reload_obstacles_from_csv(self) -> None:
        self._obstacles = {
            "temporary": defaultdict(list),
            "permanent": defaultdict(list),
        }

        self._load_obstacle_category(
            "permanent",
            self._find_latest_csv(["permanent_obstacles*.csv", "permanent_obstacles.csv"]),
        )
        self._load_obstacle_category(
            "temporary",
            self._find_latest_csv(["temporary_obstacles*.csv", "temporary_obstacles.csv"]),
        )

    def _csv_has_obstacles(self, csv_path: Path) -> bool:
        try:
            with open(csv_path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if not row:
                        continue
                    try:
                        float(row.get("lat", ""))
                        float(row.get("lng", ""))
                    except (TypeError, ValueError):
                        continue
                    return True
        except Exception:
            return False
        return False

    def _find_latest_csv(self, patterns: Union[List[str], str]) -> Optional[Path]:
        if isinstance(patterns, str):
            patterns = [patterns]

        matches: List[Path] = []
        for directory in self._search_dirs:
            for pattern in patterns:
                try:
                    matches.extend(directory.glob(pattern))
                except Exception:
                    continue

        if not matches:
            return None

        def sort_key(path: Path) -> float:
            try:
                return path.stat().st_mtime
            except Exception:
                return 0.0

        matches.sort(key=sort_key, reverse=True)
        for candidate in matches:
            if self._csv_has_obstacles(candidate):
                return candidate
        return matches[0]

    def _load_obstacle_category(self, category: str, csv_path: Optional[Path]) -> None:
        if csv_path is None or not csv_path.exists():
            return

        try:
            with open(csv_path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if not row:
                        continue

                    try:
                        lat = float(row.get("lat", ""))
                        lng = float(row.get("lng", ""))
                    except (TypeError, ValueError):
                        continue

                    oid = row.get("obstacle_id") or row.get("id") or "unknown"
                    self._obstacles[category][str(oid)].append({"lat": lat, "lng": lng})
        except Exception as exc:
            self.get_logger().warn(f"Failed to load {category} obstacles from {csv_path}: {exc}")


def main(args: Optional[List[str]] = None) -> None:
    import rclpy
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = MissionStatePathPlanner()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
