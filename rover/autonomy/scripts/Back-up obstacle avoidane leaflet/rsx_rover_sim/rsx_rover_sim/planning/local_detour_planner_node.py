#!/usr/bin/env python3
import json
import math
import time
from typing import Optional, Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


def _safe_float(v, default=0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


# ----------------- geo helpers -----------------

def rover_xy_to_latlon(x_fwd: float, y_left: float, lat0: float, lon0: float, yaw: float) -> Tuple[float, float]:
    """Convert rover-frame meters (x forward, y left) into lat/lon using small-area approximation.
    yaw radians, 0=east and +CCW.
    """
    R = 6371000.0
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    east = cy * x_fwd - sy * y_left
    north = sy * x_fwd + cy * y_left

    lat = lat0 + math.degrees(north / R)
    lon = lon0 + math.degrees(east / (R * math.cos(math.radians(lat0))))
    return lat, lon


# ----------------- geometry helpers -----------------

def _pca_wall_direction(points_xy: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """Unit direction vector (dx,dy) along the dominant axis of points (2D PCA)."""
    if len(points_xy) < 3:
        return None

    mx = sum(p[0] for p in points_xy) / len(points_xy)
    my = sum(p[1] for p in points_xy) / len(points_xy)

    sxx = syy = sxy = 0.0
    for x, y in points_xy:
        dx = x - mx
        dy = y - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    tr = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = max(0.0, tr * tr - 4.0 * det)
    lam1 = 0.5 * (tr + math.sqrt(disc))  # largest eigenvalue

    if abs(sxy) > 1e-12:
        vx = lam1 - syy
        vy = sxy
    else:
        vx, vy = (1.0, 0.0) if sxx >= syy else (0.0, 1.0)

    L = math.hypot(vx, vy)
    if L < 1e-9:
        return None
    return (vx / L, vy / L)


def _angle_deg(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))


def _dist(x: float, y: float) -> float:
    return math.hypot(x, y)


def _resample_polyline(points: List[Tuple[float, float]], step_m: float) -> List[Tuple[float, float]]:
    """Resample an ordered polyline (in meters) at approximately regular arc-length intervals."""
    if len(points) < 2:
        return points[:]

    step_m = max(0.05, float(step_m))
    out = [points[0]]

    accum = 0.0
    target = step_m

    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        seg_dx = x1 - x0
        seg_dy = y1 - y0
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-9:
            continue

        while accum + seg_len >= target:
            t = (target - accum) / seg_len
            out.append((x0 + seg_dx * t, y0 + seg_dy * t))
            target += step_m

        accum += seg_len

    if math.hypot(out[-1][0] - points[-1][0], out[-1][1] - points[-1][1]) > 1e-6:
        out.append(points[-1])
    return out


def _thin_ordered(points: List[Tuple[float, float]], eps: float = 0.08) -> List[Tuple[float, float]]:
    """Keep points that are at least eps apart to reduce jitter."""
    out: List[Tuple[float, float]] = []
    for p in points:
        if not out:
            out.append(p)
            continue
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= eps:
            out.append(p)
    return out


class LocalDetourPlannerNode(Node):
    """
    Decision-only local planner + front wall(s) extraction (cone-only).

    Key behavior:
      - When /perception/temporary_obstacles reports blocked, rover continues until it is
        about stop_distance_from_obstacle_m away FROM THE WALL AHEAD (uses nearest.x),
        then stops, decides, extracts/stores front wall(s).
    """

    def __init__(self):
        super().__init__("local_detour_planner_node")

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        self.pub_active = self.create_publisher(String, "/planner/active_path", latched)
        self.pub_cmd = self.create_publisher(String, "/sim/behavior_cmd", 10)
        self.pub_decision = self.create_publisher(String, "/planner/local_decision", 10)
        self.pub_front_walls = self.create_publisher(String, "/planner/front_walls", 10)

        self.create_subscription(String, "/planner/global_path", self.on_global_path, 10)
        self.create_subscription(String, "/perception/temporary_obstacles", self.on_perception, 10)
        self.create_subscription(String, "/sim/rover_pose", self.on_rover_pose, 10)

        # --- stop behavior params ---
        self.declare_parameter("stop_distance_from_obstacle_m", 2.0)
        self.stop_distance_from_obstacle_m = float(self.get_parameter("stop_distance_from_obstacle_m").value)

        # NEW: lead buffer to account for sim tick / command latency
        self.declare_parameter("stop_lead_m", 0.75)
        self.stop_lead_m = float(self.get_parameter("stop_lead_m").value)

        # Only treat as "in front / relevant" if obstacle is within this lateral band
        # (prevents stopping for something visible but far to the side)
        self.declare_parameter("stop_lateral_gate_m", 2.5)
        self.stop_lateral_gate_m = float(self.get_parameter("stop_lateral_gate_m").value)

        self.declare_parameter("emergency_stop_forward_m", 0.75)
        self.emergency_stop_forward_m = float(self.get_parameter("emergency_stop_forward_m").value)

        self.declare_parameter("min_decision_period_s", 0.5)
        self.min_decision_period_s = float(self.get_parameter("min_decision_period_s").value)

        self.declare_parameter("unblocked_clear_s", 0.3)
        self.unblocked_clear_s = float(self.get_parameter("unblocked_clear_s").value)

        # --- decision params (cost model) ---
        self.declare_parameter("safe_clearance_m", 2.0)
        self.safe_clearance_m = float(self.get_parameter("safe_clearance_m").value)

        self.declare_parameter("forward_buffer_m", 1.0)
        self.forward_buffer_m = float(self.get_parameter("forward_buffer_m").value)

        self.declare_parameter("edge_band_frac", 0.20)
        self.edge_band_frac = float(self.get_parameter("edge_band_frac").value)

        # --- wall extraction tuning ---
        self.declare_parameter("front_wall_angle_window_deg", 20.0)
        self.front_wall_angle_window_deg = float(self.get_parameter("front_wall_angle_window_deg").value)

        self.declare_parameter("front_wall_dist_window_m", 6.0)
        self.front_wall_dist_window_m = float(self.get_parameter("front_wall_dist_window_m").value)

        self.declare_parameter("wall_perp_tol_m", 0.8)
        self.wall_perp_tol_m = float(self.get_parameter("wall_perp_tol_m").value)

        self.declare_parameter("wall_cluster_gap_m", 1.5)
        self.wall_cluster_gap_m = float(self.get_parameter("wall_cluster_gap_m").value)

        self.declare_parameter("wall_sample_step_m", 0.5)
        self.wall_sample_step_m = float(self.get_parameter("wall_sample_step_m").value)

        self.declare_parameter("wall_forward_only", True)
        self.wall_forward_only = bool(self.get_parameter("wall_forward_only").value)

        self.declare_parameter("wall_store_jsonl_path", "")
        self.wall_store_jsonl_path = str(self.get_parameter("wall_store_jsonl_path").value)

        # --- state ---
        self.last_global: Optional[Dict[str, Any]] = None
        self.last_pose: Optional[Dict[str, Any]] = None
        self.last_perception: Optional[Dict[str, Any]] = None

        self.state = "FOLLOWING"  # FOLLOWING | STOPPED_FOR_DECISION
        self.last_decision_t = 0.0
        self.last_cmd_t = 0.0
        self.unblocked_since: Optional[float] = None

        self.detected_wall_records: List[Dict[str, Any]] = []
        self.front_walls_xy: List[List[Dict[str, float]]] = []
        self.front_walls_latlon: List[List[Dict[str, float]]] = []

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info("LocalDetourPlanner: stop ~2m BEFORE wall ahead (uses nearest.x), then decide + store")

    # ---------------- callbacks ----------------

    def on_global_path(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad /planner/global_path JSON: {e}")
            return
        self.last_global = payload
        self.pub_active.publish(String(data=json.dumps(payload)))

    def on_rover_pose(self, msg: String):
        try:
            self.last_pose = json.loads(msg.data)
        except Exception:
            return

    def on_perception(self, msg: String):
        try:
            self.last_perception = json.loads(msg.data)
        except Exception:
            self.last_perception = None

    # ---------------- main loop ----------------

    def tick(self):
        now = time.time()

        if self.state == "FOLLOWING":
            if self._is_blocked():
                # blocked but we intentionally keep moving until we're ~2m from the wall ahead
                if self._should_stop_now():
                    self._send_cmd_rate_limited("stop", min_period_s=0.05)

                    if (now - self.last_decision_t) >= self.min_decision_period_s:
                        decision, costs = self._decide_cost_based(self.last_perception)
                        self.front_walls_xy = self._extract_front_walls_xy(self.last_perception)
                        self.front_walls_latlon = self._walls_xy_to_latlon(self.front_walls_xy, self.last_pose)

                        rec = {
                            "t": now,
                            "decision": decision,
                            "costs": costs,
                            "nearest": (self.last_perception or {}).get("nearest") if isinstance(self.last_perception, dict) else None,
                            "front_walls_xy": self.front_walls_xy,
                            "front_walls_latlon": self.front_walls_latlon,
                            "stop": {
                                "stop_distance_from_obstacle_m": self.stop_distance_from_obstacle_m,
                                "stop_lead_m": self.stop_lead_m,
                                "stop_lateral_gate_m": self.stop_lateral_gate_m,
                            },
                        }

                        self.detected_wall_records.append(rec)
                        self._append_jsonl_if_enabled(rec)

                        self.pub_front_walls.publish(String(data=json.dumps({"t": now, "walls_xy": self.front_walls_xy})))
                        self.pub_decision.publish(String(data=json.dumps(rec)))

                        nx, ny, nd = self._nearest_xyz(self.last_perception)
                        self.get_logger().info(
                            f"[STOP+DECIDE] {decision} | nearest_x={nx:.2f}m y={ny:.2f}m dist={nd:.2f}m | "
                            f"stop_at_x<={self.stop_distance_from_obstacle_m + self.stop_lead_m:.2f}m | "
                            f"walls={len(self.front_walls_xy)} pts={[len(w) for w in self.front_walls_xy]}"
                        )

                        self.last_decision_t = now
                        self.unblocked_since = None

                    self.state = "STOPPED_FOR_DECISION"
                else:
                    # keep moving
                    self._send_cmd_rate_limited("follow", min_period_s=0.10)
            else:
                self._send_cmd_rate_limited("follow", min_period_s=0.50)
            return

        # STOPPED_FOR_DECISION
        if self._is_blocked():
            self._send_cmd_rate_limited("stop", min_period_s=0.10)
            self.unblocked_since = None
            return

        if self.unblocked_since is None:
            self.unblocked_since = now

        if (now - self.unblocked_since) >= self.unblocked_clear_s:
            self._send_cmd_rate_limited("follow", min_period_s=0.20)
            self.state = "FOLLOWING"
            self.unblocked_since = None

    # ---------------- stop logic (FIXED) ----------------

    def _nearest_xyz(self, perception: Optional[Dict[str, Any]]) -> Tuple[float, float, float]:
        """Return (nearest_x, nearest_y, nearest_dist). Falls back to (0,0,inf) if missing."""
        if not isinstance(perception, dict):
            return 0.0, 0.0, float("inf")
        nearest = perception.get("nearest")
        if not isinstance(nearest, dict):
            return 0.0, 0.0, float("inf")

        x = _safe_float(nearest.get("x", 0.0), 0.0)
        y = _safe_float(nearest.get("y", 0.0), 0.0)

        d = nearest.get("dist", None)
        try:
            d = float(d) if d is not None else math.hypot(x, y)
        except Exception:
            d = math.hypot(x, y)
        return x, y, d

    def _should_stop_now(self) -> bool:
        """
        Stop based on forward distance to the nearest blocking surface:
          - primary metric: nearest.x (meters forward)
          - lateral gate: |nearest.y| <= stop_lateral_gate_m
          - stop threshold: stop_distance_from_obstacle_m + stop_lead_m
          - emergency stop if nearest.x <= emergency_stop_forward_m
        """
        nx, ny, _ = self._nearest_xyz(self.last_perception)

        # If the nearest point is behind us (shouldn't happen with cone-only, but safe)
        if nx <= 0.0:
            return True

        # If obstacle is very far to the side, don't stop just because it's visible
        if abs(ny) > self.stop_lateral_gate_m:
            return False

        if nx <= self.emergency_stop_forward_m:
            return True

        stop_x = self.stop_distance_from_obstacle_m + self.stop_lead_m
        return nx <= stop_x

    # ---------------- plumbing ----------------

    def _append_jsonl_if_enabled(self, rec: Dict[str, Any]):
        if not self.wall_store_jsonl_path:
            return
        try:
            with open(self.wall_store_jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception as e:
            self.get_logger().warn(f"Failed writing wall JSONL: {e}")

    def _send_cmd_rate_limited(self, cmd: str, min_period_s: float):
        now = time.time()
        if (now - self.last_cmd_t) < min_period_s:
            return
        self.pub_cmd.publish(String(data=json.dumps({"cmd": cmd})))
        self.last_cmd_t = now

    def _is_blocked(self) -> bool:
        return bool(self.last_perception.get("blocked", False)) if isinstance(self.last_perception, dict) else False

    # ---------------- perception cloud ----------------

    def _extract_cloud(self, perception: Optional[Dict[str, Any]]) -> List[Tuple[float, float]]:
        if not isinstance(perception, dict):
            return []
        pts = perception.get("boundary_points", [])
        if not isinstance(pts, list):
            return []

        cloud: List[Tuple[float, float]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            x = _safe_float(p.get("x", 0.0), 0.0)
            y = _safe_float(p.get("y", 0.0), 0.0)
            if self.wall_forward_only and x <= 0.0:
                continue
            cloud.append((x, y))
        return cloud

    # -------- decision --------

    def _decide_cost_based(self, perception: Optional[Dict[str, Any]]) -> Tuple[str, Dict[str, Any]]:
        cloud = self._extract_cloud(perception)
        if len(cloud) < 3:
            return "LEFT", {"method": "fallback_insufficient_points"}

        ys = [y for _, y in cloud]
        min_y = min(ys)
        max_y = max(ys)
        width = max(1e-6, max_y - min_y)

        band = max(0.05, min(0.45, self.edge_band_frac)) * width
        left_thresh = max_y - band
        right_thresh = min_y + band

        left_edge_pts = [(x, y) for (x, y) in cloud if y >= left_thresh]
        right_edge_pts = [(x, y) for (x, y) in cloud if y <= right_thresh]

        left_max_x = max((x for x, _ in left_edge_pts), default=None)
        right_max_x = max((x for x, _ in right_edge_pts), default=None)
        overall_max_x = max(x for x, _ in cloud)

        left_target_y = max_y + self.safe_clearance_m
        right_target_y = min_y - self.safe_clearance_m

        left_lat = 2.0 * abs(left_target_y)
        right_lat = 2.0 * abs(right_target_y)

        left_fwd = (left_max_x if left_max_x is not None else overall_max_x) + self.forward_buffer_m
        right_fwd = (right_max_x if right_max_x is not None else overall_max_x) + self.forward_buffer_m

        left_cost = left_lat + left_fwd
        right_cost = right_lat + right_fwd

        penalty = 0.25 * overall_max_x
        if left_max_x is None and right_max_x is not None:
            left_cost += penalty
        if right_max_x is None and left_max_x is not None:
            right_cost += penalty

        decision = "LEFT" if left_cost < right_cost else "RIGHT"

        costs = {
            "method": "cost_based",
            "min_y": min_y,
            "max_y": max_y,
            "safe_clearance_m": self.safe_clearance_m,
            "forward_buffer_m": self.forward_buffer_m,
            "left": {"edge_points": len(left_edge_pts), "edge_max_x": left_max_x, "target_y": left_target_y, "total_cost": left_cost},
            "right": {"edge_points": len(right_edge_pts), "edge_max_x": right_max_x, "target_y": right_target_y, "total_cost": right_cost},
        }
        return decision, costs

    # -------- wall extraction (same as before) --------

    def _extract_front_walls_xy(self, perception: Optional[Dict[str, Any]]) -> List[List[Dict[str, float]]]:
        if not isinstance(perception, dict):
            return []

        nearest = perception.get("nearest")
        if not isinstance(nearest, dict):
            return []

        cloud = self._extract_cloud(perception)
        if len(cloud) < 3:
            return []

        nx = _safe_float(nearest.get("x", 0.0), 0.0)
        ny = _safe_float(nearest.get("y", 0.0), 0.0)

        nd = nearest.get("dist", None)
        nd = _dist(nx, ny) if nd is None else _safe_float(nd, _dist(nx, ny))

        na = nearest.get("angle_deg", None)
        na = _angle_deg(nx, ny) if na is None else _safe_float(na, _angle_deg(nx, ny))

        aw = float(self.front_wall_angle_window_deg)
        dw = float(self.front_wall_dist_window_m)

        candidates: List[Tuple[float, float]] = []
        for x, y in cloud:
            d = _dist(x, y)
            a = _angle_deg(x, y)
            da = (a - na + 180.0) % 360.0 - 180.0
            if abs(da) <= aw and abs(d - nd) <= dw:
                candidates.append((x, y))

        if len(candidates) < 3:
            candidates = []
            for x, y in cloud:
                d = _dist(x, y)
                a = _angle_deg(x, y)
                da = (a - na + 180.0) % 360.0 - 180.0
                if abs(da) <= (aw * 1.5) and abs(d - nd) <= (dw * 1.5):
                    candidates.append((x, y))

        if len(candidates) < 3:
            return []

        dir_xy = _pca_wall_direction(candidates)
        if dir_xy is None:
            return []
        dx, dy = dir_xy
        px, py = -dy, dx

        mx = sum(x for x, _ in candidates) / len(candidates)
        my = sum(y for _, y in candidates) / len(candidates)

        filt: List[Tuple[float, float, float]] = []
        tol = float(self.wall_perp_tol_m)
        for x, y in candidates:
            rx = x - mx
            ry = y - my
            perp = abs(rx * px + ry * py)
            if perp <= tol:
                s = rx * dx + ry * dy
                filt.append((s, x, y))

        if len(filt) < 3:
            return []

        filt.sort(key=lambda t: t[0])
        clusters: List[List[Tuple[float, float, float]]] = []
        cur: List[Tuple[float, float, float]] = [filt[0]]

        gap = float(self.wall_cluster_gap_m)
        for t in filt[1:]:
            if abs(t[0] - cur[-1][0]) > gap:
                clusters.append(cur)
                cur = [t]
            else:
                cur.append(t)
        clusters.append(cur)

        clusters = [c for c in clusters if len(c) >= 3]
        if not clusters:
            return []

        step = float(self.wall_sample_step_m)
        walls_out: List[List[Dict[str, float]]] = []

        for c in clusters:
            c.sort(key=lambda t: t[0])
            ordered_xy = [(x, y) for (_, x, y) in c]
            ordered_xy = _thin_ordered(ordered_xy, eps=0.08)
            if len(ordered_xy) < 2:
                continue

            sampled_xy = _resample_polyline(ordered_xy, step)
            wall_pts = [{"x": float(x), "y": float(y)} for (x, y) in sampled_xy]
            if len(wall_pts) >= 3:
                walls_out.append(wall_pts)

        return walls_out

    def _walls_xy_to_latlon(
        self,
        walls_xy: List[List[Dict[str, float]]],
        pose: Optional[Dict[str, Any]],
    ) -> List[List[Dict[str, float]]]:
        if not isinstance(pose, dict):
            return []

        lat0 = pose.get("lat", None)
        lon0 = pose.get("lon", None)
        if lat0 is None or lon0 is None:
            return []

        lat0f = _safe_float(lat0, None)
        lon0f = _safe_float(lon0, None)
        if lat0f is None or lon0f is None:
            return []

        yaw = _safe_float(pose.get("yaw", 0.0), 0.0)

        out: List[List[Dict[str, float]]] = []
        for wall in walls_xy:
            ll_wall: List[Dict[str, float]] = []
            for p in wall:
                x = _safe_float(p.get("x", 0.0), 0.0)
                y = _safe_float(p.get("y", 0.0), 0.0)
                lat, lon = rover_xy_to_latlon(x, y, lat0f, lon0f, yaw)
                ll_wall.append({"lat": float(lat), "lon": float(lon)})
            if ll_wall:
                out.append(ll_wall)
        return out


def main():
    rclpy.init()
    node = LocalDetourPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
