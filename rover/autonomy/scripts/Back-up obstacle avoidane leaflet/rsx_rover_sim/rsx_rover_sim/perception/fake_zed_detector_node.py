#!/usr/bin/env python3
import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def latlon_to_enu_m(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """
    Small-area equirectangular approximation:
    returns (east_m, north_m) from (lat0, lon0) to (lat, lon).
    """
    R = 6371000.0
    east = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * R
    north = math.radians(lat - lat0) * R
    return east, north


def enu_to_rover_xy(east: float, north: float, yaw_rad: float) -> Tuple[float, float]:
    """
    Convert ENU delta (east, north) into rover frame (x forward, y left),
    given yaw where 0=east and +CCW (same convention as your rover_sim_node).
    """
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    # R(-yaw) * [east; north]
    x = cy * east + sy * north
    y = -sy * east + cy * north
    return x, y


def sample_segment(p0: Tuple[float, float], p1: Tuple[float, float], step_m: float) -> List[Tuple[float, float]]:
    """
    Sample points along segment p0->p1 in ENU meters.
    Includes p0, excludes p1 (caller will add next vertex).
    """
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return [p0]
    n = max(1, int(math.ceil(L / max(1e-6, step_m))))
    pts = []
    for i in range(n):
        t = i / n
        pts.append((x0 + dx * t, y0 + dy * t))
    return pts


class FakeZedDetectorNode(Node):
    """
    Fake ZED detector milestone #2:
      - reads rover pose
      - reads TEMP obstacles ground-truth ONLY for simulation
      - publishes ONLY what the cone can "see" (size-agnostic)

    Publishes std_msgs/String JSON on:
      /perception/temporary_obstacles

    Output schema:
      {
        "source": "fake_zed",
        "stamp": <unix_time>,
        "frame": "base_link",
        "fov_deg": 150.0,
        "range_m": 25.0,
        "blocked": true/false,
        "nearest": {"x":..,"y":..,"dist":..,"angle_deg":..} | null,
        "boundary_points": [{"x":..,"y":..}, ...],
        "debug": {"num_gt_obstacles": N, "num_visible_samples": M}
      }

    NOTE:
      This node is the *only* piece allowed to read /world/temporary_obstacles in sim.
      Your local planner later must consume only /perception/temporary_obstacles.
    """

    def __init__(self):
        super().__init__("fake_zed_detector_node")

        self.declare_parameter("fov_deg", 150.0)
        self.declare_parameter("range_m", 25.0)
        self.declare_parameter("publish_hz", 8.0)
        self.declare_parameter("edge_sample_m", 0.75)   # smaller = more accurate, more points
        self.declare_parameter("max_points", 80)        # cap boundary_points

        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.range_m = float(self.get_parameter("range_m").value)
        self.edge_sample_m = float(self.get_parameter("edge_sample_m").value)
        self.max_points = int(self.get_parameter("max_points").value)

        hz = float(self.get_parameter("publish_hz").value)
        hz = max(0.1, hz)

        self.pub = self.create_publisher(String, "/perception/temporary_obstacles", 10)

        self._rover_pose: Optional[Dict[str, Any]] = None
        self._gt_temp_obs: List[Dict[str, Any]] = []

        self.create_subscription(String, "/sim/rover_pose", self._on_rover_pose, 10)
        # SIM ONLY: ground-truth temporary obstacles drawn in UI
        self.create_subscription(String, "/world/temporary_obstacles", self._on_world_temp_obstacles, 10)

        self.timer = self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"FakeZedDetectorNode: publishing cone detections on /perception/temporary_obstacles @ {hz:.1f} Hz "
            f"(fov={self.fov_deg:.1f}deg, range={self.range_m:.1f}m, sample={self.edge_sample_m:.2f}m)"
        )

    def _on_rover_pose(self, msg: String):
        try:
            self._rover_pose = json.loads(msg.data)
        except Exception:
            pass

    def _on_world_temp_obstacles(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        obs = payload.get("obstacles", [])
        if isinstance(obs, list):
            self._gt_temp_obs = obs

    def _tick(self):
        # If we don't know rover pose, publish empty
        if not self._rover_pose or "lat" not in self._rover_pose or "lon" not in self._rover_pose:
            self.pub.publish(String(data=json.dumps(self._empty_payload(blocked=False, nearest=None, points=[]))))
            return

        lat0 = float(self._rover_pose["lat"])
        lon0 = float(self._rover_pose["lon"])
        yaw = float(self._rover_pose.get("yaw", 0.0))

        half_fov_rad = math.radians(self.fov_deg) / 2.0
        range_m = self.range_m

        visible_pts: List[Tuple[float, float, float, float]] = []  # (x,y,dist,angle_deg)

        for ob in self._gt_temp_obs:
            pts_ll = ob.get("points", [])
            if not isinstance(pts_ll, list) or len(pts_ll) < 2:
                continue

            # Convert obstacle vertices to ENU meters relative rover
            enu_vertices: List[Tuple[float, float]] = []
            for p in pts_ll:
                try:
                    lat = float(p["lat"])
                    lon = float(p["lon"])
                except Exception:
                    continue
                enu_vertices.append(latlon_to_enu_m(lat, lon, lat0, lon0))

            if len(enu_vertices) < 2:
                continue

            # Treat polygon as closed ring for edge sampling
            ring = enu_vertices[:]
            if ring[0] != ring[-1]:
                ring.append(ring[0])

            # Sample along edges so big obstacles are detected even if vertices are out of view
            for i in range(len(ring) - 1):
                seg_samples = sample_segment(ring[i], ring[i + 1], self.edge_sample_m)
                for (east, north) in seg_samples:
                    x, y = enu_to_rover_xy(east, north, yaw)

                    # Cone filters: in front, within range, within angle
                    if x <= 0.0:
                        continue
                    dist = math.hypot(x, y)
                    if dist > range_m:
                        continue
                    ang = math.atan2(y, x)
                    if abs(ang) > half_fov_rad:
                        continue

                    visible_pts.append((x, y, dist, math.degrees(ang)))

        blocked = len(visible_pts) > 0
        nearest = None
        boundary_points: List[Dict[str, float]] = []

        if blocked:
            # nearest point
            nx, ny, nd, na = min(visible_pts, key=lambda t: t[2])
            nearest = {"x": nx, "y": ny, "dist": nd, "angle_deg": na}

            # downsample boundary points by sorting by angle, then thinning
            visible_pts.sort(key=lambda t: t[3])  # angle_deg
            # If too many points, take evenly spaced samples
            M = len(visible_pts)
            if M <= self.max_points:
                keep = visible_pts
            else:
                keep = []
                for k in range(self.max_points):
                    idx = int(round(k * (M - 1) / (self.max_points - 1)))
                    keep.append(visible_pts[idx])

            boundary_points = [{"x": float(x), "y": float(y)} for (x, y, _, _) in keep]

        payload = {
            "source": "fake_zed",
            "stamp": time.time(),
            "frame": "base_link",
            "fov_deg": float(self.fov_deg),
            "range_m": float(self.range_m),
            "blocked": bool(blocked),
            "nearest": nearest,
            "boundary_points": boundary_points,
            "debug": {
                "num_gt_obstacles": int(len(self._gt_temp_obs)),
                "num_visible_samples": int(len(visible_pts)),
            },
        }

        self.pub.publish(String(data=json.dumps(payload)))

    def _empty_payload(self, blocked: bool, nearest: Optional[Dict[str, Any]], points: List[Dict[str, Any]]) -> Dict[str, Any]:
        return {
            "source": "fake_zed",
            "stamp": time.time(),
            "frame": "base_link",
            "fov_deg": float(self.fov_deg),
            "range_m": float(self.range_m),
            "blocked": bool(blocked),
            "nearest": nearest,
            "boundary_points": points,
            "debug": {"num_gt_obstacles": int(len(self._gt_temp_obs)), "num_visible_samples": 0},
        }


def main():
    rclpy.init()
    node = FakeZedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()