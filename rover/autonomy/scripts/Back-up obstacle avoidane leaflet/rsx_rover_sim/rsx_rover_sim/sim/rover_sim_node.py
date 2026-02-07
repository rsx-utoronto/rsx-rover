#!/usr/bin/env python3
import json
import math
import time
from typing import List, Tuple, Optional, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    R = 6371000.0
    x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * R
    y = math.radians(lat - lat0) * R
    return x, y


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    R = 6371000.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


def angle_of_segment(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])

class RoverSimNode(Node):
    """
    Minimal rover sim (no Step 8-9 behavior system):
      Subscribes: /planner/global_path  (JSON {"path":[{lat,lon},...]})
      Publishes:  /sim/rover_pose       (JSON {lat,lon,yaw,status,t})
    """

    def __init__(self):
        super().__init__("rover_sim_node")

        self.declare_parameter("speed_mps", 2.0)
        self.declare_parameter("rate_hz", 20.0)

        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.sub_path = self.create_subscription(String, "/planner/active_path", self.on_path, 10)
        self.pub_pose = self.create_publisher(String, "/sim/rover_pose", 10)

        self.sub_cmd = self.create_subscription(String, "/sim/behavior_cmd", self.on_cmd, 10)

        # scan state (rotate-in-place)
        self.scanning: bool = False
        self.scan_plan: List[float] = []        # relative yaw targets (radians)
        self.scan_i: int = 0
        self.scan_rate: float = math.radians(35.0)  # rad/s
        self.scan_hold_s: float = 0.35
        self.scan_hold_until: float = 0.0
        self.scan_center_yaw: float = 0.0


        self.pub_scan_done = self.create_publisher(String, "/sim/scan_done", 10)

        # scan state
        self.scanning: bool = False
        self.scan_plan: List[float] = []     # yaw targets (radians)
        self.scan_i: int = 0
        self.scan_rate: float = math.radians(35.0)  # rad/s (tweak)
        self.scan_hold_s: float = 0.35
        self.scan_hold_until: float = 0.0
        self.scan_center_yaw: float = 0.0

        self.paused: bool = False
        self.last_pose: Dict[str, Any] = {"status": "idle", "t": time.time()}

        self.path_latlon: List[Tuple[float, float]] = []
        self._last_pts: List[Tuple[float, float]] = []
        self.path_xy: List[Tuple[float, float]] = []
        self.seg_lengths: List[float] = []
        self.seg_index: int = 0
        self.seg_progress: float = 0.0

        self.lat0: float = 0.0
        self.lon0: float = 0.0
        self.cur_yaw: float = 0.0

        self.moving: bool = False
        self.last_t = time.monotonic()

        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self.on_tick)

        self.get_logger().info(
            f"RoverSim ready. speed_mps={self.speed_mps}, rate_hz={self.rate_hz}. "
            "Listening on /planner/global_path"
        )

    def on_path(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad /planner/global_path JSON: {e}")
            return

        path = payload.get("path", [])
        if not isinstance(path, list) or len(path) < 2:
            self.path_latlon = []
            self.path_xy = []
            self.seg_lengths = []
            self.seg_index = 0
            self.seg_progress = 0.0
            self.moving = False
            self._publish_pose(status="idle")
            return

        pts: List[Tuple[float, float]] = []
        for p in path:
            try:
                pts.append((float(p["lat"]), float(p["lon"])))
            except Exception:
                continue
        
        if pts == self._last_pts:
            return
        self._last_pts = pts

        if len(pts) < 2:
            return

        # Anchor projection around average to reduce distortion
        self.lat0 = sum(lat for lat, _ in pts) / len(pts)
        self.lon0 = sum(lon for _, lon in pts) / len(pts)

        self.path_latlon = pts
        self.path_xy = [latlon_to_xy(lat, lon, self.lat0, self.lon0) for lat, lon in pts]

        self.seg_lengths = []
        for i in range(len(self.path_xy) - 1):
            ax, ay = self.path_xy[i]
            bx, by = self.path_xy[i + 1]
            self.seg_lengths.append(math.hypot(bx - ax, by - ay))

        self.seg_index = 0
        self.seg_progress = 0.0
        self.moving = True

        if len(self.path_xy) >= 2:
            self.cur_yaw = angle_of_segment(self.path_xy[0], self.path_xy[1])

        self.get_logger().info(f"New path received: {len(self.path_xy)} pts. Starting motion.")
        self._publish_pose(lat=pts[0][0], lon=pts[0][1], yaw=self.cur_yaw, status="moving")

    def on_cmd(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        cmd = str(payload.get("cmd", "")).strip().lower()
        if cmd == "stop":
            self.paused = True
            self.moving = False
            # publish a status update immediately
            self._publish_pose(
                lat=self.last_pose.get("lat"),
                lon=self.last_pose.get("lon"),
                yaw=self.last_pose.get("yaw"),
                status="stopped"
            )
            return
        
        if cmd == "scan":
            # Stop motion and rotate in place through a sequence of yaw targets.
            # payload: {"cmd":"scan","angles_deg":[75,-75,0],"rate_deg_s":35,"hold_s":0.35}
            angles_deg = payload.get("angles_deg", [75.0, -75.0, 0.0])
            try:
                self.scan_plan = [math.radians(float(a)) for a in angles_deg]
            except Exception:
                self.scan_plan = [math.radians(75.0), math.radians(-75.0), 0.0]

            rate_deg_s = float(payload.get("rate_deg_s", 35.0))
            self.scan_rate = math.radians(max(5.0, rate_deg_s))

            self.scan_hold_s = float(payload.get("hold_s", 0.35))
            self.scan_hold_s = max(0.05, self.scan_hold_s)

            self.scanning = True
            self.scan_i = 0
            self.scan_hold_until = 0.0
            self.scan_center_yaw = float(self.last_pose.get("yaw", self.cur_yaw))

            # Stop moving while scanning
            self.paused = True
            self.moving = False

            # publish immediately
            self._publish_pose(
                lat=self.last_pose.get("lat"),
                lon=self.last_pose.get("lon"),
                yaw=self.last_pose.get("yaw", self.cur_yaw),
                status="scanning"
            )
            return

        if cmd == "follow":
            self.paused = False
            # only resume if we have a path and haven't arrived
            if len(self.path_xy) >= 2 and self.seg_index < len(self.seg_lengths):
                self.moving = (not self.paused)
                self._publish_pose(
                    lat=self.last_pose.get("lat"),
                    lon=self.last_pose.get("lon"),
                    yaw=self.last_pose.get("yaw"),
                    status="moving"
                )
            return
        
        if cmd == "scan":
            # payload example:
            # {"cmd":"scan","angles_deg":[75,-75,0],"rate_deg_s":35,"hold_s":0.35}
            angles_deg = payload.get("angles_deg", [75.0, -75.0, 0.0])
            try:
                self.scan_plan = [math.radians(float(a)) for a in angles_deg]
            except Exception:
                self.scan_plan = [math.radians(75.0), math.radians(-75.0), 0.0]

            rate_deg_s = float(payload.get("rate_deg_s", 35.0))
            self.scan_rate = math.radians(max(5.0, rate_deg_s))

            self.scan_hold_s = float(payload.get("hold_s", 0.35))
            self.scan_hold_s = max(0.05, self.scan_hold_s)

            # start scanning
            self.scanning = True
            self.scan_i = 0
            self.scan_hold_until = 0.0
            self.scan_center_yaw = float(self.last_pose.get("yaw", self.cur_yaw))

            # stop moving while scanning
            self.paused = True
            self.moving = False

            self._publish_pose(
                lat=self.last_pose.get("lat"),
                lon=self.last_pose.get("lon"),
                yaw=self.last_pose.get("yaw", self.cur_yaw),
                status="scanning"
            )
            return

    def on_tick(self):
        now = time.monotonic()
        dt = now - self.last_t
        self.last_t = now
        
        # --- scanning: rotate in place, publish scan_done when finished ---
        if self.scanning:
            lat = self.last_pose.get("lat")
            lon = self.last_pose.get("lon")
            yaw = float(self.last_pose.get("yaw", self.cur_yaw))

            # hold at target yaw?
            if self.scan_hold_until > 0.0 and now < self.scan_hold_until:
                self._publish_pose(lat=lat, lon=lon, yaw=yaw, status="scanning")
                return

            # finished all targets
            if self.scan_i >= len(self.scan_plan):
                self.scanning = False
                self.paused = True
                self.moving = False

                self.pub_scan_done.publish(String(data=json.dumps({
                    "t": time.time(),
                    "status": "done",
                    "yaw": yaw
                })))

                self._publish_pose(lat=lat, lon=lon, yaw=yaw, status="stopped")
                return

            target = self.scan_center_yaw + self.scan_plan[self.scan_i]
            err = (target - yaw + math.pi) % (2 * math.pi) - math.pi

            max_step = self.scan_rate * dt
            if abs(err) <= max_step:
                yaw = target
                self.scan_hold_until = now + self.scan_hold_s
                self.scan_i += 1
            else:
                yaw = yaw + max_step * (1.0 if err > 0 else -1.0)

            self.cur_yaw = yaw
            self._publish_pose(lat=lat, lon=lon, yaw=yaw, status="scanning")
            return


        if not self.moving or len(self.path_xy) < 2:
            return

        step = self.speed_mps * dt

        while step > 0.0 and self.seg_index < len(self.seg_lengths):
            seg_len = self.seg_lengths[self.seg_index]
            if seg_len < 1e-6:
                self.seg_index += 1
                self.seg_progress = 0.0
                continue

            remaining = seg_len - self.seg_progress
            if step < remaining:
                self.seg_progress += step
                step = 0.0
            else:
                step -= remaining
                self.seg_index += 1
                self.seg_progress = 0.0

        if self.seg_index >= len(self.seg_lengths):
            lat, lon = self.path_latlon[-1]
            if len(self.path_xy) >= 2:
                self.cur_yaw = angle_of_segment(self.path_xy[-2], self.path_xy[-1])
            self._publish_pose(lat=lat, lon=lon, yaw=self.cur_yaw, status="arrived")
            self.moving = False
            return

        a = self.path_xy[self.seg_index]
        b = self.path_xy[self.seg_index + 1]
        seg_len = self.seg_lengths[self.seg_index]
        f = 0.0 if seg_len < 1e-6 else (self.seg_progress / seg_len)

        x = a[0] + (b[0] - a[0]) * f
        y = a[1] + (b[1] - a[1]) * f

        lat, lon = xy_to_latlon(x, y, self.lat0, self.lon0)
        self.cur_yaw = angle_of_segment(a, b)

        self._publish_pose(lat=lat, lon=lon, yaw=self.cur_yaw, status="moving")

    def _publish_pose(self, lat: Optional[float] = None, lon: Optional[float] = None,
                      yaw: Optional[float] = None, status: str = "idle"):
        payload: Dict[str, Any] = {"status": status, "t": time.time()}
        if lat is not None and lon is not None:
            payload["lat"] = float(lat)
            payload["lon"] = float(lon)
        if yaw is not None:
            payload["yaw"] = float(yaw)
        self.last_pose = payload
        self.pub_pose.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = RoverSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
