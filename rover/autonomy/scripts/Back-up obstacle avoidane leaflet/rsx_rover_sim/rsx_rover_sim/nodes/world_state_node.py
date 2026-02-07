#!/usr/bin/env python3
import os
import glob
import json
import csv
import time
from typing import Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


def _pick_first(row: Dict[str, Any], keys: List[str]) -> str:
    for k in keys:
        if k in row and row[k] not in (None, ""):
            return row[k]
    return ""


def _dedupe_points(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    seen = set()
    out = []
    for lat, lon in points:
        key = (round(lat, 7), round(lon, 7))
        if key not in seen:
            seen.add(key)
            out.append((lat, lon))
    return out


def _convex_hull_lonlat(points_latlon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Monotone chain convex hull.
    Input: [(lat, lon), ...]
    Hull computed in (x=lon, y=lat).
    Output: hull as [(lat, lon), ...] in boundary order.
    """
    pts = [(lon, lat) for (lat, lon) in points_latlon]  # (x, y)
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return [(y, x) for (x, y) in pts]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

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
    return [(y, x) for (x, y) in hull]


def load_permanent_obstacles_csv(csv_path: str) -> List[Dict[str, Any]]:
    """
    Reads legacy point-row CSV and groups by obstacle_id.
    Returns obstacles with points in a stable boundary order (convex hull) for display/editing.
    """
    if not csv_path or not os.path.exists(csv_path):
        return []

    obstacles_by_id: Dict[str, Dict[str, Any]] = {}

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f, skipinitialspace=True)
        for row in reader:
            row = {(k or "").strip().lower(): (v or "").strip() for k, v in row.items()}

            lat_s = _pick_first(row, ["lat", "latitude", "y"])
            lon_s = _pick_first(row, ["lng", "lon", "longitude", "x"])
            if not lat_s or not lon_s:
                continue
            try:
                lat = float(lat_s)
                lon = float(lon_s)
            except ValueError:
                continue

            oid = _pick_first(row, ["obstacle_id", "obstacle", "oid", "group_id"])
            if not oid:
                oid = "0"

            label = _pick_first(row, ["label", "type", "category"]) or "permanent"

            if oid not in obstacles_by_id:
                obstacles_by_id[oid] = {"id": str(oid), "label": label, "points": []}

            obstacles_by_id[oid]["points"].append((lat, lon))

    obstacles = []
    for oid, o in obstacles_by_id.items():
        pts = _dedupe_points(o["points"])
        if len(pts) >= 3:
            pts = _convex_hull_lonlat(pts)
        obstacles.append({
            "id": str(oid),
            "label": o.get("label", "permanent"),
            "points": [{"lat": lat, "lon": lon} for (lat, lon) in pts]
        })

    def sort_key(item):
        try:
            return int(item["id"])
        except Exception:
            return item["id"]

    return sorted(obstacles, key=sort_key)


def write_permanent_obstacles_csv(csv_path: str, obstacles: List[Dict[str, Any]]):
    """
    Writes to a legacy-compatible format:
      id,timestamp,lat,lng,label,obstacle_id
    One row per vertex.
    """
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    fieldnames = ["id", "timestamp", "lat", "lng", "label", "obstacle_id"]
    now = time.time()
    row_id = 0

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for o in obstacles:
            oid = str(o.get("id", "0"))
            label = o.get("label", "permanent") or "permanent"
            pts = o.get("points", [])
            for pt in pts:
                row_id += 1
                writer.writerow({
                    "id": str(row_id),
                    "timestamp": f"{now:.6f}",
                    "lat": f"{pt['lat']:.8f}",
                    "lng": f"{pt['lon']:.8f}",
                    "label": label,
                    "obstacle_id": oid
                })


class WorldStateNode(Node):
    def __init__(self):
        super().__init__("world_state_node")

        default_gui_dir = os.path.expanduser("~/rover_ws/src/rsx-rover/scripts/GUI")
        self.declare_parameter("gui_dir", default_gui_dir)
        self.gui_dir = os.path.expanduser(os.path.expandvars(
            self.get_parameter("gui_dir").get_parameter_value().string_value
        ))

        # load from latest permanent_obstacles_*.csv
        self.perm_pattern = "permanent_obstacles_*.csv"
        self.output_csv = os.path.join(self.gui_dir, "permanent_obstacles_sim.csv")

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub_perm = self.create_publisher(String, "/world/permanent_obstacles", qos)

        # UI sends permanent obstacle edits here
        self.sub_events = self.create_subscription(
            String,
            "/ui/permanent_obstacles/events",
            self.on_ui_event,
            10
        )

        self.permanent_obstacles: Dict[str, Dict[str, Any]] = {}

        self._load_initial()
        self._publish_perm()

        self.get_logger().info(f"WorldState ready. gui_dir={self.gui_dir}")
        self.get_logger().info(f"Persistence file: {self.output_csv}")

    def _latest_file(self, pattern: str) -> str:
        matches = glob.glob(os.path.join(self.gui_dir, pattern))
        if not matches:
            return ""
        matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return matches[0]

    def _load_initial(self):
        latest = self._latest_file(self.perm_pattern)
        if not latest:
            self.get_logger().warn("No permanent_obstacles_*.csv found; starting empty.")
            return

        obs = load_permanent_obstacles_csv(latest)
        self.permanent_obstacles = {o["id"]: o for o in obs}
        self.get_logger().info(f"Loaded {len(obs)} permanent obstacles from {latest}")

        # Write a clean editable copy immediately (so your latest file becomes predictable)
        write_permanent_obstacles_csv(self.output_csv, list(self.permanent_obstacles.values()))

    def _publish_perm(self):
        payload = {"obstacles": list(self.permanent_obstacles.values())}
        self.pub_perm.publish(String(data=json.dumps(payload)))

    def _autosave(self):
        write_permanent_obstacles_csv(self.output_csv, list(self.permanent_obstacles.values()))

    def _next_numeric_id(self) -> str:
        nums = []
        for k in self.permanent_obstacles.keys():
            try:
                nums.append(int(k))
            except Exception:
                pass
        return str(max(nums) + 1 if nums else 1)

    def on_ui_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Bad UI event JSON: {e}")
            return

        ev_type = ev.get("type", "")
        changed = False

        if ev_type == "created":
            # If UI didn't supply an ID, assign an integer ID like map_viewer_2
            oid = str(ev.get("id") or self._next_numeric_id())
            pts = ev.get("points", [])
            if isinstance(pts, list) and len(pts) >= 2:
                self.permanent_obstacles[oid] = {
                    "id": oid,
                    "label": "permanent",
                    "points": pts
                }
                changed = True

        elif ev_type == "edited":
            items = ev.get("items", [])
            if isinstance(items, list):
                for it in items:
                    oid = str(it.get("id", ""))
                    pts = it.get("points", [])
                    if oid and oid in self.permanent_obstacles and isinstance(pts, list) and len(pts) >= 2:
                        self.permanent_obstacles[oid]["points"] = pts
                        changed = True

        elif ev_type == "deleted":
            ids = ev.get("ids", [])
            if isinstance(ids, list):
                for oid in ids:
                    oid = str(oid)
                    if oid in self.permanent_obstacles:
                        del self.permanent_obstacles[oid]
                        changed = True

        if changed:
            self._autosave()
            self._publish_perm()
            self.get_logger().info(f"Permanent obstacles updated. Count={len(self.permanent_obstacles)}")
        else:
            self.get_logger().debug("UI event received but no changes applied.")


def main():
    rclpy.init()
    node = WorldStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
