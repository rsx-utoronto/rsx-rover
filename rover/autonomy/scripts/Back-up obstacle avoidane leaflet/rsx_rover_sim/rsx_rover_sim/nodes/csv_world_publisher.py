#!/usr/bin/env python3
import os
import glob
import json
import csv
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String


def _pick_first(row: Dict[str, Any], keys: List[str]) -> str:
    for k in keys:
        if k in row and row[k] not in (None, ""):
            return row[k]
    return ""


def load_places_to_go(csv_path: str) -> List[Dict[str, Any]]:
    places = []
    if not os.path.exists(csv_path):
        return places

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f, skipinitialspace=True)
        for i, row in enumerate(reader):
            row = { (k or "").strip().lower(): (v or "").strip() for k, v in row.items() }

            lat_s = _pick_first(row, ["lat", "latitude", "y"])
            lon_s = _pick_first(row, ["lng", "lon", "longitude", "x"])

            if not lat_s or not lon_s:
                continue
            try:
                lat = float(lat_s)
                lon = float(lon_s)
            except ValueError:
                continue

            name = _pick_first(row, [
                "name", "label", "place", "title",
                "point_name", "point", "waypoint", "wp",
                "destination", "dest", "location", "loc",
                "tag", "marker", "id"
            ])

            if not name:
                name = f"Place {i+1}"

            places.append({"name": name, "lat": lat, "lon": lon})

    return places


def load_permanent_obstacles(csv_path: str) -> List[Dict[str, Any]]:
    """
    Expected-ish format (based on your existing project):
      id,timestamp,lat,lng,label,obstacle_id,height_cm
    But this loader is tolerant: it searches for lat/lon and obstacle_id-like fields.
    height_cm specifies the obstacle height in centimeters (default 100cm if not specified).
    Obstacles <= 30cm are traversable with a cost penalty.
    """
    obstacles_by_id: Dict[str, Dict[str, Any]] = {}

    if not os.path.exists(csv_path):
        return []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
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
                # fallback: group everything together if no obstacle id exists
                oid = "0"

            label = _pick_first(row, ["label", "type", "category"])
            
            # Parse height_cm (default to 100cm = blocking obstacle)
            height_s = _pick_first(row, ["height_cm", "height", "z", "elevation"])
            try:
                height_cm = float(height_s) if height_s else 100.0
            except ValueError:
                height_cm = 100.0
            
            if oid not in obstacles_by_id:
                obstacles_by_id[oid] = {
                    "id": str(oid), 
                    "label": label, 
                    "height_cm": height_cm,
                    "points": []
                }

            obstacles_by_id[oid]["points"].append({"lat": lat, "lon": lon})

            # if label appears later, keep it
            if label and not obstacles_by_id[oid].get("label"):
                obstacles_by_id[oid]["label"] = label
            
            # Use the max height if multiple points have different heights
            if height_cm > obstacles_by_id[oid].get("height_cm", 0):
                obstacles_by_id[oid]["height_cm"] = height_cm

    # Return in numeric-ish order when possible
    def sort_key(item):
        k = item["id"]
        try:
            return int(k)
        except Exception:
            return k

    return sorted(obstacles_by_id.values(), key=sort_key)


class CsvWorldPublisher(Node):
    def __init__(self):
        super().__init__("csv_world_publisher")

        # Folder in your screenshot:
        default_gui_dir = os.path.expanduser("~/rover_ws/src/rsx-rover/scripts/GUI")
        self.declare_parameter("gui_dir", default_gui_dir)

        self.gui_dir = self.get_parameter("gui_dir").get_parameter_value().string_value
        self.gui_dir = os.path.expanduser(os.path.expandvars(self.gui_dir))

        # "Latched" publisher behavior (ROS2 version): transient local durability
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub_places = self.create_publisher(String, "/world/places_to_go", qos)
        self.pub_perm_obs = self.create_publisher(String, "/world/permanent_obstacles", qos)

        # Publish once immediately, then periodically in case something restarts
        self.timer = self.create_timer(1.0, self.publish_world)

        self.get_logger().info(f"Reading CSVs from: {self.gui_dir}")

    def _latest_file(self, pattern: str) -> str:
        matches = glob.glob(os.path.join(self.gui_dir, pattern))
        if not matches:
            return ""
        matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return matches[0]

    def publish_world(self):
        places_path = os.path.join(self.gui_dir, "places_to_go.csv")
        perm_obs_path = self._latest_file("permanent_obstacles_*.csv")

        places = load_places_to_go(places_path)
        perm_obs = load_permanent_obstacles(perm_obs_path) if perm_obs_path else []

        self.pub_places.publish(String(data=json.dumps({"places": places})))
        self.pub_perm_obs.publish(String(data=json.dumps({"obstacles": perm_obs})))

        self.get_logger().info(
            f"Published: {len(places)} places, {len(perm_obs)} permanent obstacles "
            f"(places_to_go.csv={'YES' if os.path.exists(places_path) else 'NO'}, "
            f"permanent_obstacles={'YES' if bool(perm_obs_path) else 'NO'})"
        )


def main():
    rclpy.init()
    node = CsvWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
